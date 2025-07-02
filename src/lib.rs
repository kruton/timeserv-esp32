#![no_std]
#![no_main]

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources};
use embassy_net_wiznet::{chip::W5500, Device, Runner, State};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Duration;
use embedded_io_async::Write;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::I2c,
    rng::Rng,
    spi::{
        master::{Config as SpiConfig, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    timer::timg::TimerGroup,
    Async,
};
use log::{info, warn};
// use mipidsi::{interface::SpiInterface, models::ILI9342CRgb565, Builder};
use static_cell::StaticCell;

type Spi2Bus = Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>;
// type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, Async>>;

type EthernetSPI = SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>;
#[embassy_executor::task]
async fn ethernet_task(
    runner: Runner<'static, W5500, EthernetSPI, Input<'static>, Output<'static>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device<'static>>) -> ! {
    runner.run().await
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    log::info!("starting");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let _i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_hal_embassy::init(timg0.timer0);

    // Shared SPI bus

    let spi_cfg = SpiConfig::default()
        .with_frequency(Rate::from_mhz(40))
        .with_mode(Mode::_0);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(peripherals.SPI2, spi_cfg)
        .unwrap()
        .with_sck(peripherals.GPIO18)
        .with_miso(peripherals.GPIO38)
        .with_mosi(peripherals.GPIO23)
        .with_dma(peripherals.DMA_SPI2)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

    static SPI_BUS: StaticCell<Spi2Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    // Configure the W5500 Ethernet chip
    let cs_net: Output<'_> = Output::new(peripherals.GPIO33, Level::High, OutputConfig::default());
    let spi_net = SpiDevice::new(spi_bus, cs_net);

    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00];
    static STATE: StaticCell<State<8, 8>> = StaticCell::new();
    let state = STATE.init(State::<8, 8>::new());

    let w5500_int = Input::new(
        peripherals.GPIO19,
        InputConfig::default().with_pull(Pull::Up),
    );
    let w5500_rst = Output::new(peripherals.GPIO24, Level::High, OutputConfig::default());
    let (device, runner) = embassy_net_wiznet::new(mac_addr, state, spi_net, w5500_int, w5500_rst)
        .await
        .unwrap();
    spawner.spawn(ethernet_task(runner)).unwrap();

    // Configure the LCD display
    // let dc_lcd = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    // let cs_lcd = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    // let spi_lcd = SpiDevice::new(spi_bus, cs_lcd);

    // static BUF_LCD: StaticCell<[u8; 512]> = StaticCell::new();
    // let mut buf_lcd = BUF_LCD.init([0; 512]);
    // let di = SpiInterface::new(spi_lcd, dc_lcd, buf_lcd);

    // let lcd = Builder::new(ILI9342CRgb565, di);
    // .display_size(crate::LCD_H_RES as u16, crate::LCD_V_RES as u16)
    // .orientation(Orientation::new().flip_vertical().flip_horizontal())
    // .color_order(mipidsi::options::ColorOrder::Bgr)
    // .reset_pin(Output::new(
    //     peripherals.GPIO4,
    //     Level::High,
    //     OutputConfig::default(),
    // ))
    // .build()
    // .await
    // .unwrap();

    // let lcd_di = lcd_display_interface!(peripherals, spi_bus);
    // Generate a random seed for the network stack
    let mut rng = Rng::new(peripherals.RNG);
    let mut seed = [0; 8];
    rng.read(&mut seed);
    let seed = u64::from_le_bytes(seed);

    // Init network stack
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(net_task(runner)).unwrap();

    stack.wait_config_up().await;

    info!("Waiting for DHCP...");
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    info!("IP address: {:?}", local_addr);

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];
    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            warn!("accept error: {:?}", e);
            continue;
        }
        info!("Received connection from {:?}", socket.remote_endpoint());

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("{:?}", e);
                    break;
                }
            };
            info!("rxd {}", core::str::from_utf8(&buf[..n]).unwrap());

            if let Err(e) = socket.write_all(&buf[..n]).await {
                warn!("write error: {:?}", e);
                break;
            }
        }
    }
}

async fn wait_for_config(stack: Stack<'static>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config.clone();
        }
        yield_now().await;
    }
}
