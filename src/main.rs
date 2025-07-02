#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(impl_trait_in_assoc_type)]

use crate::task::web;
use crate::task::web::{web_task, WEB_TASK_POOL_SIZE};
use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources};
use embassy_net_wiznet::{chip::W5500, Device, Runner, State};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Duration;
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    //dma::{DmaRxBuf, DmaTxBuf},
    //dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::I2c,
    rng::Rng,
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::Rate,
    Async,
};
use esp_println as _;
//use mipidsi::{interface::SpiInterface, models::ILI9342CRgb565, Builder};
use mipidsi::interface::SpiInterface;
use static_cell::StaticCell;

mod task;

const DISPLAY_FREQ: u32 = 64_000_000;
const NET_FREQ: u32 = 40_000_000;

type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async>>;
// type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, Async>>;
type EthernetSPI = SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, Async>, Output<'static>>;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

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
    info!("starting");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    // i2c
    let sda = peripherals.GPIO21;
    let scl = peripherals.GPIO22;
    // spi
    let spi = peripherals.SPI2;
    let sck = peripherals.GPIO18;
    let miso = peripherals.GPIO38;
    let mosi = peripherals.GPIO23;
    let _dma = peripherals.DMA_SPI2;
    let w5500_cs = peripherals.GPIO33;
    let w5500_rst = peripherals.GPIO24;
    let w5500_int = peripherals.GPIO19;
    let lcd_dc = peripherals.GPIO15;
    let lcd_cs = peripherals.GPIO5;
    let lcd_bl = peripherals.GPIO3;
    let lcd_rst = peripherals.GPIO4;

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let _i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    // Shared SPI bus

    let spi_cfg = SpiConfig::default()
        .with_frequency(Rate::from_mhz(40))
        .with_mode(Mode::_0);

    //let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    //let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    //let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(spi, spi_cfg)
        .unwrap()
        .with_sck(sck)
        .with_miso(miso)
        .with_mosi(mosi)
        //.with_dma(dma)
        //.with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

    static SPI_BUS: StaticCell<Spi2Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    // Configure the W5500 Ethernet chip
    let net_config = SpiConfig::default()
        .with_frequency(Rate::from_hz(NET_FREQ))
        .with_mode(Mode::_0);
    let spi_net = SpiDeviceWithConfig::new(
        spi_bus,
        Output::new(w5500_cs, Level::High, OutputConfig::default()),
        net_config,
    );

    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00];
    static STATE: StaticCell<State<8, 8>> = StaticCell::new();
    let state = STATE.init(State::<8, 8>::new());

    let net_int = Input::new(w5500_int, InputConfig::default().with_pull(Pull::Up));
    let net_rst = Output::new(w5500_rst, Level::High, OutputConfig::default());
    let (device, runner) = embassy_net_wiznet::new(mac_addr, state, spi_net, net_int, net_rst)
        .await
        .unwrap();
    spawner.spawn(ethernet_task(runner)).unwrap();

    check_is_spi_bus(spi_bus.get_mut());
    // Configure the LCD display
    let mut display_config = SpiConfig::default().with_frequency(Rate::from_hz(DISPLAY_FREQ));

    let display_spi = SpiDeviceWithConfig::new(
        spi_bus,
        Output::new(lcd_cs, Level::High, OutputConfig::default()),
        display_config,
    );

    //let lcd_dc = Output::new(lcd_dc, Level::Low, OutputConfig::default());
    // let lcd_rst = Output::new(lcd_rst, Level::Low, OutputConfig::default());

    let _bl = Output::new(lcd_bl, Level::High, OutputConfig::default());

    static BUF_LCD: StaticCell<[u8; 512]> = StaticCell::new();
    let mut buf_lcd = BUF_LCD.init([0; 512]);
    let dc_lcd = Output::new(lcd_dc, Level::Low, OutputConfig::default());
    let _di = SpiInterface::new(display_spi, dc_lcd, buf_lcd);

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
    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
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

    static WEB_SERVER: StaticCell<web::WebServer> = StaticCell::new();
    let webserver = WEB_SERVER.init(web::WebServer::new());
    for id in 0..WEB_TASK_POOL_SIZE {
        spawner.must_spawn(web_task(id, stack, webserver));
    }

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

fn check_is_spi_bus(_bus: &impl embedded_hal_async::spi::SpiBus) {}
