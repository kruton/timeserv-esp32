#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(impl_trait_in_assoc_type)]
#![feature(allocator_api)]

use crate::task::web;
use crate::task::web::{web_task, WEB_TASK_POOL_SIZE};
use alloc::vec::Vec;
use axp192::Axp192;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources};
use embassy_net_wiznet::{chip::W5500, Device, Runner, State};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Duration;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::Write;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    rng::Rng,
    spi::{
        master::{Config as SpiConfig, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Async,
};
use esp_println::println;
use lcd_async::options::{ColorInversion, ColorOrder};
use lcd_async::{interface, models::ILI9342CRgb565, raw_framebuf::RawFrameBuf, Builder};
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

mod task;

const DISPLAY_FREQ: u32 = 20_000_000;
const NET_FREQ: u32 = 40_000_000;

// Display parameters
const LCD_H_RES: u16 = 320;
const LCD_V_RES: u16 = 240;
const PIXEL_SIZE: usize = 2; // RGB565 = 2 bytes per pixel
const FRAME_SIZE: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize) * PIXEL_SIZE;

static FRAME_BUFFER: StaticCell<Vec<u8, esp_alloc::ExternalMemory>> = StaticCell::new();

type Spi2Bus = Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>;
//type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, Async>>;
type EthernetSPI =
    SpiDeviceWithConfig<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>;

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
    println!("starting");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let mut delay = embassy_time::Delay;

    // i2c
    let sda = peripherals.GPIO21;
    let scl = peripherals.GPIO22;
    // spi
    let spi = peripherals.SPI2;
    let sck = peripherals.GPIO18;
    let miso = peripherals.GPIO38;
    let mosi = peripherals.GPIO23;
    let dma = peripherals.DMA_SPI2;
    let w5500_cs = peripherals.GPIO33;
    let w5500_rst = peripherals.GPIO24;
    let w5500_int = peripherals.GPIO19;
    let lcd_dc = peripherals.GPIO15;
    let lcd_cs = peripherals.GPIO5;
    let lcd_bl = peripherals.GPIO3;
    let lcd_rst = peripherals.GPIO4;

    // Initialize I2C components

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    let mut pmu = Axp192::new(i2c);
    m5sc2_init(&mut pmu, &mut delay).await.unwrap();

    // Shared SPI bus

    let spi_cfg = SpiConfig::default()
        .with_frequency(Rate::from_mhz(20))
        .with_mode(Mode::_0);

    #[allow(clippy::manual_div_ceil)]
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, 32_000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(spi, spi_cfg)
        .unwrap()
        .with_sck(sck)
        .with_miso(miso)
        .with_mosi(mosi)
        .with_dma(dma)
        .with_buffers(dma_rx_buf, dma_tx_buf)
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

    // Configure the LCD display
    let display_config = SpiConfig::default().with_frequency(Rate::from_hz(DISPLAY_FREQ));

    let display_spi = SpiDeviceWithConfig::new(
        spi_bus,
        Output::new(lcd_cs, Level::High, OutputConfig::default()),
        display_config,
    );

    let lcd_rst = Output::new(lcd_rst, Level::Low, OutputConfig::default());

    let _bl = Output::new(lcd_bl, Level::High, OutputConfig::default());

    let dc_lcd = Output::new(lcd_dc, Level::Low, OutputConfig::default());
    let di = interface::SpiInterface::new(display_spi, dc_lcd);

    let mut display = Builder::new(ILI9342CRgb565, di)
        .reset_pin(lcd_rst)
        .display_size(LCD_H_RES, LCD_V_RES)
        .color_order(ColorOrder::Bgr)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
        .unwrap();

    println!("Display initialized!");

    // Initialize frame buffer
    let frame_buffer =
        FRAME_BUFFER.init(Vec::with_capacity_in(FRAME_SIZE, esp_alloc::ExternalMemory));
    frame_buffer.resize(FRAME_SIZE, 0);

    // Create a framebuffer for drawing
    let mut raw_fb = RawFrameBuf::<Rgb565, _>::new(
        frame_buffer.as_mut_slice(),
        LCD_H_RES.into(),
        LCD_V_RES.into(),
    );

    // Clear the framebuffer to black
    raw_fb.clear(Rgb565::BLACK).unwrap();
    Circle::new(Point::new(0, 0), 80)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
        .draw(&mut raw_fb)
        .unwrap();

    // Send the framebuffer data to the display
    display
        .show_raw_data(0, 0, LCD_H_RES, LCD_V_RES, frame_buffer.as_slice())
        .await
        .unwrap();

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

    println!("Waiting for DHCP...");
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    println!("IP address: {:?}", local_addr);

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
        println!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            println!("accept error: {:?}", e);
            continue;
        }
        println!("Received connection from {:?}", socket.remote_endpoint());

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    println!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    println!("{:?}", e);
                    break;
                }
            };
            println!("rxd {}", core::str::from_utf8(&buf[..n]).unwrap());

            if let Err(e) = socket.write_all(&buf[..n]).await {
                println!("write error: {:?}", e);
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

async fn m5sc2_init<I2C, E>(
    axp: &mut axp192::Axp192<I2C>,
    delay: &mut embassy_time::Delay,
) -> Result<(), E>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    // Default setup for M5Stack Core 2
    axp.set_dcdc1_voltage(3350)?; // Voltage to provide to the microcontroller (this one!)

    axp.set_ldo2_voltage(3300)?; // Peripherals (LCD, ...)
    axp.set_ldo2_on(true)?;

    axp.set_ldo3_voltage(2000)?; // Vibration motor
    axp.set_ldo3_on(false)?;

    axp.set_dcdc3_voltage(2800)?; // LCD backlight
    axp.set_dcdc3_on(true)?;

    axp.set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Power LED
    axp.set_gpio1_output(false)?; // In open drain modes, state is opposite to what you might
                                  // expect

    axp.set_gpio2_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Speaker
    axp.set_gpio2_output(true)?;

    axp.set_key_mode(
        // Configure how the power button press will work
        axp192::ShutdownDuration::Sd4s,
        axp192::PowerOkDelay::Delay64ms,
        true,
        axp192::LongPress::Lp1000ms,
        axp192::BootTime::Boot512ms,
    )?;

    axp.set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput)?; // LCD reset control

    axp.set_battery_voltage_adc_enable(true)?;
    axp.set_battery_current_adc_enable(true)?;
    axp.set_acin_current_adc_enable(true)?;
    axp.set_acin_voltage_adc_enable(true)?;

    // Actually reset the LCD
    axp.set_gpio4_output(false)?;
    axp.set_ldo3_on(true)?; // Buzz the vibration motor while intializing ¯\_(ツ)_/¯
    delay.delay_ms(100u32).await;
    axp.set_gpio4_output(true)?;
    axp.set_ldo3_on(false)?;
    delay.delay_ms(100u32).await;

    Ok(())
}
