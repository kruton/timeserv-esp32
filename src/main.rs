#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(impl_trait_in_assoc_type)]
#![feature(allocator_api)]

use crate::backlight::{
    Axp192BacklightDevice, BacklightChannel, BacklightConfig, BacklightController, BacklightSystem,
};
#[cfg(feature = "ethernet")]
use crate::ethernet::EthernetTask;
use alloc::vec::Vec;
use axp192::Axp192;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_sync::channel::Channel;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    spi::{
        master::{Config as SpiConfig, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Async,
};
use lcd_async::options::{ColorInversion, ColorOrder};
use lcd_async::{interface, models::ILI9342CRgb565, raw_framebuf::RawFrameBuf, Builder};
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

mod backlight;
#[cfg(feature = "ethernet")]
mod ethernet;
mod pmu;
mod task;

const DISPLAY_FREQ: u32 = 20_000_000;

// Display parameters
const LCD_H_RES: u16 = 320;
const LCD_V_RES: u16 = 240;
const PIXEL_SIZE: usize = 2; // RGB565 = 2 bytes per pixel
const FRAME_SIZE: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize) * PIXEL_SIZE;

static FRAME_BUFFER: StaticCell<Vec<u8, esp_alloc::ExternalMemory>> = StaticCell::new();

pub type Pmu = Mutex<NoopRawMutex, Axp192<I2c<'static, Async>>>;
pub type Spi2Bus = Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>;
//type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, Async>>;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn backlight_task(mut controller: BacklightController<Axp192BacklightDevice>) {
    controller.run().await;
}

#[esp_hal_embassy::main]
async fn main(#[cfg_attr(not(feature = "ethernet"), allow(unused_variables))] spawner: Spawner) {
    info!("starting");

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
    let lcd_dc = peripherals.GPIO15;
    let lcd_cs = peripherals.GPIO5;
    let lcd_bl = peripherals.GPIO3;
    let lcd_rst = peripherals.GPIO4;

    // Initialize I2C components

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();

    static PMU: StaticCell<Pmu> = StaticCell::new();
    let pmu = PMU.init(Mutex::new(Axp192::new(i2c)));
    pmu::m5sc2_init(pmu.get_mut(), &mut delay).await.unwrap();

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

    info!("Display initialized!");

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

    cfg_if::cfg_if! {
        if #[cfg(feature="ethernet")] {
            use esp_hal::rng::Rng;

            let w5500_cs = peripherals.GPIO33;
            let w5500_rst = peripherals.GPIO24;
            let w5500_int = peripherals.GPIO19;

            let ethernet_task = EthernetTask::new();
            let mut rng = Rng::new(peripherals.RNG);

            ethernet_task.init(spawner, &mut rng, w5500_cs, w5500_rst, w5500_int, spi_bus).await;
        }
    }

    static BACKLIGHT_CHANNEL: StaticCell<BacklightChannel> = StaticCell::new();
    let channel = BACKLIGHT_CHANNEL.init(Channel::new());

    let (backlight_system, receiver) = BacklightSystem::new(channel);
    let backlight_device = Axp192BacklightDevice::new(pmu);
    let config = BacklightConfig {
        normal_brightness: 100,
        dimmed_brightness: 20,
        dim_timeout: Duration::from_secs(3),
        off_timeout: Duration::from_secs(2),
    };
    let controller = BacklightController::new(backlight_device, config, receiver);
    spawner.spawn(backlight_task(controller)).unwrap();

    loop {
        Timer::after(Duration::from_secs(8)).await;
        info!("poking the backlight system");
        backlight_system.wake();
    }
}
