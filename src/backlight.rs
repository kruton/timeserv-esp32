use axp192::Axp192;
use defmt::info;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
    mutex::Mutex,
};
use embassy_time::{with_timeout, Duration, TimeoutError};
use esp_hal::{i2c::master::I2c, Async};

pub type BacklightChannel = Channel<NoopRawMutex, BacklightCommand, 2>;

pub trait BacklightDevice {
    async fn set_brightness(&mut self, brightness: u8);

    async fn set_backlight_on(&mut self, on: bool);
}

impl<T> BacklightDevice for &mut T
where
    T: BacklightDevice + ?Sized,
{
    #[inline]
    async fn set_brightness(&mut self, brightness: u8) {
        T::set_brightness(self, brightness).await
    }

    #[inline]
    async fn set_backlight_on(&mut self, on: bool) {
        T::set_backlight_on(self, on).await
    }
}

#[derive(Debug, Clone, Copy)]
pub enum BacklightCommand {
    Wake,
}

#[derive(Debug, Clone, Copy)]
pub struct BacklightConfig {
    /// Normal brightness level (0-100%)
    pub normal_brightness: u8,
    /// Dimmed brightness level (0-100%)
    pub dimmed_brightness: u8,
    /// Time before dimming starts
    pub dim_timeout: Duration,
    /// Time before turning off after dimming
    pub off_timeout: Duration,
}

impl Default for BacklightConfig {
    fn default() -> Self {
        Self {
            normal_brightness: 80,
            dimmed_brightness: 20,
            dim_timeout: Duration::from_secs(30),
            off_timeout: Duration::from_secs(10),
        }
    }
}

pub struct BacklightController<U: BacklightDevice> {
    device: U,
    config: BacklightConfig,
    command_receiver: Receiver<'static, NoopRawMutex, BacklightCommand, 2>,
}

impl<U: BacklightDevice> BacklightController<U> {
    pub fn new(
        device: U,
        config: BacklightConfig,
        command_receiver: Receiver<'static, NoopRawMutex, BacklightCommand, 2>,
    ) -> Self {
        Self {
            device,
            config,
            command_receiver,
        }
    }

    pub async fn run(&mut self) {
        let _ = self.device.set_backlight_on(true).await;
        let _ = self
            .device
            .set_brightness(self.config.normal_brightness)
            .await;

        loop {
            // Wait for dim timeout or command
            match with_timeout(self.config.dim_timeout, self.command_receiver.receive()).await {
                Ok(command) => {
                    match command {
                        BacklightCommand::Wake => {
                            // Reset to normal brightness
                            let _ = self.device.set_backlight_on(true).await;
                            let _ = self
                                .device
                                .set_brightness(self.config.normal_brightness)
                                .await;
                        }
                    }
                }
                Err(TimeoutError) => {
                    let _ = self
                        .device
                        .set_brightness(self.config.dimmed_brightness)
                        .await;

                    // Wait for off timeout or wake command
                    match with_timeout(self.config.off_timeout, self.command_receiver.receive())
                        .await
                    {
                        Ok(BacklightCommand::Wake) => {
                            let _ = self
                                .device
                                .set_brightness(self.config.normal_brightness)
                                .await;
                        }
                        Err(TimeoutError) => {
                            let _ = self.device.set_backlight_on(false).await;

                            // Wait indefinitely for wake command
                            match self.command_receiver.receive().await {
                                BacklightCommand::Wake => {
                                    let _ = self.device.set_backlight_on(true).await;
                                    let _ = self
                                        .device
                                        .set_brightness(self.config.normal_brightness)
                                        .await;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

pub struct BacklightSystem {
    command_sender: Sender<'static, NoopRawMutex, BacklightCommand, 2>,
}

impl BacklightSystem {
    pub fn new(
        channel: &'static BacklightChannel,
    ) -> (Self, Receiver<'static, NoopRawMutex, BacklightCommand, 2>) {
        let sender = channel.sender();
        let receiver = channel.receiver();
        (
            Self {
                command_sender: sender,
            },
            receiver,
        )
    }

    /// Wake up the backlight (interrupt dimming/turn on)
    pub fn wake(&self) {
        let _ = self.command_sender.try_send(BacklightCommand::Wake);
    }
}

/// Axp192 backlight device implementation using DCDC3 voltage control
pub struct Axp192BacklightDevice {
    pmu: &'static Mutex<NoopRawMutex, Axp192<I2c<'static, Async>>>,
}

impl Axp192BacklightDevice {
    /// Create a new Axp192BacklightDevice
    pub fn new(pmu: &'static Mutex<NoopRawMutex, Axp192<I2c<'static, Async>>>) -> Self {
        Self { pmu }
    }

    /// Convert brightness percentage (0-100) to voltage (2500-3300)
    fn brightness_to_voltage(brightness: u8) -> u16 {
        let brightness = brightness.min(100) as u32;
        let voltage = 2500u32 + (brightness * 800u32) / 100u32;
        voltage as u16
    }
}

impl BacklightDevice for Axp192BacklightDevice {
    async fn set_brightness(&mut self, brightness: u8) {
        let voltage = Self::brightness_to_voltage(brightness);
        info!(
            "Setting brightness {}% as voltage to {} mV",
            brightness, voltage
        );
        let _ = self.pmu.lock().await.set_dcdc3_voltage(voltage);
    }

    async fn set_backlight_on(&mut self, on: bool) {
        let _ = self.pmu.lock().await.set_dcdc3_on(on);
    }
}

// Example usage:
//
// #[embassy_executor::task]
// async fn backlight_task(
//     mut controller: BacklightController<Axp192BacklightDevice>
// ) {
//     controller.run().await;
// }
//
// In your main function:
// static BACKLIGHT_CHANNEL: StaticCell<BacklightChannel> = StaticCell::new();
// let channel = BACKLIGHT_CHANNEL.init(Channel::new());
//
// let (backlight_system, receiver) = BacklightSystem::new(channel);
// let backlight_device = Axp192BacklightDevice::new(pmu);
// let config = BacklightConfig::default();
// let controller = BacklightController::new(backlight_device, config, receiver);
//
// spawner.spawn(backlight_task(controller)).unwrap();
//
// From touchscreen interrupt or other tasks:
// backlight_system.wake(); // Wake up the screen
