#![no_std]
#![no_main]

use {defmt as _, esp_println as _};

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use esp_hal::timer::timg::TimerGroup;

    #[init]
    fn init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let timer0 = TimerGroup::new(peripherals.TIMG1);
        esp_hal_embassy::init(timer0.timer0);
    }

    #[test]
    async fn hello_test() {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        assert_eq!(1 + 1, 2);
    }
}
