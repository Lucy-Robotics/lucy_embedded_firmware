use arduino_hal::hal::port::{PE0, PE1};
use arduino_hal::hal::wdt;
use arduino_hal::port::{mode, Pin};

use crate::channel::Pca9685Bank;

pub type Serial = arduino_hal::Usart<
    arduino_hal::pac::USART0,
    Pin<mode::Input, PE0>,
    Pin<mode::Output, PE1>,
>;

pub struct Resources {
    pub serial: Serial,

    pub pca: Pca9685Bank<arduino_hal::I2c>,

    pub clock: arduino_hal::pac::TC1,

    pub watchdog: wdt::Wdt,
}

pub fn init() -> Resources {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // a wedged I2C bus never returns: reset, PCA init turns the outputs off
    let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr());
    watchdog.start(wdt::Timeout::Ms500).unwrap();

    let serial = arduino_hal::default_serial!(dp, pins, 1_000_000);

    let mut scl = pins.d21.into_output_high();
    for _ in 0..9 {
        scl.set_low();
        arduino_hal::delay_us(5);
        scl.set_high();
        arduino_hal::delay_us(5);
    }

    // 100 kHz takes ~10 ms of the 25 ms tick
    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.d20.into_pull_up_input(),
        scl.into_pull_up_input(),
        400_000,
    );
    let mut pca = Pca9685Bank { i2c, addresses: &crate::config::PCA_ADDRESSES };
    pca.init().ok();

    crate::servo_pins::init(dp.TC5, mega_servo_pins!(pins));

    dp.TC1.tccr1b().write(|w| w.cs1().prescale_1024());

    Resources { serial, pca, clock: dp.TC1, watchdog }
}
