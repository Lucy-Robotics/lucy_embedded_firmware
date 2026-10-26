use arduino_hal::hal::port::{PE0, PE1};
use arduino_hal::port::{mode, Pin};

use crate::channel::Pca9685Bank;

pub type Serial = arduino_hal::Usart<
    arduino_hal::pac::USART0,
    Pin<mode::Input, PE0>,
    Pin<mode::Output, PE1>,
>;

pub struct Resources {
    // MANDATORY
    pub serial: Serial,

    // GENERATED
    pub pca: Pca9685Bank<arduino_hal::I2c>,
}

pub fn init() -> Resources {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Host link over the on-board USB-serial chip; matches mcu_bridge's 1M baud.
    let serial = arduino_hal::default_serial!(dp, pins, 1_000_000);

    // Hardware I2C is D20 (SDA) / D21 (SCL). 400 kHz keeps 18 servo writes to ~3 ms of the
    // 25 ms bridge tick (100 kHz would take ~10 ms); the PCA9685 is rated to 1 MHz.
    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.d20.into_pull_up_input(),
        pins.d21.into_pull_up_input(),
        400_000,
    );
    let mut pca = Pca9685Bank { i2c, addresses: &crate::config::PCA_ADDRESSES };
    // A PCA missing now is reported (and set up once it answers) by the per-tick check.
    pca.init().ok();

    // Nervo ribbon servos on Mega pins, pulsed by Timer5.
    crate::servo_pins::init(dp.TC5, mega_servo_pins!(pins));

    Resources { serial, pca }
}
