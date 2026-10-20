use arduino_hal::hal::port::{PE0, PE1};
use arduino_hal::port::{mode, Pin};

pub type Serial = arduino_hal::Usart<
    arduino_hal::pac::USART0,
    Pin<mode::Input, PE0>,
    Pin<mode::Output, PE1>,
>;

/// Nervo board: PCA9685 channel 6. Bare Mega: servo on D6.
#[cfg(feature = "nervo")]
pub type Servo6Channel = crate::channel::Pca9685Channel<arduino_hal::I2c>;
#[cfg(not(feature = "nervo"))]
pub type Servo6Channel = crate::channel::MegaPwmChannel;

pub struct Resources {
    // MANDATORY
    pub serial: Serial,

    // GENERATED
    pub servo6: Servo6Channel,
}

pub fn init() -> Resources {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Host link over the on-board USB-serial chip; matches mcu_bridge's 1M baud.
    let serial = arduino_hal::default_serial!(dp, pins, 1_000_000);

    #[cfg(feature = "nervo")]
    let servo6 = {
        // Hardware I2C is D20 (SDA) / D21 (SCL); the Nervo board PCA9685 answers at 0x41.
        let i2c = arduino_hal::I2c::new(
            dp.TWI,
            pins.d20.into_pull_up_input(),
            pins.d21.into_pull_up_input(),
            100_000,
        );
        let mut servo6 = crate::channel::Pca9685Channel { i2c, address: 0x41, channel: 6 };
        // shortcut: a missing PCA only shows as a jaw that never moves.
        // Upgrade to reporting init/write errors to the host once states carry diagnostics.
        servo6.init().ok();
        servo6
    };

    #[cfg(not(feature = "nervo"))]
    let servo6 = {
        // Output low while OC4A is disconnected, so the servo line idles low, not floating.
        pins.d6.into_output();
        crate::channel::MegaPwmChannel::new(dp.TC4)
    };

    Resources { serial, servo6 }
}
