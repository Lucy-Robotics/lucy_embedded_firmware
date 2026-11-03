//! Lists the I2C devices on D20/D21 every 2 s at 57600 baud (PCA9685s answer at 0x40..0x7F).
//! A stuck bus shows as a "scan" line with no table after it.

#![no_std]
#![no_main]

use arduino_hal::prelude::*;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.d20.into_pull_up_input(),
        pins.d21.into_pull_up_input(),
        100_000,
    );
    loop {
        for &b in b"scan\r\n" {
            serial.write_byte(b);
        }
        i2c.i2cdetect(&mut serial, arduino_hal::i2c::Direction::Write).unwrap_infallible();
        arduino_hal::delay_ms(2000);
    }
}
