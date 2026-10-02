#![no_std]
#![no_main]

use embedded_hal::pwm;
use panic_halt as _;

mod channel;
use channel::{*};

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let _servo_pin = pins.d11.into_output();

    // let mut servo = ServoTimer1::new(dp.TC1);

    let mut pwm_channel = ArduinoMega2560PwmChannel::new(dp.TC1);
    pwm_channel.release();


    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);

    
loop {
    // Prompt
    for byte in b"Angle (0-180): " {
        serial.write_byte(*byte);
    }

    // Read a line
    let mut angle: u16 = 0;

    loop {
        let byte = serial.read_byte();

        match byte {
            b'0'..=b'9' => {
                angle = angle
                    .saturating_mul(10)
                    .saturating_add((byte - b'0') as u16);

                // Echo character
                serial.write_byte(byte);
            }

            b'\r' | b'\n' => {
                // Enter pressed
                break;
            }

            8 | 127 => {
                // Backspace
                // For a simple implementation, ignore it.
            }

            _ => {
                // Ignore anything else
            }
        }
    }

    // Clamp to valid servo range
    angle = angle.min(180);

    pwm_channel.set_angle(angle);

    // Report what we did
    for byte in b"\r\nServo angle set!\r\n\r\n" {
        serial.write_byte(*byte);
    }
}
}