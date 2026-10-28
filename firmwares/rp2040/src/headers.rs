//! Hardware-PWM outputs for the Servo2040 headers listed in the generated table.

use embedded_hal::pwm::SetDutyCycle;
use lucy_embedded_firmware_core::joint::{Command, TorqueStatus};
use lucy_embedded_firmware_pwm::pulse::pulse_width_us;
use rp2040_hal::gpio::bank0::{Gpio18, Gpio20, Gpio21};
use rp2040_hal::gpio::{FunctionNull, Pin, PullDown};
use rp2040_hal::{gpio, pwm};

use crate::table::PWM_ROWS;

pub struct KeptPins {
    pub gpio18: Pin<Gpio18, FunctionNull, PullDown>,
    pub gpio20: Pin<Gpio20, FunctionNull, PullDown>,
    pub gpio21: Pin<Gpio21, FunctionNull, PullDown>,
}

macro_rules! bind_pwm {
    ($slices:expr, $pin:expr, $slice:ident, $channel:ident) => {{
        if let Some(pin) = $pin {
            $slices.$slice.$channel.output_to(pin.into_function::<gpio::FunctionPwm>());
            $slices.$slice.set_div_int(125);
            $slices.$slice.set_top(19_999);
            $slices.$slice.enable();
        }
    }};
}

/// Route each header in the table to its PWM slice. UART is not initialized.
pub fn attach(pins: gpio::Pins, slices: &mut pwm::Slices) -> KeptPins {
    let mut used = [false; 18];
    for row in PWM_ROWS {
        if (row.gpio as usize) < used.len() {
            used[row.gpio as usize] = true;
        }
    }
    let gpio::Pins {
        gpio0,
        gpio1,
        gpio2,
        gpio3,
        gpio4,
        gpio5,
        gpio6,
        gpio7,
        gpio8,
        gpio9,
        gpio10,
        gpio11,
        gpio12,
        gpio13,
        gpio14,
        gpio15,
        gpio16,
        gpio17,
        gpio18,
        gpio20,
        gpio21,
        ..
    } = pins;
    if used[0] { bind_pwm!(slices, Some(gpio0), pwm0, channel_a); } else { drop(gpio0); }
    if used[1] { bind_pwm!(slices, Some(gpio1), pwm0, channel_b); } else { drop(gpio1); }
    if used[2] { bind_pwm!(slices, Some(gpio2), pwm1, channel_a); } else { drop(gpio2); }
    if used[3] { bind_pwm!(slices, Some(gpio3), pwm1, channel_b); } else { drop(gpio3); }
    if used[4] { bind_pwm!(slices, Some(gpio4), pwm2, channel_a); } else { drop(gpio4); }
    if used[5] { bind_pwm!(slices, Some(gpio5), pwm2, channel_b); } else { drop(gpio5); }
    if used[6] { bind_pwm!(slices, Some(gpio6), pwm3, channel_a); } else { drop(gpio6); }
    if used[7] { bind_pwm!(slices, Some(gpio7), pwm3, channel_b); } else { drop(gpio7); }
    if used[8] { bind_pwm!(slices, Some(gpio8), pwm4, channel_a); } else { drop(gpio8); }
    if used[9] { bind_pwm!(slices, Some(gpio9), pwm4, channel_b); } else { drop(gpio9); }
    if used[10] { bind_pwm!(slices, Some(gpio10), pwm5, channel_a); } else { drop(gpio10); }
    if used[11] { bind_pwm!(slices, Some(gpio11), pwm5, channel_b); } else { drop(gpio11); }
    if used[12] { bind_pwm!(slices, Some(gpio12), pwm6, channel_a); } else { drop(gpio12); }
    if used[13] { bind_pwm!(slices, Some(gpio13), pwm6, channel_b); } else { drop(gpio13); }
    if used[14] { bind_pwm!(slices, Some(gpio14), pwm7, channel_a); } else { drop(gpio14); }
    if used[15] { bind_pwm!(slices, Some(gpio15), pwm7, channel_b); } else { drop(gpio15); }
    if used[16] { bind_pwm!(slices, Some(gpio16), pwm0, channel_a); } else { drop(gpio16); }
    if used[17] { bind_pwm!(slices, Some(gpio17), pwm0, channel_b); } else { drop(gpio17); }
    KeptPins { gpio18, gpio20, gpio21 }
}

pub fn write(slices: &mut pwm::Slices, commands: &[Command]) {
    for row in PWM_ROWS {
        let command = commands.get(row.virtual_pin).copied().unwrap_or_default();
        let duty = if command.torque_enabled == TorqueStatus::Enabled {
            pulse_width_us(command.position, row.amplitude, row.min_pulse, row.max_pulse)
        } else {
            0
        };
        set_duty(slices, row.gpio, duty);
    }
}

fn set_duty(slices: &mut pwm::Slices, gpio: u8, duty: u16) {
    let result = match gpio {
        0 | 16 => slices.pwm0.channel_a.set_duty_cycle(duty),
        1 | 17 => slices.pwm0.channel_b.set_duty_cycle(duty),
        2 => slices.pwm1.channel_a.set_duty_cycle(duty),
        3 => slices.pwm1.channel_b.set_duty_cycle(duty),
        4 => slices.pwm2.channel_a.set_duty_cycle(duty),
        5 => slices.pwm2.channel_b.set_duty_cycle(duty),
        6 => slices.pwm3.channel_a.set_duty_cycle(duty),
        7 => slices.pwm3.channel_b.set_duty_cycle(duty),
        8 => slices.pwm4.channel_a.set_duty_cycle(duty),
        9 => slices.pwm4.channel_b.set_duty_cycle(duty),
        10 => slices.pwm5.channel_a.set_duty_cycle(duty),
        11 => slices.pwm5.channel_b.set_duty_cycle(duty),
        12 => slices.pwm6.channel_a.set_duty_cycle(duty),
        13 => slices.pwm6.channel_b.set_duty_cycle(duty),
        14 => slices.pwm7.channel_a.set_duty_cycle(duty),
        15 => slices.pwm7.channel_b.set_duty_cycle(duty),
        _ => Ok(()),
    };
    let _ = result;
}
