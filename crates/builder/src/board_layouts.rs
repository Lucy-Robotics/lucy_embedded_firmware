use lucy_embedded_firmware_core::board_layout::{
    parse_adc_channel, parse_servo_channel, parse_uart_channel, BoardLayout, HardwareIdentity,
};
use lucy_embedded_firmware_core::drivers::PCA9685_DEFAULT_ADDR;

#[derive(Debug, Clone, Copy, Default)]
pub struct Rp2040Servo2040Layout;

impl Rp2040Servo2040Layout {
    pub const SERVO_COUNT: u8 = 18;
    pub const ADC_COUNT: u8 = 4;

    pub fn servo_gpio(servo_index: u8) -> Option<u8> {
        if (1..=Self::SERVO_COUNT).contains(&servo_index) {
            Some(servo_index - 1)
        } else {
            None
        }
    }
}

impl BoardLayout for Rp2040Servo2040Layout {
    fn resolve(&self, channel: &str) -> Option<HardwareIdentity> {
        if let Some(idx) = parse_servo_channel(channel) {
            let gpio = Self::servo_gpio(idx)?;
            return Some(HardwareIdentity::PwmGpio {
                servo_index: idx,
                gpio,
            });
        }
        if let Some(adc) = parse_adc_channel(channel) {
            if adc < Self::ADC_COUNT {
                return Some(HardwareIdentity::Adc { channel: adc });
            }
            return None;
        }
        if let Some((uart, device_id)) = parse_uart_channel(channel) {
            return Some(HardwareIdentity::UartBus { uart, device_id });
        }
        let mut parts = channel.split(':');
        let bus = parts.next()?.strip_prefix("I2C")?.parse::<u8>().ok()?;
        let device_name = parts.next()?;
        let ch = parts.next()?.parse::<u8>().ok()?;
        if parts.next().is_some() {
            return None;
        }
        let device = match device_name {
            "PCA9685" => PCA9685_DEFAULT_ADDR,
            _ => return None,
        };
        if ch > 15 {
            return None;
        }
        Some(HardwareIdentity::I2cPwm {
            bus,
            device,
            channel: ch,
        })
    }
}

pub fn layout_for_board(
    board_class: &str,
    firmware_crate: Option<&str>,
) -> Option<&'static dyn BoardLayout> {
    match board_class {
        "internal_servo_only" | "internal_servo_i2c_pwm" | "bus_servo_only" => {
            return Some(&Rp2040Servo2040Layout);
        }
        _ => {}
    }
    if let Some(path) = firmware_crate {
        if path.contains("rp2040_servo2040")
            || path.contains("rp2040_internal_pwm")
            || path.contains("rp2040_i2c_pwm")
        {
            return Some(&Rp2040Servo2040Layout);
        }
    }
    None
}
