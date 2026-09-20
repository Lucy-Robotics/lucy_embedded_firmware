//! Channel-name → hardware-identity mapping owned by each board class.
//!
//! Channel names in firmware YAML (`Servo10`, `ADC0`, `UART0:1`) are resolved
//! through a [`BoardLayout`] before the builder assigns Modbus register blocks.

/// Resolved hardware resource for a named channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareIdentity {
    /// On-board PWM servo pad. `servo_index` is 1-based (`Servo1` → 1).
    PwmGpio { servo_index: u8, gpio: u8 },
    /// On-board ADC input (`ADC0` → channel 0).
    Adc { channel: u8 },
    /// UART bus-servo lane; optional device id after `:` (`UART0:1`).
    UartBus { uart: u8, device_id: Option<u8> },
    /// I2C PWM expander channel (`I2C0:PCA9685:3`).
    I2cPwm {
        bus: u8,
        device: u8,
        channel: u8,
    },
}

/// Maps architecture channel names to concrete hardware identities.
pub trait BoardLayout {
    fn resolve(&self, channel: &str) -> Option<HardwareIdentity>;
}

/// Parse `UART0` / `UART0:3` style channel names.
pub fn parse_uart_channel(channel: &str) -> Option<(u8, Option<u8>)> {
    let rest = channel.strip_prefix("UART")?;
    let (uart_str, device) = match rest.split_once(':') {
        Some((u, d)) => (u, Some(d.parse::<u8>().ok()?)),
        None => (rest, None),
    };
    let uart = uart_str.parse::<u8>().ok()?;
    Some((uart, device))
}

/// Parse `ADC0`..`ADC3` style channel names.
pub fn parse_adc_channel(channel: &str) -> Option<u8> {
    let rest = channel.strip_prefix("ADC")?;
    rest.parse::<u8>().ok()
}

/// Parse `Servo1`..`Servo16` style channel names (1-based index).
pub fn parse_servo_channel(channel: &str) -> Option<u8> {
    let rest = channel.strip_prefix("Servo")?;
    let idx = rest.parse::<u8>().ok()?;
    if idx == 0 {
        None
    } else {
        Some(idx)
    }
}

/// RP2040 internal PWM / Pimoroni Servo2040: `ServoN` → GPIO `N-1`
/// (`Servo1`→GPIO0 … `Servo18`→GPIO17), matching the board silk numbers.
#[derive(Debug, Clone, Copy, Default)]
pub struct Rp2040InternalPwmLayout;

impl Rp2040InternalPwmLayout {
    pub const SERVO_COUNT: u8 = 18;
    pub const ADC_COUNT: u8 = 4;

    /// GPIO for 1-based servo / silk index, or `None` if out of table.
    pub fn servo_gpio(servo_index: u8) -> Option<u8> {
        if (1..=Self::SERVO_COUNT).contains(&servo_index) {
            Some(servo_index - 1)
        } else {
            None
        }
    }
}

impl BoardLayout for Rp2040InternalPwmLayout {
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
        None
    }
}

/// Bus-servo board: UART lanes + optional device ids.
#[derive(Debug, Clone, Copy, Default)]
pub struct Rp2040BusServoLayout;

impl BoardLayout for Rp2040BusServoLayout {
    fn resolve(&self, channel: &str) -> Option<HardwareIdentity> {
        if let Some((uart, device_id)) = parse_uart_channel(channel) {
            return Some(HardwareIdentity::UartBus { uart, device_id });
        }
        None
    }
}

/// Internal PWM + I2C expander board (stub layout).
#[derive(Debug, Clone, Copy, Default)]
pub struct Rp2040I2cPwmLayout;

impl BoardLayout for Rp2040I2cPwmLayout {
    fn resolve(&self, channel: &str) -> Option<HardwareIdentity> {
        // Reuse internal PWM servo/ADC map, then I2C expander channels.
        if let Some(id) = Rp2040InternalPwmLayout.resolve(channel) {
            return Some(id);
        }
        // I2C0:PCA9685:N  (N = 0..15)
        let mut parts = channel.split(':');
        let bus = parts.next()?.strip_prefix("I2C")?.parse::<u8>().ok()?;
        let device_name = parts.next()?;
        let ch = parts.next()?.parse::<u8>().ok()?;
        if parts.next().is_some() {
            return None;
        }
        let device = match device_name {
            "PCA9685" => 0x40,
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

/// Select a layout by `board_class` or `firmware_crate` path fragment.
pub fn layout_for_board(board_class: &str, firmware_crate: Option<&str>) -> Option<&'static dyn BoardLayout> {
    if let Some(path) = firmware_crate {
        if path.contains("rp2040_internal_pwm") {
            return Some(&Rp2040InternalPwmLayout);
        }
        if path.contains("rp2040_bus_servo") {
            return Some(&Rp2040BusServoLayout);
        }
        if path.contains("rp2040_i2c_pwm") {
            return Some(&Rp2040I2cPwmLayout);
        }
    }
    match board_class {
        "internal_servo_only" => Some(&Rp2040InternalPwmLayout),
        "bus_servo_only" => Some(&Rp2040BusServoLayout),
        "internal_servo_i2c_pwm" => Some(&Rp2040I2cPwmLayout),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn internal_pwm_servo_gpio_table() {
        let layout = Rp2040InternalPwmLayout;
        assert_eq!(
            layout.resolve("Servo1"),
            Some(HardwareIdentity::PwmGpio {
                servo_index: 1,
                gpio: 0
            })
        );
        assert_eq!(
            layout.resolve("Servo6"),
            Some(HardwareIdentity::PwmGpio {
                servo_index: 6,
                gpio: 5
            })
        );
        assert_eq!(
            layout.resolve("Servo18"),
            Some(HardwareIdentity::PwmGpio {
                servo_index: 18,
                gpio: 17
            })
        );
        assert_eq!(layout.resolve("Servo19"), None);
        assert_eq!(layout.resolve("Servo0"), None);
    }

    #[test]
    fn internal_pwm_adc_and_unknown() {
        let layout = Rp2040InternalPwmLayout;
        assert_eq!(
            layout.resolve("ADC0"),
            Some(HardwareIdentity::Adc { channel: 0 })
        );
        assert_eq!(
            layout.resolve("ADC3"),
            Some(HardwareIdentity::Adc { channel: 3 })
        );
        assert_eq!(layout.resolve("ADC4"), None);
        assert_eq!(layout.resolve("NotAChannel"), None);
    }

    #[test]
    fn uart_and_i2c_channels() {
        assert_eq!(
            Rp2040BusServoLayout.resolve("UART0:1"),
            Some(HardwareIdentity::UartBus {
                uart: 0,
                device_id: Some(1)
            })
        );
        assert_eq!(
            Rp2040I2cPwmLayout.resolve("I2C0:PCA9685:3"),
            Some(HardwareIdentity::I2cPwm {
                bus: 0,
                device: 0x40,
                channel: 3
            })
        );
    }
}
