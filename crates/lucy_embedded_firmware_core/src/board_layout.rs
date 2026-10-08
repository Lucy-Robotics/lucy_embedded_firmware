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

/// Parse `Servo1`..`Servo18` style channel names (1-based index).
pub fn parse_servo_channel(channel: &str) -> Option<u8> {
    let rest = channel.strip_prefix("Servo")?;
    let idx = rest.parse::<u8>().ok()?;
    if idx == 0 {
        None
    } else {
        Some(idx)
    }
}

/// Pimoroni Servo2040: on-board PWM (`ServoN` → GPIO `N-1`), ADC0..3,
/// optional UART and I2C/PCA9685 channels. Used for both
/// `internal_servo_only` and `internal_servo_i2c_pwm` board classes.
#[derive(Debug, Clone, Copy, Default)]
pub struct Rp2040Servo2040Layout;

impl Rp2040Servo2040Layout {
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

/// Alias kept for older call sites / docs.
pub type Rp2040InternalPwmLayout = Rp2040Servo2040Layout;
/// Alias: I2C-capable profile uses the same Servo2040 layout.
pub type Rp2040I2cPwmLayout = Rp2040Servo2040Layout;

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

/// Select a layout by `board_class` or `firmware_crate` path fragment.
///
/// Prefer `board_class` when set so `bus_servo_only` keeps the UART-only
/// channel map even though it builds the shared `rp2040_servo2040` crate.
pub fn layout_for_board(
    board_class: &str,
    firmware_crate: Option<&str>,
) -> Option<&'static dyn BoardLayout> {
    match board_class {
        "internal_servo_only" | "internal_servo_i2c_pwm" => {
            return Some(&Rp2040Servo2040Layout);
        }
        "bus_servo_only" => return Some(&Rp2040BusServoLayout),
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
