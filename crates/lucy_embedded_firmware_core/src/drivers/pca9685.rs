//! PCA9685 I2C PWM expander stub (16 channels).
//!
//! Layout resolves `I2C0:PCA9685:N` channels; firmware [`I2cPwmBank`](in
//! `rp2040_support`) processes Modbus until this driver is wired to a real
//! [`crate::i2c::I2cChannel`].

/// Default PCA9685 I2C 7-bit address.
pub const PCA9685_DEFAULT_ADDR: u8 = 0x40;

/// Register block size matches on-board PWM servos (cmd + pulse).
pub const PCA9685_PWM_REGS: u16 = 2;

/// Placeholder config shared with [`super::PwmServoConfig`] at the Modbus layer.
/// Hardware init (prescale / mode) is not performed until I2C is connected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pca9685Config {
    pub address: u8,
    pub channel: u8,
}

impl Pca9685Config {
    pub const fn new(channel: u8) -> Self {
        Self {
            address: PCA9685_DEFAULT_ADDR,
            channel,
        }
    }
}
