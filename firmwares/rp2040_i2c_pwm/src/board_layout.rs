//! Board layout for `rp2040_i2c_pwm` (onboard PWM + PCA9685).
pub use lucy_embedded_firmware_core::board_layout::{
    BoardLayout, HardwareIdentity, Rp2040I2cPwmLayout,
};

pub type Layout = Rp2040I2cPwmLayout;
