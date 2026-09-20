//! Board pinout for the Pimoroni Servo2040 firmware.
//!
//! Handles `board_class` values `internal_servo_only` and `internal_servo_i2c_pwm`
//! (PWM silk 1..18, ADC0..3, optional I2C PCA9685 / UART channels).
//!
//! Runtime / codegen layout:
//! [`lucy_embedded_firmware_core::board_layout::Rp2040Servo2040Layout`].

#![allow(unused_imports)]

pub use lucy_embedded_firmware_core::board_layout::{
    BoardLayout, HardwareIdentity, Rp2040Servo2040Layout,
};

/// Compile-time alias for this board layout.
pub type Layout = Rp2040Servo2040Layout;
