//! Board pinout for the Pimoroni Servo2040 firmware.
//!
//! One crate for all Servo2040 `board_class` values (`internal_servo_only`,
//! `internal_servo_i2c_pwm`, `bus_servo_only`). PWM silk 1..18, ADC0..3,
//! optional I2C PCA9685, and UART bus on GPIO0/1/2 when YAML enables it.
//!
//! Runtime / codegen layout:
//! [`lucy_embedded_firmware_core::board_layout::Rp2040Servo2040Layout`]
//! (UART-only `bus_servo_only` YAML still uses
//! [`lucy_embedded_firmware_core::board_layout::Rp2040BusServoLayout`] at
//! codegen time).

#![allow(unused_imports)]

pub use lucy_embedded_firmware_core::board_layout::{
    BoardLayout, HardwareIdentity, Rp2040Servo2040Layout,
};

/// Compile-time alias for this board layout.
pub type Layout = Rp2040Servo2040Layout;
