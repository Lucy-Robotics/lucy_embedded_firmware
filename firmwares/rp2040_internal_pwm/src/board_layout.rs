//! Board pinout documentation for `rp2040_internal_pwm` (Pimoroni Servo2040).
//!
//! | Channel    | GPIO | Servo2040 silk |
//! |------------|------|----------------|
//! | Servo1     | 0    | 1              |
//! | Servo2     | 1    | 2              |
//! | …          | …    | …              |
//! | Servo18    | 17   | 18             |
//! | ADC0–ADC3  | ADC channels 0–3 |
//!
//! Runtime / codegen layout lives in
//! [`lucy_embedded_firmware_core::board_layout::Rp2040InternalPwmLayout`].

#![allow(unused_imports)]

pub use lucy_embedded_firmware_core::board_layout::{
    BoardLayout, HardwareIdentity, Rp2040InternalPwmLayout,
};

/// Compile-time alias for this board class layout.
pub type Layout = Rp2040InternalPwmLayout;
