//! Board pinout documentation for `rp2040_internal_pwm`.
//!
//! | Channel   | GPIO / resource                          |
//! |-----------|------------------------------------------|
//! | Servo1    | GPIO6  (first wired pad)                 |
//! | Servo2    | GPIO7                                    |
//! | Servo3    | GPIO8                                    |
//! | Servo4    | GPIO9                                    |
//! | Servo5    | GPIO10                                   |
//! | Servo6    | GPIO11                                   |
//! | Servo7–16 | GPIO12–21 (deterministic extension)      |
//! | ADC0–ADC3 | ADC channels 0–3                         |
//!
//! Runtime / codegen layout lives in
//! [`lucy_embedded_firmware_core::board_layout::Rp2040InternalPwmLayout`].

#![allow(unused_imports)]

pub use lucy_embedded_firmware_core::board_layout::{
    BoardLayout, HardwareIdentity, Rp2040InternalPwmLayout,
};

/// Compile-time alias for this board class layout.
pub type Layout = Rp2040InternalPwmLayout;
