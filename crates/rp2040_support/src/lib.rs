//! Shared RP2040 bring-up helpers for Lucy board firmwares.
//!
//! Board binaries own pinmux and peripheral banks; this crate holds USB-facing
//! pieces every RP2040 Lucy firmware needs (picotool force-reset).

#![no_std]

pub mod picotool_reset;

pub use picotool_reset::PicoToolReset;
