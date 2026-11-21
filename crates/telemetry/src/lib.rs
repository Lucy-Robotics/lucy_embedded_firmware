//! Board telemetry: how the firmware itself runs, never what the sensors read.
//!
//! Integer arithmetic on `u32` microsecond timestamps only, so it costs the same on a board
//! without an FPU.
#![no_std]

pub mod loop_timer;
