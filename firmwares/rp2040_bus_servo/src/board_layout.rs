//! Pin layout: [`lucy_embedded_firmware_core::board_layout::Rp2040BusServoLayout`].
//!
//! Hardware (SO-ARM PoC): UART0 TX=GPIO0, RX=GPIO1, half-duplex DIR=GPIO2 @ 1 Mbaud.
//! YAML channels are `UART0:<feetech_id>` (e.g. `UART0:1`).

pub use lucy_embedded_firmware_core::board_layout::Rp2040BusServoLayout;

/// Compile-time alias for this board class layout.
#[allow(dead_code)]
pub type Layout = Rp2040BusServoLayout;
