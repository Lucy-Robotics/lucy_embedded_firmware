//! Local socket for calibration tooling: bus health and EEPROM homing.
//!
//! One request per line, one JSON reply per line. This firmware owns the serial port, so anything
//! that needs the bus outside the control loop has to come through here. [`protocol`] defines the
//! wire format; [`server`] carries it over a Unix socket.

mod protocol;
mod server;

// Not wired into main() until the next PR in the stack lands the handler.
#[allow(unused_imports)]
pub use protocol::{Request, RestoreEntry, json_error, json_string};
#[allow(unused_imports)]
pub use server::{Server, socket_path};
