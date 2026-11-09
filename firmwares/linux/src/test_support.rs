//! Test fixtures shared by the maintenance and status-report test suites.

use lucy_embedded_firmware_core::joint::TorqueStatus;
use lucy_embedded_firmware_core::robot::Robot;

use crate::config::get_robot;
use crate::maintenance::RestoreEntry;
use crate::resources::Resources;
use crate::usb_port::{Adapter, lucy_usb_config};

pub(crate) fn offline() -> Resources {
    Resources::new(lucy_usb_config())
}

pub(crate) fn open_on(serial: Option<&str>) -> Resources {
    let mut resources = offline();
    resources.open_adapter = Some(Adapter {
        port_name: "/dev/ttyACM0".to_string(),
        serial: serial.map(str::to_string),
    });
    resources
}

pub(crate) fn torque_enabled_robot() -> Robot<6> {
    let mut robot = get_robot();
    robot.joints_command[2].torque_enabled = TorqueStatus::Enabled;
    robot
}

pub(crate) fn entries() -> Vec<RestoreEntry> {
    (1..=6)
        .map(|id| RestoreEntry {
            id,
            homing_offset: -5,
            min_limit: 0,
            max_limit: 4095,
        })
        .collect()
}

pub(crate) fn temp_socket(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("lucy-test-{}-{name}", std::process::id()));
    dir.join("node.sock")
}
