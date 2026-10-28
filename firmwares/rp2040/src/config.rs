use lucy_embedded_firmware_core::joint::CommandConfig;
use lucy_embedded_firmware_core::robot::Robot;

use crate::board::Resources;

const JOINTS_CONFIG: [CommandConfig; 32] = [CommandConfig {
    default: 0.0,
    limit_min: 0.0,
    limit_max: 0.0,
}; 32];

pub fn get_robot() -> Robot<32> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [lucy_embedded_firmware_core::joint::Command::default(); 32],
        joints_state: [lucy_embedded_firmware_core::joint::State::default(); 32],
    }
}

pub fn robot_tick(_tick: u64, robot: &mut Robot<32>, resources: &mut Resources) {
    crate::headers::write(&mut resources.slices, &robot.joints_command);
    if let Some(bank) = resources.i2c.as_mut() {
        bank.write_rows(&robot.joints_command);
    }
    // Open-loop echo so USB state frames match the last commanded positions.
    for i in 0..robot.joints_command.len() {
        robot.joints_state[i].position = robot.joints_command[i].position;
    }
}
