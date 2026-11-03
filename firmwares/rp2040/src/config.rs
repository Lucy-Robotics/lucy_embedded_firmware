use lucy_embedded_firmware_core::joint::CommandConfig;
use lucy_embedded_firmware_core::robot::Robot;

use crate::board::Resources;

/// Must match HI ``MAX_ACTUATORS`` / mcu_bridge frame size.
pub const JOINTS: usize = 32;

const JOINTS_CONFIG: [CommandConfig; JOINTS] = [CommandConfig {
    default: 0.0,
    limit_min: 0.0,
    limit_max: 0.0,
}; JOINTS];

pub fn get_robot() -> Robot<JOINTS> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [lucy_embedded_firmware_core::joint::Command::default(); JOINTS],
        joints_state: [lucy_embedded_firmware_core::joint::State::default(); JOINTS],
    }
}

/// Drive header PWM + PCA banks from the projected tables. Always succeeds (open-loop).
pub fn robot_tick(_tick: u64, robot: &mut Robot<JOINTS>, resources: &mut Resources) -> bool {
    crate::headers::write(&mut resources.slices, &robot.joints_command);
    if let Some(bank) = resources.i2c.as_mut() {
        bank.write_rows(&robot.joints_command);
    }
    // Open-loop echo so USB state frames match the last commanded positions.
    for i in 0..robot.joints_command.len() {
        robot.joints_state[i].position = robot.joints_command[i].position;
    }
    true
}
