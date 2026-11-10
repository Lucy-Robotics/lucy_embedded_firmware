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

/// No position guard: the projected tables clamp each pulse to its servo travel.
pub type LucyRobot = Robot<JOINTS, ()>;

pub fn get_robot() -> LucyRobot {
    Robot::new(&JOINTS_CONFIG, ())
}

/// Drive header PWM + PCA banks from the projected tables. Always succeeds (open-loop).
pub fn robot_tick(_tick: u64, robot: &mut LucyRobot, resources: &mut Resources) -> bool {
    robot.tick(|command, state| {
        crate::headers::write(&mut resources.slices, command);
        if let Some(bank) = resources.i2c.as_mut() {
            bank.write_rows(command);
        }
        // Open-loop echo so USB state frames match the last commanded positions.
        for (state, command) in state.iter_mut().zip(command.iter()) {
            state.position = command.position;
        }
        true
    })
}
