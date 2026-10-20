use lucy_embedded_firmware_core::joint::{*};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_core::actuator::{ActuatorGroup};

use lucy_embedded_firmware_pwm::pwm_servo::{PwmServoConfig, PwmServoDriver};

use crate::board::{Resources, Servo6Channel};

use core::f64::consts::PI;

const DEG: f64 = PI / 180.0;

// shortcut: slots follow the torso_head HI joint order (jaw = 7); others are unwired.
// Upgrade when the builder generates this from the robot config.
const UNUSED: CommandConfig = CommandConfig { default: 0.0, limit_min: 0.0, limit_max: 0.0 };

// Jaw: rest at 25 deg, mechanical range 25..120 deg.
pub const JOINTS_CONFIG: [CommandConfig; 8] = [
    UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED,
    CommandConfig { default: 25.0 * DEG, limit_min: 25.0 * DEG, limit_max: 120.0 * DEG },
];

// Arduino Servo library defaults: 0..180 deg = 544..2400 us.
pub const PWM_0: PwmServoConfig = PwmServoConfig {
    amplitude: PI,
    min_pulse: 544,
    max_pulse: 2400,
};

pub fn get_robot() -> Robot<8> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [Command::default(); 8],
        joints_state: [State::default(); 8],
    }
}

pub fn robot_tick(
    tick: u64,
    robot: &mut Robot<8>,
    resources: &mut Resources) {

    let mut driver = PwmServoDriver::<Servo6Channel>::new(&PWM_0);
    driver.update(
        tick,
        &mut resources.servo6,
        &mut robot.joints_command[7..8],
        &mut robot.joints_state[7..8],
    ).ok();
}
