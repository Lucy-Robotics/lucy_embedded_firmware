use lucy_embedded_firmware_core::joint::{*};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_core::actuator::{ActuatorGroup};
use lucy_embedded_firmware_core::data::MAX_ACTUATORS;

use lucy_embedded_firmware_pwm::pwm_servo::{PwmServoConfig, PwmServoDriver};

use crate::board::Resources;

use core::f64::consts::PI;

// One firmware per board, picked by the cargo feature named in the board's
// `compile_definition` (robot hardware YAML).
// shortcut: board tables are copied by hand from the YAML (slot = virtual_pin,
// output = physical_pin - 1, limits in servo radians). Upgrade when the builder generates them.
#[cfg(feature = "torso_head")]
mod torso_head;
#[cfg(feature = "torso_head")]
pub use torso_head::{JOINTS_CONFIG, OUTPUTS, PCA_ADDRESSES};

#[cfg(feature = "arms")]
mod arms;
#[cfg(feature = "arms")]
pub use arms::{JOINTS_CONFIG, OUTPUTS, PCA_ADDRESSES};

#[cfg(not(any(feature = "torso_head", feature = "arms")))]
compile_error!("pick the board: --features torso_head or --features arms");
#[cfg(all(feature = "torso_head", feature = "arms"))]
compile_error!("one board per firmware: enable only one of torso_head / arms");

pub const JOINTS: usize = OUTPUTS.len();

/// Full 0..180 deg travel, resting at `default`.
const fn full_range(default: f64) -> CommandConfig {
    CommandConfig { default, limit_min: 0.0, limit_max: PI }
}

// Arduino Servo library defaults: 0..180 deg = 544..2400 us.
pub const PWM_0: PwmServoConfig = PwmServoConfig {
    amplitude: PI,
    min_pulse: 544,
    max_pulse: 2400,
};

// Board table checks: limits fit the travel PWM_0 maps, each output exists and drives one joint.
const _: () = {
    assert!(JOINTS_CONFIG.len() == JOINTS && JOINTS <= MAX_ACTUATORS);
    let mut i = 0;
    while i < JOINTS {
        let c = JOINTS_CONFIG[i];
        assert!(0.0 <= c.limit_min && c.limit_min <= c.limit_max && c.limit_max <= PWM_0.amplitude);
        assert!((OUTPUTS[i] as usize) < 16 * PCA_ADDRESSES.len());
        let mut j = i + 1;
        while j < JOINTS {
            assert!(OUTPUTS[i] != OUTPUTS[j]);
            j += 1;
        }
        i += 1;
    }
};

pub fn get_robot() -> Robot<JOINTS> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [Command::default(); JOINTS],
        joints_state: [State::default(); JOINTS],
    }
}

pub fn robot_tick(
    tick: u64,
    robot: &mut Robot<JOINTS>,
    resources: &mut Resources) {

    for (i, &output) in OUTPUTS.iter().enumerate() {
        let mut channel = resources.pca.output(output);
        PwmServoDriver::new(&PWM_0).update(
            tick,
            &mut channel,
            &mut robot.joints_command[i..i + 1],
            &mut robot.joints_state[i..i + 1],
        ).ok();
    }
}
