use lucy_embedded_firmware_core::joint::{*};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_core::actuator::{ActuatorGroup};

use lucy_embedded_firmware_pwm::pwm_servo::{PwmServoConfig, PwmServoDriver};
use lucy_embedded_firmware_feetech::feetech::{FeetechServoConfig, FeetechBusConfig, FeetechBusDriver};

use crate::board::Uart0Channel;
use crate::board::Resources;

use core::f64::consts::{PI, TAU};

pub const JOINTS: usize = 6;

pub const JOINTS_CONFIG: [CommandConfig; JOINTS] = [
    CommandConfig { default: PI as f64, limit_min: 0.0, limit_max: TAU as f64 },
    CommandConfig { default: PI as f64, limit_min: 0.0, limit_max: TAU as f64 },
    CommandConfig { default: PI as f64, limit_min: 0.0, limit_max: TAU as f64 },
    CommandConfig { default: PI as f64, limit_min: 0.0, limit_max: TAU as f64 },
    CommandConfig { default: PI as f64, limit_min: 0.0, limit_max: TAU as f64 },
    CommandConfig { default: PI as f64, limit_min: 0.0, limit_max: TAU as f64 },
];

pub const FEETECH_0: FeetechBusConfig<6> = FeetechBusConfig {
    servos: [
        FeetechServoConfig { id: 1, amplitude: TAU, min_pulse: 0, max_pulse: 4096 },
        FeetechServoConfig { id: 2, amplitude: TAU, min_pulse: 0, max_pulse: 4096 },
        FeetechServoConfig { id: 3, amplitude: TAU, min_pulse: 0, max_pulse: 4096 },
        FeetechServoConfig { id: 4, amplitude: TAU, min_pulse: 0, max_pulse: 4096 },
        FeetechServoConfig { id: 5, amplitude: TAU, min_pulse: 0, max_pulse: 4096 },
        FeetechServoConfig { id: 6, amplitude: TAU, min_pulse: 0, max_pulse: 4096 },
    ],
};

pub fn get_robot() -> Robot<JOINTS> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [Command::default(); JOINTS],
        joints_state: [State::default(); JOINTS],
    }
}

use crate::board::{Servo3Channel};

pub fn robot_tick(
    tick: u64,
    robot: &mut Robot<JOINTS>,
    resources: &mut Resources) -> bool {

    let mut driver = FeetechBusDriver::<Uart0Channel, 6>::new(&FEETECH_0);
    let feetech_ok = driver.update(
        tick,
        &mut resources.uart0,
        &mut robot.joints_command[0..6],
        &mut robot.joints_state[0..6],
    ).is_ok();

    feetech_ok
}
