use lucy_embedded_firmware_core::joint::{*};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_core::actuator::{ActuatorGroup};

use lucy_embedded_firmware_pwm::pwm_servo::{PwmServoConfig, PwmServoDriver};
use lucy_embedded_firmware_feetech::feetech::{FeetechServoConfig, FeetechBusConfig, FeetechBusDriver};

use crate::board::Uart0Channel;
use crate::board::Resources;

use core::f64::consts::{PI, TAU};

pub const JOINTS_CONFIG: [CommandConfig; 7] = [
    CommandConfig { default: PI as f64, limit_min: 0.0, limit_max: TAU as f64 },
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

pub const PWM_0: PwmServoConfig = PwmServoConfig {
    amplitude: PI,
    min_pulse: 500,
    max_pulse: 2500,
};

pub fn get_robot() -> Robot<7> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [Command::default(); 7],
        joints_state: [State::default(); 7],
    }
}

use crate::board::{Servo3Channel};

pub fn robot_tick(
    tick: u64,
    robot: &mut Robot<7>,
    resources: &mut Resources) {

    let mut driver = FeetechBusDriver::<Uart0Channel, 6>::new(&FEETECH_0);
    driver.update(
        tick,
        &mut resources.uart0,
        &mut robot.joints_command[0..6],
        &mut robot.joints_state[0..6],
    ).ok();

    let mut driver = PwmServoDriver::<Servo3Channel>::new(&PWM_0);
    driver.update(
        tick,
        &mut resources.servo3,
        &mut robot.joints_command[6..7],
        &mut robot.joints_state[6..7],
    ).ok();

}
