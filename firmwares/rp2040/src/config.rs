use lucy_embedded_firmware_core::joint::{*};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_core::actuator::{ActuatorGroup};

use lucy_embedded_firmware_pwm_servo::pwm_servo::{PwmServoConfig, PwmServoDriver};
use lucy_embedded_firmware_feetech::feetech::{FeetechServoConfig, FeetechBusConfig, FeetechBusDriver};
use lucy_embedded_firmware_security::limits::PositionLimitGuard;
use lucy_embedded_firmware_security::thermal::{ThermalGuard, ThermalLimit};

use crate::board::Uart0Channel;
use crate::board::Resources;

use core::f32::consts::{PI, TAU};

pub const JOINTS: usize = 6;

pub type LucyRobot = Robot<JOINTS, (ThermalGuard<JOINTS>, PositionLimitGuard<JOINTS>)>;

pub const JOINTS_CONFIG: [CommandConfig; JOINTS] = [
    CommandConfig { default: PI, limit_min: 0.0, limit_max: TAU },
    CommandConfig { default: PI, limit_min: 0.0, limit_max: TAU },
    CommandConfig { default: PI, limit_min: 0.0, limit_max: TAU },
    CommandConfig { default: PI, limit_min: 0.0, limit_max: TAU },
    CommandConfig { default: PI, limit_min: 0.0, limit_max: TAU },
    CommandConfig { default: PI, limit_min: 0.0, limit_max: TAU },
];

pub const FEETECH_0: FeetechBusConfig<6> = FeetechBusConfig {
    servos: [
        FeetechServoConfig { id: 1, amplitude: TAU, min_pulse: 0, max_pulse: 4096, torque: 2.94, gear_ratio: 1.0, stall_current: 2.7, no_load_current: 0.18, current_lsb: 0.0065 },
        FeetechServoConfig { id: 2, amplitude: TAU, min_pulse: 0, max_pulse: 4096, torque: 2.94, gear_ratio: 1.0, stall_current: 2.7, no_load_current: 0.18, current_lsb: 0.0065 },
        FeetechServoConfig { id: 3, amplitude: TAU, min_pulse: 0, max_pulse: 4096, torque: 2.94, gear_ratio: 1.0, stall_current: 2.7, no_load_current: 0.18, current_lsb: 0.0065 },
        FeetechServoConfig { id: 4, amplitude: TAU, min_pulse: 0, max_pulse: 4096, torque: 2.94, gear_ratio: 1.0, stall_current: 2.7, no_load_current: 0.18, current_lsb: 0.0065 },
        FeetechServoConfig { id: 5, amplitude: TAU, min_pulse: 0, max_pulse: 4096, torque: 2.94, gear_ratio: 1.0, stall_current: 2.7, no_load_current: 0.18, current_lsb: 0.0065 },
        FeetechServoConfig { id: 6, amplitude: TAU, min_pulse: 0, max_pulse: 4096, torque: 2.94, gear_ratio: 1.0, stall_current: 2.7, no_load_current: 0.18, current_lsb: 0.0065 },
    ],
};

pub const THERMAL_LIMITS: [ThermalLimit; JOINTS] = [ThermalLimit::new(65.0, 55.0); JOINTS];

pub fn get_robot() -> LucyRobot {
    Robot::new(
        &JOINTS_CONFIG,
        (ThermalGuard::new(&THERMAL_LIMITS), PositionLimitGuard::new(&JOINTS_CONFIG)),
    )
}

use crate::board::{Servo3Channel};

pub fn robot_tick(
    tick: u64,
    robot: &mut LucyRobot,
    resources: &mut Resources) -> bool {

    let mut driver = FeetechBusDriver::<Uart0Channel, 6>::new(&FEETECH_0);
    robot.tick(|command, state| {
        driver.update(tick, &mut resources.uart0, &mut command[0..6], &mut state[0..6]).is_ok()
    })
}
