use lucy_embedded_firmware_core::joint::*;
use lucy_embedded_firmware_core::robot::Robot;

use lucy_embedded_firmware_feetech::feetech::{
    FeetechBusConfig, FeetechBusDriver, FeetechServoConfig,
};
use lucy_embedded_firmware_security::limits::PositionLimitGuard;
use lucy_embedded_firmware_security::thermal::{ThermalGuard, ThermalLimit};

use crate::resources::Resources;
use lucy_embedded_firmware_host::usb_port::UsbPort;

use core::f32::consts::{PI, TAU};
use std::io::Write;
use std::time::Instant;

pub const JOINTS: usize = 6;

pub type LucyRobot = Robot<JOINTS, (ThermalGuard<JOINTS>, PositionLimitGuard<JOINTS>)>;

pub const JOINTS_CONFIG: [CommandConfig; JOINTS] = [
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
];

pub const FEETECH_0: FeetechBusConfig<6> = FeetechBusConfig {
    servos: [
        FeetechServoConfig {
            id: 1,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 2,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 3,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 4,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 5,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 6,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
    ],
};

pub const THERMAL_LIMITS: [ThermalLimit; JOINTS] = [ThermalLimit::new(65.0, 55.0); JOINTS];

pub fn get_robot() -> LucyRobot {
    Robot::new(
        &JOINTS_CONFIG,
        (
            ThermalGuard::new(&THERMAL_LIMITS),
            PositionLimitGuard::new(&JOINTS_CONFIG),
        ),
    )
}

/// When each servo last answered a position read. The bus result alone can't say which one failed,
/// and the SHM state counter advances regardless.
#[derive(Default)]
pub struct Telemetry {
    pub last_seen: [Option<Instant>; 6],
}

pub fn robot_tick(
    tick: u64,
    robot: &mut LucyRobot,
    resources: &mut Resources,
    telemetry: &mut Telemetry,
) {
    let Some(usb) = resources.usb0.as_mut() else {
        return;
    };
    let seen = robot.tick(|command, state| {
        let mut seen = [false; 6];
        let _ = driver().update_tracked(tick, usb, &mut command[0..6], &mut state[0..6], &mut seen);
        seen
    });
    let now = Instant::now();
    for (last, seen) in telemetry.last_seen.iter_mut().zip(seen) {
        if seen {
            *last = Some(now);
        }
    }
}

pub fn robot_release(resources: &mut Resources) {
    let Some(usb) = resources.usb0.as_mut() else {
        return;
    };
    let mut driver = driver();
    let mut ok = false;
    for _ in 0..2 {
        ok |= driver.set_torque(usb, &[Command::default(); 6]).is_ok();
    }
    if !ok {
        let _ = writeln!(
            std::io::stderr(),
            "Warning: torque off failed, the servos may still hold"
        );
    }
}

pub(crate) fn driver() -> FeetechBusDriver<UsbPort, 6> {
    FeetechBusDriver::new(&FEETECH_0)
}
