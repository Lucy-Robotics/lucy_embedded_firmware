use lucy_embedded_firmware_core::actuator::ActuatorGroup;
use lucy_embedded_firmware_core::joint::*;
use lucy_embedded_firmware_core::robot::Robot;

use lucy_embedded_firmware_feetech::feetech::{FeetechServoConfig, FeetechBusConfig, FeetechBusDriver};

use crate::usb_port::UsbPort;
use crate::resources::Resources;

use core::f64::consts::{PI, TAU};
use std::io::Write;

pub const JOINTS_CONFIG: [CommandConfig; 6] = [
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

pub fn get_robot() -> Robot<6> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [Command::default(); 6],
        joints_state: [State::default(); 6],
    }
}

pub fn robot_tick(
    tick: u64,
    robot: &mut Robot<6>,
    resources: &mut Resources,
) {
    let mut driver: FeetechBusDriver<UsbPort, 6> = FeetechBusDriver::<UsbPort, 6>::new(&FEETECH_0);
    let _ = driver.update(
        tick,
        &mut resources.usb0,
        &mut robot.joints_command[0..6],
        &mut robot.joints_state[0..6],
    );
}

/// Torque off on every servo so the arm goes limp instead of holding when the host stops.
pub fn robot_release(resources: &mut Resources) {
    let mut driver: FeetechBusDriver<UsbPort, 6> = FeetechBusDriver::<UsbPort, 6>::new(&FEETECH_0);
    let _ = driver.set_torque(&mut resources.usb0, &[Command::default(); 6]);
    // Make sure the frame is on the wire before the process exits and the port closes.
    let _ = resources.usb0.port.flush();
}
