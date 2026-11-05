#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use lucy_embedded_firmware_core::joint::TorqueStatus;
use lucy_embedded_firmware_communication::{
    encode, frame_len, Body, Error, Frame, FrameDecoder, JointCommands, JointStates,
};

use arduino_hal::prelude::*;
use panic_halt as _;

mod channel;
// board.rs uses mega_servo_pins!
#[macro_use]
mod config;
mod board;
use board::{init, Resources};
mod servo_pins;
use config::{get_robot, robot_tick, JOINTS, LINK, PCA_STATUS_SLOT};

const FRAME_LEN: usize = frame_len(LINK);
const TX_CHUNK_GAP_US: u32 = 500;
const COMMAND_TIMEOUT_TICKS: u16 = (200_000 / 64) as u16;

fn apply_commands(robot: &mut lucy_embedded_firmware_core::robot::Robot<JOINTS>, commands: &JointCommands<LINK>) {
    for i in 0..JOINTS {
        let position = robot.clamp_position(i, commands.positions[i]);
        let joint = &mut robot.joints_command[i];
        joint.position = position;
        joint.velocity = commands.velocities[i];
        joint.acceleration = commands.accelerations[i];
        joint.torque_enabled = if commands.torque_enabled[i] {
            TorqueStatus::Enabled
        } else {
            TorqueStatus::Disabled
        };
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let mut resources = init();
    let mut robot = get_robot();

    let mut rx = [0u8; FRAME_LEN];
    let mut decoder = FrameDecoder::<LINK>::new(&mut rx);
    let mut tx = [0u8; FRAME_LEN];
    let mut tick: u64 = 0;
    let mut last_command: Option<u16> = None;

    // 2-byte USART FIFO: only poll while a frame streams in
    loop {
        let now = resources.clock.tcnt1().read().bits();
        let Ok(byte) = resources.serial.read() else {
            if last_command.is_some_and(|t| now.wrapping_sub(t) > COMMAND_TIMEOUT_TICKS) {
                for joint in robot.joints_command.iter_mut() {
                    joint.torque_enabled = TorqueStatus::Disabled;
                }
                robot_tick(tick, &mut robot, &mut resources);
                last_command = None;
            }
            continue;
        };
        match decoder.push_byte(byte) {
            Some(Ok(Frame { seq, body: Body::Command(commands), .. })) => {
                apply_commands(&mut robot, &commands);
                robot_tick(tick, &mut robot, &mut resources);
                tick = tick.wrapping_add(1);
                last_command = Some(now);
                reply(seq, &mut resources, &mut tx);
            }
            Some(Err(Error::JointCount { .. })) => reply(0, &mut resources, &mut tx),
            _ => {}
        }
    }
}

fn reply(seq: u32, resources: &mut Resources, tx: &mut [u8; FRAME_LEN]) {
    // NaN = no position feedback
    let mut states = JointStates::<LINK>::default();
    states.positions[..JOINTS].fill(f32::NAN);
    states.positions[PCA_STATUS_SLOT] = (0x100 | resources.pca.check() as u16) as f32;

    let Ok(len) = encode(&Frame::new(seq, Body::State(states)), tx) else {
        return;
    };
    for (i, &b) in tx[..len].iter().enumerate() {
        resources.serial.write_byte(b);
        // lets the 16U2's 128-byte ring drain
        if i % 64 == 63 {
            arduino_hal::delay_us(TX_CHUNK_GAP_US);
        }
    }
}
