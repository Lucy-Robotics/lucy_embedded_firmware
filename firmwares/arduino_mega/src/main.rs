#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use lucy_embedded_firmware_core::joint::TorqueStatus;
use lucy_embedded_firmware_communication::{
    encode, frame_len, Body, Frame, FrameDecoder, JointCommands, JointStates,
};

use arduino_hal::prelude::*;
use panic_halt as _;

mod channel;
// macro_use: board.rs uses the generated mega_servo_pins! macro.
#[macro_use]
mod config;
mod board;
use board::init;
mod servo_pins;
use config::{get_robot, robot_tick, JOINTS, LINK, PCA_STATUS_SLOT};

const FRAME_LEN: usize = frame_len(LINK);
const TX_CHUNK_GAP_US: u32 = 500;

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

    // Request/response with mcu_bridge: the USART has a 2-byte FIFO, so only poll bytes
    // while a command frame streams in, and do the slow work once it is complete.
    // The servos are only driven after the first valid command frame; a corrupt frame is
    // dropped at its 0x00 delimiter and the bridge times out and resends.
    loop {
        let Ok(byte) = resources.serial.read() else {
            continue;
        };
        let Some(Ok(Frame { seq, body: Body::Command(commands), .. })) = decoder.push_byte(byte) else {
            continue;
        };

        apply_commands(&mut robot, &commands);
        robot_tick(tick, &mut robot, &mut resources);
        tick = tick.wrapping_add(1);

        // PWM servos report no position: NaN tells the hardware interface to keep echoing its
        // command instead of publishing a made-up one.
        let mut states = JointStates::<LINK>::default();
        states.positions[..JOINTS].fill(f64::NAN);
        states.positions[PCA_STATUS_SLOT] = (0x100 | resources.pca.check() as u16) as f64;

        let Ok(len) = encode(&Frame::new(seq, Body::State(states)), &mut tx) else {
            continue;
        };
        for (i, &b) in tx[..len].iter().enumerate() {
            resources.serial.write_byte(b);
            // The 16U2 USB-serial chip relays through a 128-byte ring: sent in one 1 Mbaud
            // burst, the frame's last bytes (its CRC) overwrote bytes still queued 128 earlier.
            // A pause per 64 bytes lets it drain; 500 us was the shortest gap clean on the bench.
            if i % 64 == 63 {
                arduino_hal::delay_us(TX_CHUNK_GAP_US);
            }
        }
    }
}
