#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use portable_atomic::AtomicU64;

use lucy_embedded_firmware_core::data::{CommandBlock, StateBlock, MAX_ACTUATORS};
use lucy_embedded_firmware_communication::{encode_frame, framed_len, FrameDecoder};

use arduino_hal::prelude::*;
use panic_halt as _;

mod channel;
// macro_use: board.rs uses the generated mega_servo_pins! macro.
#[macro_use]
mod config;
mod board;
use board::init;
mod servo_pins;
use config::get_robot;
use config::robot_tick;

const STATE_BYTES: usize = size_of::<StateBlock<MAX_ACTUATORS>>();
const COMMAND_BYTES: usize = size_of::<CommandBlock<MAX_ACTUATORS>>();

// AVR aligns u64/f64 to 1; frames must still match the host's 64-aligned layout byte for byte.
const _: () = assert!(COMMAND_BYTES == 576 && STATE_BYTES == 192);

fn struct_as_bytes<T>(value: &T) -> &[u8] {
    unsafe {
        core::slice::from_raw_parts(
            value as *const T as *const u8,
            size_of::<T>(),
        )
    }
}

fn struct_as_bytes_mut<T>(value: &mut T) -> &mut [u8] {
    unsafe {
        core::slice::from_raw_parts_mut(
            value as *mut T as *mut u8,
            size_of::<T>(),
        )
    }
}

const FRAME_LEN: usize = framed_len(STATE_BYTES);
const TX_CHUNK_GAP_US: u32 = 500;

// Statics, not locals: AVR does not realign its stack, and LLVM computes field addresses
// of these align(64) blocks from that assumed alignment, which garbles the frames.
static mut COMMANDS: CommandBlock<MAX_ACTUATORS> = CommandBlock {
    command_seq: AtomicU64::new(0),
    hw_commands: [0.0; MAX_ACTUATORS],
    hw_velocities: [0.0; MAX_ACTUATORS],
    hw_accelerations: [0.0; MAX_ACTUATORS],
    hw_torque_enabled: [0.0; MAX_ACTUATORS],
};

static mut STATES: StateBlock<MAX_ACTUATORS> = StateBlock {
    state_seq: AtomicU64::new(0),
    hw_positions: [0.0; MAX_ACTUATORS],
};

#[arduino_hal::entry]
fn main() -> ! {
    let mut resources = init();
    let mut robot = get_robot();

    // SAFETY: only main touches these, and it never returns.
    let commands = unsafe { &mut *(&raw mut COMMANDS) };
    let states = unsafe { &mut *(&raw mut STATES) };

    let mut decoder = FrameDecoder::<COMMAND_BYTES>::new();
    let mut frame = [0u8; FRAME_LEN];

    // Request/response with mcu_bridge: the USART has a 2-byte FIFO, so only poll bytes
    // while a command frame streams in, and do the slow work once it is complete.
    // The servo is only driven after the first valid command frame.
    // shortcut: idle measured in empty polls (~2 us each), not a timer; ~10 ms here.
    // Upgrade to a hardware timer if the loop body grows.
    const IDLE_RESYNC_POLLS: u16 = 5_000;
    let mut idle_polls: u16 = 0;
    loop {
        let Ok(byte) = resources.serial.read() else {
            // A gap means the host finished (or abandoned) a frame: drop any partial
            // sync so a stray 0xAA mid-frame can't lock the decoder out for good.
            idle_polls = idle_polls.saturating_add(1);
            if idle_polls == IDLE_RESYNC_POLLS {
                decoder = FrameDecoder::new();
            }
            continue;
        };
        idle_polls = 0;
        let Some(payload) = decoder.push_byte(byte) else {
            continue;
        };
        struct_as_bytes_mut(commands).copy_from_slice(payload);

        robot.sync_write(commands);
        robot_tick(
            states.state_seq.load(portable_atomic::Ordering::Relaxed),
            &mut robot,
            &mut resources,
        );
        robot.sync_read(states);

        encode_frame(struct_as_bytes(states), &mut frame);
        for (i, &b) in frame.iter().enumerate() {
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
