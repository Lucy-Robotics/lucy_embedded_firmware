use std::fs::File;
use std::io::Read;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{SharedTable, ShmHeader};
use lucy_embedded_firmware_communication::{
    encode, frame_len, Body, Frame, FrameDecoder, JointCommands, JointStates,
};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod usb_port;
use usb_port::{UsbPort, UsbPortConfig};

/// Must match `JOINTS` in `firmwares/rp2040/src/config.rs`.
const JOINTS: usize = 32;
const FRAME_LEN: usize = frame_len(JOINTS);

const REPLY_TIMEOUT: Duration = Duration::from_millis(40);
/// Wait for LucySystemHardware to create SHM (bringup may start the bridge first).
const SHM_OPEN_RETRY: Duration = Duration::from_millis(200);
const SHM_OPEN_TIMEOUT: Duration = Duration::from_secs(30);

fn die(message: impl std::fmt::Display) -> ! {
    let message = message.to_string();
    eprintln!("{message}");
    // The launcher TUI reads these when the Real Hardware row is CRASHED.
    let _ = std::fs::write("/tmp/.lucy_real_hardware_status", "1\n");
    let _ = std::fs::write("/tmp/.lucy_real_hardware_error", &message);
    std::process::exit(1);
}

struct BridgeArgs {
    node: String,
    serial: String,
    device: String,
}

fn bridge_args() -> BridgeArgs {
    let incomplete = "This robot configuration does not name the device to open. Fix it, then relaunch Lucy.";
    let mut args = std::env::args().skip(1);
    let Some(node) = args.next().filter(|value| !value.is_empty()) else {
        die(incomplete);
    };
    let Some(serial) = args.next().filter(|value| !value.is_empty()) else {
        die(incomplete);
    };
    let device = args
        .next()
        .filter(|value| !value.is_empty())
        .unwrap_or_else(|| serial.clone());
    BridgeArgs { node, serial, device }
}

fn open_shm(name: &str, min_len: usize) -> File {
    let c_name = CString::new(name).expect("shm name contains a NUL byte");
    let deadline = Instant::now() + SHM_OPEN_TIMEOUT;

    loop {
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666 as libc::c_uint) };
        if fd != -1 {
            let file = unsafe { File::from_raw_fd(fd) };
            let len = match file.metadata() {
                Ok(meta) => meta.len() as usize,
                Err(_) => die("Lucy could not attach to the robot. Relaunch Lucy."),
            };
            if len < min_len {
                die("This robot configuration does not match Lucy. Relaunch Lucy.");
            }
            return file;
        }
        if Instant::now() >= deadline {
            die("Lucy did not finish starting. Relaunch Lucy.");
        }
        std::thread::sleep(SHM_OPEN_RETRY);
    }
}

fn read_commands(shm: &SharedTable, n: usize) -> JointCommands<JOINTS> {
    loop {
        let before = shm.header().command_seq.load(Ordering::Acquire);
        if before % 2 != 0 {
            std::hint::spin_loop();
            continue;
        }

        let mut commands = JointCommands::<JOINTS>::default();
        commands.positions[..n].copy_from_slice(&shm.hw_commands()[..n]);
        commands.velocities[..n].copy_from_slice(&shm.hw_velocities()[..n]);
        commands.accelerations[..n].copy_from_slice(&shm.hw_accelerations()[..n]);
        for (enabled, &t) in commands.torque_enabled.iter_mut().zip(&shm.hw_torque_enabled()[..n]) {
            *enabled = t >= 0.5;
        }

        if shm.header().command_seq.load(Ordering::Acquire) == before {
            return commands;
        }
    }
}

fn write_states(shm: &mut SharedTable, n: usize, states: &JointStates<JOINTS>) {
    shm.header().state_seq.fetch_add(1, Ordering::SeqCst);
    shm.hw_positions_mut()[..n].copy_from_slice(&states.positions[..n]);
    shm.header().state_seq.fetch_add(1, Ordering::SeqCst);
}

fn wait_reply(
    port: &mut UsbPort,
    decoder: &mut FrameDecoder<'_, JOINTS>,
    seq: u32,
) -> Option<JointStates<JOINTS>> {
    let deadline = Instant::now() + REPLY_TIMEOUT;
    let mut buf = [0u8; 256];
    while Instant::now() < deadline {
        let count = match port.port.read(&mut buf) {
            Ok(count) => count,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => {
                eprintln!("Serial read failed: {e}");
                return None;
            }
        };
        for &byte in &buf[..count] {
            match decoder.push_byte(byte) {
                Some(Ok(Frame { seq: s, body: Body::State(states), .. })) if s == seq => {
                    return Some(states);
                }
                Some(Ok(frame)) => eprintln!("Ignoring unexpected frame seq={} (waiting for {seq})", frame.seq),
                Some(Err(e)) => eprintln!("Dropping corrupt frame from rp2040: {e}"),
                None => {}
            }
        }
    }
    None
}

fn main() {
    let bridge = bridge_args();
    let node = bridge.node;
    println!("Attaching to Lucy shared memory for node_name={node}");

    let shm_file = open_shm(&format!("/{node}"), size_of::<ShmHeader>());

    let table_map = match unsafe { MmapMut::map_mut(&shm_file) } {
        Ok(map) => map,
        Err(_) => die("Lucy could not attach to the robot. Relaunch Lucy."),
    };
    let mut ass = unsafe { SharedTable::from_raw(table_map.as_ptr() as *mut u8, table_map.len()) }
        .unwrap_or_else(|_| die("Lucy could not attach to the robot. Relaunch Lucy."));
    let n = JOINTS.min(ass.max_actuators());
    if ass.max_actuators() != JOINTS {
        eprintln!(
            "Warning: shm declares {} joints but the rp2040 link carries {JOINTS}; only the first {n} are forwarded",
            ass.max_actuators()
        );
    }

    println!("Opening Pico serial {} for node_name={node}", bridge.serial);
    let usb_config = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000, bridge.serial);
    let mut port = match usb_config.open() {
        Ok(port) => port,
        Err(err) => die(usb_port::user_fix(err.kind(), &bridge.device)),
    };
    port.clear().ok();

    let mut rx_buf = [0u8; FRAME_LEN];
    let mut decoder = FrameDecoder::<JOINTS>::new(&mut rx_buf);
    let mut frame = [0u8; FRAME_LEN];
    let mut seq: u32 = 0;

    let period = Duration::from_millis(50);
    let mut next_tick = Instant::now();

    loop {
        next_tick += period;
        seq = seq.wrapping_add(1);

        let request = Frame::new(seq, Body::Command(read_commands(&ass, n)));
        let len = encode(&request, &mut frame).expect("command frame exceeds FRAME_LEN");

        match port.write(&frame[..len]) {
            Ok(()) => match wait_reply(&mut port, &mut decoder, seq) {
                Some(states) => write_states(&mut ass, n, &states),
                None => eprintln!("No state reply from rp2040 for seq={seq}, dropping tick"),
            },
            Err(e) => {
                eprintln!("Failed to write commands over serial port: {e}, dropping tick");
            }
        }

        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            eprintln!("Warning: tick took longer than expected {:?}", now - next_tick);
            next_tick = now;
        }
    }
}
