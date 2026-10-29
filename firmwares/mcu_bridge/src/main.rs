use std::fs::File;
use std::io::Read;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{SharedTable, ShmHeader};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use lucy_embedded_firmware_communication::{
    encode, frame_len, Body, Frame, FrameDecoder, JointCommands, JointStates,
};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod usb_port;
use usb_port::{UsbPort, UsbPortConfig};

/// Must match `JOINTS` in `firmwares/rp2040/src/config.rs`.
const JOINTS: usize = 6;
const FRAME_LEN: usize = frame_len(JOINTS);

const REPLY_TIMEOUT: Duration = Duration::from_millis(40);

fn node_name() -> String {
    std::env::args()
        .nth(1)
        .or_else(|| std::env::var("LUCY_NODE_NAME").ok())
        .filter(|n| !n.is_empty())
        .unwrap_or_else(|| "lucy".to_string())
}

fn open_shm(name: &str, min_len: usize) -> File {
    let c_name = CString::new(name).expect("shm name contains a NUL byte");
    let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666 as libc::c_uint) };
    if fd == -1 {
        panic!(
            "shm_open(\"{name}\") failed: {}. Is the ROS 2 stack running with node_name={}?",
            std::io::Error::last_os_error(),
            name.trim_start_matches('/').split('.').next().unwrap_or(name),
        );
    }
    let file = unsafe { File::from_raw_fd(fd) };
    let len = file
        .metadata()
        .unwrap_or_else(|e| panic!("stat on shm object \"{name}\" failed: {e}"))
        .len() as usize;
    if len < min_len {
        panic!("shm object \"{name}\" is {len} bytes, expected at least {min_len}");
    }
    file
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
    let node = node_name();
    println!("Attaching to Lucy shared memory for node_name={node}");

    let shm_file = open_shm(
        &format!("/{node}"),
        size_of::<ShmHeader>(),
    );

    let table_map = unsafe { MmapMut::map_mut(&shm_file).unwrap() };
    let mut ass = unsafe { SharedTable::from_raw(table_map.as_ptr() as *mut u8, table_map.len()) }
        .unwrap_or_else(|e| panic!("invalid shm layout: {e}"));
    let n = JOINTS.min(ass.max_actuators());
    if ass.max_actuators() != JOINTS {
        eprintln!(
            "Warning: shm declares {} joints but the rp2040 link carries {JOINTS}; only the first {n} are forwarded",
            ass.max_actuators()
        );
    }

    let usb_config = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000);
    let mut port = usb_config.open().expect("Failed to initialize USB Port");
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
