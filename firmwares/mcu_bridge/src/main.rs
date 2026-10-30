use std::fs::File;
use std::io::Read;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{SharedTable, ShmHeader};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use lucy_embedded_firmware_communication::{
    encode, frame_len, Body, Error as FrameError, Frame, FrameDecoder, JointCommands, JointStates,
};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod usb_port;
use usb_port::{UsbPort, UsbPortConfig};

/// Must match `JOINTS` in `firmwares/rp2040/src/config.rs`.
const RP2040_JOINTS: usize = 6;

/// A Mega replies within ~10 ms; past this the tick is dropped and the next one resends.
const REPLY_TIMEOUT: Duration = Duration::from_millis(60);

#[derive(Clone, Copy, PartialEq)]
enum Board {
    Rp2040,
    /// Links its joints plus one PCA9685 status slot (see `pca_changes`).
    Mega,
}

enum ReplyError {
    Timeout,
    Io(std::io::Error),
    Corrupt(FrameError),
}

impl std::fmt::Display for ReplyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ReplyError::Timeout => write!(f, "no reply within {REPLY_TIMEOUT:?}"),
            ReplyError::Io(e) => write!(f, "io error ({:?}): {e}", e.kind()),
            // Bytes *did* arrive up to a 0x00 delimiter, but they don't decode to a frame:
            // framing/corruption on the wire, not a dead link.
            ReplyError::Corrupt(e) => write!(f, "corrupt frame ({e})"),
        }
    }
}

/// Seqlock read of the first `n` command slots the hardware interface wrote.
fn read_commands<const N: usize>(shm: &SharedTable, n: usize) -> JointCommands<N> {
    loop {
        let before = shm.header().command_seq.load(Ordering::Acquire);
        if before % 2 != 0 {
            std::hint::spin_loop();
            continue;
        }

        let mut commands = JointCommands::<N>::default();
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

fn write_states<const N: usize>(shm: &mut SharedTable, n: usize, states: &JointStates<N>) {
    shm.header().state_seq.fetch_add(1, Ordering::SeqCst);
    shm.hw_positions_mut()[..n].copy_from_slice(&states.positions[..n]);
    shm.header().state_seq.fetch_add(1, Ordering::SeqCst);
}

/// Reads until the state frame answering `seq`; replies to older ticks are skipped.
fn wait_reply<const N: usize>(
    port: &mut UsbPort,
    decoder: &mut FrameDecoder<'_, N>,
    seq: u32,
) -> Result<JointStates<N>, ReplyError> {
    let deadline = Instant::now() + REPLY_TIMEOUT;
    let mut buf = [0u8; 256];
    while Instant::now() < deadline {
        let count = match port.port.read(&mut buf) {
            Ok(count) => count,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(ReplyError::Io(e)),
        };
        for &byte in &buf[..count] {
            match decoder.push_byte(byte) {
                Some(Ok(Frame { seq: s, body: Body::State(states), .. })) if s == seq => return Ok(states),
                Some(Ok(_)) | None => {}
                Some(Err(e)) => return Err(ReplyError::Corrupt(e)),
            }
        }
    }
    Err(ReplyError::Timeout)
}

fn node_name() -> String {
    std::env::args()
        .nth(1)
        .or_else(|| std::env::var("LUCY_NODE_NAME").ok())
        .filter(|n| !n.is_empty())
        .unwrap_or_else(|| "lucy".to_string())
}

/// USB serial number of the board to drive (its `serial_id`); unset = first board found.
fn usb_serial() -> Option<String> {
    std::env::args()
        .nth(2)
        .or_else(|| std::env::var("LUCY_USB_SERIAL").ok())
        .filter(|s| !s.is_empty())
}

/// PCA9685 addresses of a Mega board (`0x41,0x40`), in its `pca_addresses` order.
fn parse_pca_addresses(arg: &str) -> Vec<u8> {
    arg.split(',')
        .filter_map(|a| {
            let a = a.trim();
            a.strip_prefix("0x").map_or_else(|| a.parse().ok(), |h| u8::from_str_radix(h, 16).ok())
        })
        .collect()
}

/// The Mega firmware reports its PCA9685s in the last link slot as `0x100 | missing bits`
/// (bit i = `addresses[i]`). Returns a line for every chip the first time, then per change.
fn pca_changes(addresses: &[u8], slot: f64, last: &mut Option<u8>) -> Vec<String> {
    let value = slot as u16;
    if addresses.is_empty() || value & 0x100 == 0 {
        return Vec::new();
    }
    let missing = value as u8;
    let lines = addresses
        .iter()
        .enumerate()
        .filter(|&(i, _)| last.is_none_or(|m| (m ^ missing) & (1 << i) != 0))
        .map(|(i, address)| {
            if missing & (1 << i) != 0 {
                format!("PCA9685 {address:#04x} not answering: its servos can't move")
            } else {
                format!("PCA9685 {address:#04x} answering")
            }
        })
        .collect();
    *last = Some(missing);
    lines
}

/// No states frame for this long means the board is on USB but not answering (frozen, or
/// still in its bootloader); it covers a normal reboot after connecting.
const SILENT_AFTER: Duration = Duration::from_secs(3);

const RETRY: Duration = Duration::from_secs(1);

/// Flashers (config pipeline, `pixi run firmware-flash-mega`) hold this file while avrdude
/// owns the port: the bridge must not touch the board then.
fn flash_lock(serial: Option<&str>) -> Option<std::path::PathBuf> {
    serial.map(|s| std::path::PathBuf::from(format!("/tmp/.lucy_mcu_flashing_{s}")))
}

fn try_open_shm(name: &str, min_len: usize) -> Result<File, String> {
    let c_name = CString::new(name).expect("shm name contains a NUL byte");
    let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666 as libc::c_uint) };
    if fd == -1 {
        return Err(format!("shm_open: {}", std::io::Error::last_os_error()));
    }
    let file = unsafe { File::from_raw_fd(fd) };
    let len = file.metadata().map_err(|e| format!("stat: {e}"))?.len() as usize;
    if len < min_len {
        return Err(format!("{len} bytes, expected at least {min_len}"));
    }
    Ok(file)
}

/// The hardware interface creates the table once the control stack is up: wait for it.
fn attach_shm(name: &str, min_len: usize) -> File {
    let mut waiting = false;
    loop {
        match try_open_shm(name, min_len) {
            Ok(file) => return file,
            Err(e) if !waiting => {
                println!("Waiting for the control stack's shared memory {name} ({e})");
                waiting = true;
            }
            Err(_) => {}
        }
        std::thread::sleep(RETRY);
    }
}

/// RP2040 first, else Arduino Mega 2560.
fn open_board(serial: Option<&str>) -> std::io::Result<(UsbPort, Board)> {
    UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000)
        .open(serial)
        .map(|port| (port, Board::Rp2040))
        .or_else(|_| {
            let mut port = UsbPortConfig::new(0x2341, 0x0042, 1_000_000).open(serial)?;
            // Opening resets the Mega; bytes sent while its bootloader runs keep it from
            // ever starting the firmware, so wait it out before the first frame.
            std::thread::sleep(Duration::from_secs(2));
            port.clear()?;
            Ok((port, Board::Mega))
        })
}

/// Open the board, waiting (one message per state) while it is unplugged or being flashed.
fn connect(serial: Option<&str>, lock: Option<&std::path::Path>) -> (UsbPort, Board) {
    let label = serial.unwrap_or("(first found)");
    let mut said: Option<String> = None;
    loop {
        let state = if lock.is_some_and(|l| l.exists()) {
            "is being flashed".to_string()
        } else {
            match open_board(serial) {
                Ok(board) => {
                    println!("Board {label} connected");
                    return board;
                }
                Err(e) => format!("not found ({e})"),
            }
        };
        if said.as_deref() != Some(state.as_str()) {
            println!("Board {label} {state}, waiting");
            said = Some(state);
        }
        std::thread::sleep(RETRY);
    }
}

fn main() {
    let node = node_name();
    println!("Attaching to Lucy shared memory for node_name={node}");

    let shm_file = attach_shm(&format!("/{node}"), size_of::<ShmHeader>());

    let mut table_map = unsafe { MmapMut::map_mut(&shm_file).unwrap() };
    let shm = unsafe { SharedTable::from_raw(table_map.as_mut_ptr(), table_map.len()) }
        .unwrap_or_else(|e| panic!("invalid shm layout: {e}"));

    let serial = usb_serial();
    let lock = flash_lock(serial.as_deref());
    let (port, board) = connect(serial.as_deref(), lock.as_deref());

    // Frames are sized at compile time: one `run` per link width. A Mega is built with one
    // slot per virtual_pin of its node, so its width follows the shm.
    // shortcut: assumes a Mega node's virtual_pins are 0..joints-1 (generated configs are);
    // a gap shows up as "frame carries X joints, expected Y". Send the width from the board if
    // that ever changes.
    let width = match board {
        Board::Rp2040 => RP2040_JOINTS,
        Board::Mega => shm.max_actuators() + 1,
    };
    macro_rules! dispatch {
        ($($w:literal)*) => {
            match width {
                $($w => run::<$w>(shm, port, board, serial, lock),)*
                w => panic!("no link for {w} slots (shm declares {} joints)", shm.max_actuators()),
            }
        };
    }
    dispatch!(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33);
}

fn run<const N: usize>(
    mut shm: SharedTable,
    mut port: UsbPort,
    board: Board,
    serial: Option<String>,
    lock: Option<std::path::PathBuf>,
) -> ! {
    let n = shm.max_actuators();
    // Joint slots both sides carry; a Mega's last link slot is its PCA9685 status.
    let joints = if board == Board::Mega { N - 1 } else { N.min(n) };
    if joints != n {
        eprintln!("Warning: shm declares {n} joints but the link carries {joints}; only the first {joints} are forwarded");
    }

    let mut rx_buf = vec![0u8; frame_len(N)];
    let mut decoder = FrameDecoder::<N>::new(&mut rx_buf);
    let mut frame = vec![0u8; frame_len(N)];
    let mut seq: u32 = 0;

    // 40 Hz target: 20 Hz made big servo moves visibly stepped.
    let period = Duration::from_millis(25);
    let mut next_tick = Instant::now();

    let mut problems = ProblemLog::new();
    let mut failed_writes = 0u32;
    let pcas = parse_pca_addresses(&std::env::args().nth(3).unwrap_or_default());
    let mut pcas_missing = None;
    let label = serial.clone().unwrap_or_default();
    let mut last_answer = Instant::now();
    let mut answering = None;
    loop {
        let flashing = lock.as_deref().is_some_and(|l| l.exists());
        if flashing || failed_writes >= 3 {
            if flashing {
                println!("Board {label} is being flashed: releasing the port");
            } else {
                println!("Board {label} stopped answering: reconnecting");
            }
            drop(port);
            port = connect(serial.as_deref(), lock.as_deref()).0;
            decoder.reset();
            failed_writes = 0;
            answering = None;
            next_tick = Instant::now();
            last_answer = next_tick;
        }
        next_tick += period;
        seq = seq.wrapping_add(1);

        let commands = read_commands::<N>(&shm, joints);
        let len = encode(&Frame::new(seq, Body::Command(commands)), &mut frame)
            .expect("frame buffer is frame_len(N)");

        match port.write(&frame[..len]) {
            Ok(()) => {
                failed_writes = 0;
                match wait_reply(&mut port, &mut decoder, seq) {
                    Ok(states) => {
                        write_states(&mut shm, joints, &states);
                        last_answer = Instant::now();
                        if answering != Some(true) {
                            answering = Some(true);
                            println!("Board {label} answering");
                        }
                        if board == Board::Mega {
                            for line in pca_changes(&pcas, states.positions[N - 1], &mut pcas_missing) {
                                println!("{line}");
                            }
                        }
                    }
                    Err(e) => {
                        let backlog = port.port.bytes_to_read();
                        let kind = match e {
                            ReplyError::Corrupt(_) => Problem::Crc,
                            ReplyError::Timeout | ReplyError::Io(_) => Problem::ReadIo,
                        };
                        problems.record(kind, || format!(
                            "Failed to read states frame from MCU: {e} (rx backlog: {backlog:?}), dropping"
                        ));
                    }
                }
            }
            Err(e) => {
                problems.record(Problem::Write, || {
                    format!("Failed to write commands over serial port: {e}, dropping tick")
                });
                failed_writes += 1;
            }
        }

        let now = Instant::now();
        if answering != Some(false) && now - last_answer > SILENT_AFTER {
            answering = Some(false);
            println!("Board {label} not answering");
        }
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            // When a round trip outlasts the period the bridge simply runs as fast as the link
            // allows; only an overrun of a whole period is a stall worth reporting.
            let late = now - next_tick;
            if late > period {
                problems.record(Problem::Late, || format!("Warning: tick took longer than expected {late:?}"));
            }
            next_tick = now;
        }
        problems.flush();
    }
}

#[derive(Clone, Copy)]
enum Problem {
    Crc,
    ReadIo,
    Write,
    Late,
}

const PROBLEM_NAMES: [&str; 4] = [
    "corrupt states frames",
    "states frames lost (timeout or io)",
    "command writes failed",
    "stalled ticks (over a whole period late)",
];
const SUMMARY_PERIOD: Duration = Duration::from_secs(10);

/// Per-tick problems would print up to 40 lines a second: the first of each kind prints in full,
/// later ones are counted and summed up every SUMMARY_PERIOD.
struct ProblemLog {
    counts: [u32; 4],
    reported: [bool; 4],
    since: std::time::Instant,
}

impl ProblemLog {
    fn new() -> Self {
        Self { counts: [0; 4], reported: [false; 4], since: std::time::Instant::now() }
    }

    fn record(&mut self, problem: Problem, detail: impl FnOnce() -> String) {
        let i = problem as usize;
        if self.reported[i] {
            self.counts[i] += 1;
        } else {
            self.reported[i] = true;
            eprintln!("{} (repeats are summed up every {}s)", detail(), SUMMARY_PERIOD.as_secs());
        }
    }

    fn flush(&mut self) {
        if self.since.elapsed() < SUMMARY_PERIOD {
            return;
        }
        let parts: Vec<String> = PROBLEM_NAMES
            .iter()
            .zip(self.counts)
            .filter(|(_, n)| *n > 0)
            .map(|(name, n)| format!("{n} {name}"))
            .collect();
        if !parts.is_empty() {
            eprintln!("Last {}s: {}", SUMMARY_PERIOD.as_secs(), parts.join(", "));
        }
        self.counts = [0; 4];
        self.since = std::time::Instant::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shm_round_trip_forwards_only_the_linked_joints() {
        let n = 3;
        let mut mem = vec![0u64; ShmHeader::size_for(n) / 8];
        let base = mem.as_mut_ptr() as *mut u8;
        unsafe { (*(base as *mut ShmHeader)).max_actuators = n as u32 };
        let mut shm = unsafe { SharedTable::from_raw(base, mem.len() * 8) }.unwrap();
        // hw_commands then hw_torque_enabled, as the hardware interface lays them out.
        let payload = unsafe { std::slice::from_raw_parts_mut(base.add(144) as *mut f64, 5 * n) };
        payload[..n].copy_from_slice(&[1.0, 2.0, 3.0]);
        payload[3 * n..4 * n].copy_from_slice(&[1.0, 0.0, 1.0]);

        // A Mega link: two joints forwarded, last slot left for the PCA status.
        let commands = read_commands::<3>(&shm, 2);
        assert_eq!(commands.positions, [1.0, 2.0, 0.0]);
        assert_eq!(commands.torque_enabled, [true, false, false]);

        write_states(&mut shm, 2, &JointStates { positions: [f64::NAN, 5.0, 256.0] });
        assert!(shm.hw_positions()[0].is_nan());
        assert_eq!(&shm.hw_positions()[1..], [5.0, 0.0]);
        assert_eq!(shm.header().state_seq.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn pca_addresses_parse_hex_and_decimal() {
        assert_eq!(parse_pca_addresses("0x41,64"), vec![0x41, 0x40]);
        assert!(parse_pca_addresses("").is_empty());
    }

    #[test]
    fn pca_changes_print_once_then_on_change() {
        let pcas = [0x41, 0x40];
        let mut last = None;
        // Firmware without the PCA check leaves the slot at 0: nothing to say.
        assert!(pca_changes(&pcas, 0.0, &mut last).is_empty());
        assert_eq!(
            pca_changes(&pcas, (0x100 | 0b10) as f64, &mut last),
            ["PCA9685 0x41 answering", "PCA9685 0x40 not answering: its servos can't move"]
        );
        assert!(pca_changes(&pcas, (0x100 | 0b10) as f64, &mut last).is_empty());
        assert_eq!(pca_changes(&pcas, 0x100 as f64, &mut last), ["PCA9685 0x40 answering"]);
    }
}
