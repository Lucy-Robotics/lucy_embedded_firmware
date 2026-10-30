use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{SharedTable, ShmHeader};
use lucy_embedded_firmware_communication::{
    Body, Error as FrameError, Frame, FrameDecoder, JointCommands, JointStates, encode, frame_len,
};
use lucy_embedded_firmware_core::data::{LinkState, SharedTable};
use lucy_embedded_firmware_host::shm::{Shm, node_name};
use lucy_embedded_firmware_host::signal::{install_stop_handler, stop_requested};
use lucy_embedded_firmware_host::usb_port::{ClearBuffer, UsbPort, UsbPortConfig};
use lucy_embedded_firmware_serial::SerialChannel;

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

/// Seqlock read
fn read_commands<const N: usize>(shm: &SharedTable, n: usize) -> JointCommands<N> {
    loop {
        let before = shm.header().command_seq.load(Ordering::Acquire);
        if !before.is_multiple_of(2) {
            std::hint::spin_loop();
            continue;
        }

        let mut commands = JointCommands::<N>::default();
        commands.positions[..n].copy_from_slice(&shm.hw_commands()[..n]);
        commands.velocities[..n].copy_from_slice(&shm.hw_velocities()[..n]);
        commands.accelerations[..n].copy_from_slice(&shm.hw_accelerations()[..n]);
        for (enabled, &t) in commands
            .torque_enabled
            .iter_mut()
            .zip(&shm.hw_torque_enabled()[..n])
        {
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
    shm.hw_temperatures_mut()[..n].copy_from_slice(&states.temperatures[..n]);
    shm.hw_efforts_mut()[..n].copy_from_slice(&states.efforts[..n]);
    shm.hw_currents_mut()[..n].copy_from_slice(&states.currents[..n]);
    shm.hw_loads_mut()[..n].copy_from_slice(&states.loads[..n]);
    shm.header().state_seq.fetch_add(1, Ordering::SeqCst);
}

fn wait_reply<const N: usize>(
    port: &mut UsbPort,
    decoder: &mut FrameDecoder<'_, N>,
    seq: u32,
    corrupt_frames: &mut u32,
) -> Option<JointStates<JOINTS>> {
    let deadline = Instant::now() + REPLY_TIMEOUT;
    let mut buf = [0u8; 256];
    let mut corrupt = None;
    while Instant::now() < deadline {
        let count = match port.read_available(&mut buf) {
            Ok(count) => count,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(ReplyError::Io(e)),
        };
        for &byte in &buf[..count] {
            match decoder.push_byte(byte) {
                Some(Ok(Frame { seq: s, body: Body::State(states), .. })) if s == seq => {
                    *corrupt_frames = 0;
                    return Some(states);
                }
                Some(Ok(frame)) => eprintln!("Ignoring unexpected frame seq={} (waiting for {seq})", frame.seq),
                Some(Err(e)) => {
                    *corrupt_frames = corrupt_frames.saturating_add(1);
                    // Old UF2s speak a different wire format; don't flood the launch log.
                    if *corrupt_frames == 1 || *corrupt_frames == 10 || *corrupt_frames == 100 {
                        eprintln!(
                            "Dropping corrupt frame from rp2040: {e} (count={corrupt_frames}). \
                             Board firmware is likely outdated vs this mcu_bridge — \
                             Activate → flash a UF2 from the current tip, then relaunch."
                        );
                    }
                }
                None => {}
            }
        };
        if said != Some(state) {
            println!("Board {label} {state}, waiting");
            said = Some(state);
        }
        std::thread::sleep(RETRY);
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
    dispatch!(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33);
}

    println!("Opening Pico serial {} for node_name={node}", bridge.serial);
    let usb_config = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000, bridge.serial);
    let mut port = match usb_config.open() {
        Ok(port) => port,
        Err(err) => die(usb_port::diagnose_open_failure(&usb_config, &bridge.device, &err)),
    };
    port.clear().ok();

    let mut rx_buf = vec![0u8; frame_len(N)];
    let mut decoder = FrameDecoder::<N>::new(&mut rx_buf);
    let mut frame = vec![0u8; frame_len(N)];
    let mut seq: u32 = 0;
    let mut corrupt_frames: u32 = 0;
    let mut missed_replies: u32 = 0;

    // 20 Hz looks stepped
    let period = Duration::from_millis(25);
    let mut next_tick = Instant::now();
    let mut last_reply: Option<Instant> = None;

    let mut problems = ProblemLog::new();
    let pcas = parse_pca_addresses(&std::env::args().nth(3).unwrap_or_default());
    let mut pcas_missing = None;
    let label = serial.clone().unwrap_or_default();
    let mut last_answer = Instant::now();
    let mut answering = None;
    while !stop_requested() {
        if lock.as_deref().is_some_and(|l| l.exists()) {
            println!("Board {label} is being flashed: releasing the port");
            drop(port);
            shm.header().set_link_state(LinkState::Disconnected);
            port = match connect(serial.as_deref(), lock.as_deref()) {
                Some((port, _)) => port,
                None => break,
            };
            decoder.reset();
            answering = None;
            last_reply = None;
            next_tick = Instant::now();
            last_answer = next_tick;
        }
        next_tick += period;
        seq = seq.wrapping_add(1);

        let commands = read_commands::<N>(&shm, joints);
        let len = encode(&Frame::new(seq, Body::Command(commands)), &mut frame)
            .expect("frame buffer is frame_len(N)");

        match port.write(&frame[..len]) {
            Ok(()) => match wait_reply(&mut port, &mut decoder, seq, &mut corrupt_frames) {
                Some(states) => {
                    missed_replies = 0;
                    write_states(&mut ass, n, &states);
                }
                None => {
                    missed_replies = missed_replies.saturating_add(1);
                    if missed_replies == 1 || missed_replies == 10 || missed_replies == 100 {
                        eprintln!(
                            "No state reply from rp2040 for seq={seq} (missed={missed_replies}). \
                             If corrupt-frame warnings also appear, reflash the UF2 from this tip."
                        );
                    }
                }
            },
            Err(e) => {
                problems.record(Problem::Write, || {
                    format!("Failed to write commands over serial port: {e}, dropping tick")
                });
            }
            Err(_) => {}
        }

        let linked = port.connected() && last_reply.is_some_and(|t| t.elapsed() < LINK_TIMEOUT);
        shm.header().set_link_state(if linked {
            LinkState::Connected
        } else {
            LinkState::Disconnected
        });
        shm.header().beat();

        let now = Instant::now();
        if answering != Some(false) && now - last_answer > SILENT_AFTER {
            answering = Some(false);
            println!("Board {label} not answering");
            if board == Board::Mega {
                std::thread::sleep(BOOTLOADER_TIMEOUT);
                next_tick = Instant::now();
                continue;
            }
        }
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            let late = now - next_tick;
            if late > period {
                problems.record(Problem::Late, || {
                    format!("Warning: tick took longer than expected {late:?}")
                });
            }
            next_tick = now;
        }
        problems.flush();
    }

    shm.header().set_link_state(LinkState::Disconnected);
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

struct ProblemLog {
    counts: [u32; 4],
    reported: [bool; 4],
    since: std::time::Instant,
}

impl ProblemLog {
    fn new() -> Self {
        Self {
            counts: [0; 4],
            reported: [false; 4],
            since: std::time::Instant::now(),
        }
    }

    fn record(&mut self, problem: Problem, detail: impl FnOnce() -> String) {
        let i = problem as usize;
        if self.reported[i] {
            self.counts[i] += 1;
        } else {
            self.reported[i] = true;
            eprintln!(
                "{} (repeats are summed up every {}s)",
                detail(),
                SUMMARY_PERIOD.as_secs()
            );
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
        self.reported = self.counts.map(|n| n > 0);
        self.counts = [0; 4];
        self.since = std::time::Instant::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lucy_embedded_firmware_core::data::ShmHeader;

    #[test]
    fn shm_round_trip_forwards_only_the_linked_joints() {
        let n = 3;
        let mut mem = vec![0u64; ShmHeader::size_for(n).div_ceil(8)];
        let base = mem.as_mut_ptr() as *mut u8;
        unsafe { (*(base as *mut ShmHeader)).max_actuators = n as u32 };
        let mut shm = unsafe { SharedTable::from_raw(base, mem.len() * 8) }.unwrap();
        let payload = unsafe { std::slice::from_raw_parts_mut(base.add(144) as *mut f32, 5 * n) };
        payload[..n].copy_from_slice(&[1.0, 2.0, 3.0]);
        payload[3 * n..4 * n].copy_from_slice(&[1.0, 0.0, 1.0]);

        let commands = read_commands::<3>(&shm, 2);
        assert_eq!(commands.positions, [1.0, 2.0, 0.0]);
        assert_eq!(commands.torque_enabled, [true, false, false]);

        let states = JointStates {
            positions: [f32::NAN, 5.0, 256.0],
            temperatures: [41.0, 42.0, 43.0],
            efforts: [0.1, 0.2, 0.3],
            currents: [1.0, 2.0, 3.0],
            loads: [-1.0, -2.0, -3.0],
        };
        write_states(&mut shm, 2, &states);
        assert!(shm.hw_positions()[0].is_nan());
        assert_eq!(&shm.hw_positions()[1..], [5.0, 0.0]);
        assert_eq!(shm.hw_temperatures(), [41.0, 42.0, 0.0]);
        assert_eq!(shm.hw_efforts(), [0.1, 0.2, 0.0]);
        assert_eq!(shm.hw_currents(), [1.0, 2.0, 0.0]);
        assert_eq!(shm.hw_loads(), [-1.0, -2.0, 0.0]);
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
        assert!(pca_changes(&pcas, 0.0, &mut last).is_empty());
        assert_eq!(
            pca_changes(&pcas, (0x100 | 0b10) as f32, &mut last),
            [
                "PCA9685 0x41 answering",
                "PCA9685 0x40 not answering: its servos can't move"
            ]
        );
        assert!(pca_changes(&pcas, (0x100 | 0b10) as f32, &mut last).is_empty());
        assert_eq!(
            pca_changes(&pcas, 0x100 as f32, &mut last),
            ["PCA9685 0x40 answering"]
        );
    }
}
