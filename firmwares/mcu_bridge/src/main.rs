use std::fs::File;
use std::time::Duration;

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{Table, StateBlock, MAX_ACTUATORS};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use lucy_embedded_firmware_communication::{encode_frame, framed_len, read_frame};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod usb_port;
use usb_port::{UsbPort, UsbPortConfig};

const STATE_BYTES: usize = size_of::<StateBlock<MAX_ACTUATORS>>();

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

enum ReadFrameError {
    Io(std::io::Error),
    CrcMismatch,
}

impl std::fmt::Display for ReadFrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            // Covers a plain read timeout (nothing received at all) as well as any other
            // transport error (e.g. device unplugged) — the exact io::ErrorKind tells them apart.
            ReadFrameError::Io(e) => write!(f, "io error ({:?}): {e}", e.kind()),
            // Bytes *did* arrive and a magic byte synced a full frame, but its CRC didn't match
            // — framing/corruption on the wire, not a dead link.
            ReadFrameError::CrcMismatch => write!(f, "CRC mismatch (frame arrived but is corrupt)"),
        }
    }
}

fn read_framed_payload(port: &mut UsbPort, payload_len: usize) -> Result<Vec<u8>, ReadFrameError> {
    let mut frame = vec![0u8; payload_len + 2];
    let ok = read_frame(|buf| port.port.read_exact(buf), &mut frame).map_err(ReadFrameError::Io)?;
    if !ok {
        return Err(ReadFrameError::CrcMismatch);
    }

    frame.truncate(payload_len);
    Ok(frame)
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

/// Flashers (config pipeline, `pixi run firmware-flash-mega`) hold this file while avrdude
/// owns the port: opening it then would corrupt the upload.
fn wait_while_flashing(serial: &str) {
    let lock = std::path::PathBuf::from(format!("/tmp/.lucy_mcu_flashing_{serial}"));
    if lock.exists() {
        println!("Board {serial} is being flashed, waiting for {}", lock.display());
        while lock.exists() {
            std::thread::sleep(Duration::from_millis(500));
        }
    }
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
    if len != min_len {
        panic!("shm object \"{name}\" is {len} bytes, expected at least {min_len}");
    }
    file
}


fn main() {
    let node = node_name();
    println!("Attaching to Lucy shared memory for node_name={node}");

    let shm_file = open_shm(
        &format!("/{node}"),
        size_of::<Table<MAX_ACTUATORS>>(),
    );

    let table_map = unsafe { MmapMut::map_mut(&shm_file).unwrap() };
    let ass: &mut Table<MAX_ACTUATORS> = unsafe {
        &mut *(table_map.as_ptr() as *mut Table<MAX_ACTUATORS>)
    };

    // RP2040 first, else Arduino Mega 2560.
    let serial = usb_serial();
    if let Some(s) = &serial {
        wait_while_flashing(s);
    }
    let mut port = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000)
        .open(serial.as_deref())
        .or_else(|_| {
            let mut port = UsbPortConfig::new(0x2341, 0x0042, 1_000_000).open(serial.as_deref())?;
            // Opening resets the Mega; bytes sent while its bootloader runs keep it from
            // ever starting the firmware, so wait it out before the first frame.
            std::thread::sleep(Duration::from_secs(2));
            port.clear()?;
            Ok::<_, std::io::Error>(port)
        })
        .unwrap_or_else(|e| {
            panic!("No RP2040 / Arduino Mega USB port (serial {}): {e}", serial.as_deref().unwrap_or("any"))
        });

    // 40 Hz target: the Mega round trip is ~31 ms, so in practice ~32 Hz; 20 Hz made big servo
    // moves visibly stepped.
    let period = Duration::from_millis(25);
    let mut next_tick = std::time::Instant::now();

    let mut problems = ProblemLog::new();
    loop {
        next_tick += period;

        let payload = struct_as_bytes(&ass.commands);
        let mut frame = vec![0u8; framed_len(payload.len())];
        encode_frame(payload, &mut frame);

        match port.write(&frame) {
            Ok(()) => {
                match read_framed_payload(&mut port, STATE_BYTES) {
                    Ok(payload) => {
                        struct_as_bytes_mut(&mut ass.states).copy_from_slice(&payload);
                    }
                    Err(e) => {
                        let backlog = port.port.bytes_to_read();
                        let kind = match e {
                            ReadFrameError::CrcMismatch => Problem::Crc,
                            ReadFrameError::Io(_) => Problem::ReadIo,
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
            }
        }

        let now = std::time::Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            // A Mega round trip (~31 ms) outlasts the 25 ms period, so the bridge simply runs as
            // fast as the link allows; only an overrun of a whole period is a stall worth reporting.
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
    "states frames with a bad CRC",
    "states frames lost (io)",
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
