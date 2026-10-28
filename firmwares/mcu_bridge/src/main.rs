use std::fs::File;
use std::time::Duration;

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{Table, StateBlock};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use lucy_embedded_firmware_communication::{encode_frame, framed_len, read_frame};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod usb_port;
use usb_port::{UsbPort, UsbPortConfig};

/// Must match HI ``MAX_ACTUATORS`` / ``ActuatorSharedState`` size.
const SHM_ACTUATORS: usize = 32;
/// Misses below this are serial-timeout jitter on the 50 ms tick.
const TICK_OVERRUN_WARN: Duration = Duration::from_millis(5);
const STATE_BYTES: usize = size_of::<StateBlock<SHM_ACTUATORS>>();
/// Wait for LucySystemHardware to create SHM (bringup may start the bridge first).
const SHM_OPEN_RETRY: Duration = Duration::from_millis(200);
const SHM_OPEN_TIMEOUT: Duration = Duration::from_secs(30);

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

fn open_shm(name: &str, expected_len: usize) -> File {
    let c_name = CString::new(name).expect("shm name contains a NUL byte");
    let deadline = std::time::Instant::now() + SHM_OPEN_TIMEOUT;

    loop {
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666 as libc::c_uint) };
        if fd != -1 {
            let file = unsafe { File::from_raw_fd(fd) };
            let len = match file.metadata() {
                Ok(meta) => meta.len() as usize,
                Err(_) => die("Lucy could not attach to the robot. Relaunch Lucy."),
            };
            if len != expected_len {
                die("This robot configuration does not match Lucy. Relaunch Lucy.");
            }
            return file;
        }
        if std::time::Instant::now() >= deadline {
            die("Lucy did not finish starting. Relaunch Lucy.");
        }
        std::thread::sleep(SHM_OPEN_RETRY);
    }
}


fn main() {
    let bridge = bridge_args();
    let node = bridge.node;
    println!("Attaching to Lucy shared memory for node_name={node}");

    let shm_file = open_shm(
        &format!("/{node}"),
        size_of::<Table<SHM_ACTUATORS>>(),
    );

    let table_map = match unsafe { MmapMut::map_mut(&shm_file) } {
        Ok(map) => map,
        Err(_) => die("Lucy could not attach to the robot. Relaunch Lucy."),
    };
    let ass: &mut Table<SHM_ACTUATORS> = unsafe {
        &mut *(table_map.as_ptr() as *mut Table<SHM_ACTUATORS>)
    };

    println!("Opening Pico serial {} for node_name={node}", bridge.serial);
    let usb_config = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000, bridge.serial);
    let mut port = match usb_config.open() {
        Ok(port) => port,
        Err(err) => die(usb_port::user_fix(err.kind(), &bridge.device)),
    };

    let period = Duration::from_millis(50);
    let mut next_tick = std::time::Instant::now();

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
                        eprintln!(
                            "Failed to read states frame from rp2040: {e} (rx backlog: {backlog:?}), dropping"
                        );
                    }
                }
            }
            Err(e) => {
                eprintln!("Failed to write commands over serial port: {e}, dropping tick");
            }
        }

        let now = std::time::Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            let late = now.saturating_duration_since(next_tick);
            if late >= TICK_OVERRUN_WARN {
                eprintln!("Warning: tick took longer than expected {late:?}");
            }
            next_tick = now;
        }
    }
}
