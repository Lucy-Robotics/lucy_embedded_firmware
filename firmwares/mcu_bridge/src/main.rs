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

const STATE_BYTES: usize = size_of::<StateBlock<10>>();

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
        size_of::<Table<10>>(),
    );

    let table_map = unsafe { MmapMut::map_mut(&shm_file).unwrap() };
    let ass: &mut Table<10> = unsafe {
        &mut *(table_map.as_ptr() as *mut Table<10>)
    };

    let usb_config = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000);
    let mut port = usb_config.open().expect("Failed to initialize USB Port");

    let period = Duration::from_millis(50);
    let mut next_tick = std::time::Instant::now();
    let mut tick_count: u64 = 0;

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
            eprintln!("Warning: tick took longer than expected {:?}", now - next_tick);
            next_tick = now;
        }
        tick_count += 1;
    }
}
