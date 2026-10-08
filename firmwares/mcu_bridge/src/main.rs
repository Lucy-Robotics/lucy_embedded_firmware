use std::fs::File;

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{Table, StateBlock};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use lucy_embedded_firmware_communication::{encode_frame, framed_len, read_frame};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod usb_port;
use usb_port::{UsbPort, UsbPortConfig};

const STATE_BYTES: usize = size_of::<StateBlock<32>>();

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

fn read_framed_payload(port: &mut UsbPort, payload_len: usize) -> Option<Vec<u8>> {
    let mut frame = vec![0u8; payload_len + 2];
    let ok = read_frame(|buf| port.port.read_exact(buf), &mut frame).ok()?;
    if !ok {
        return None;
    }

    frame.truncate(payload_len);
    Some(frame)
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
        size_of::<Table<32>>(),
    );

    let table_map = unsafe { MmapMut::map_mut(&shm_file).unwrap() };
    let ass: &mut Table<32> = unsafe {
        &mut *(table_map.as_ptr() as *mut Table<32>)
    };

    let usb_config = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000);
    let mut port = usb_config.open().expect("Failed to initialize USB Port");

    loop {
        let payload = struct_as_bytes(&ass.commands);
        let mut frame = vec![0u8; framed_len(payload.len())];
        encode_frame(payload, &mut frame);

        port.write(&frame).expect("Failed to write commands over serial port");

        match read_framed_payload(&mut port, STATE_BYTES) {
            Some(payload) => {
                struct_as_bytes_mut(&mut ass.states).copy_from_slice(&payload);
            }
            None => eprintln!("Failed to read states frame from rp2040, dropping"),
        }
    }
}
