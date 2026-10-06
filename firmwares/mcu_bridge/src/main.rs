use std::fs::File;
use std::time::Duration;
use core::option::{Option};

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{Table};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use crc::{Crc, CRC_16_MODBUS};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod usb_port;
use usb_port::{UsbPort, UsbPortConfig};

const TABLE_CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

fn table_as_bytes<const N: usize>(table: &Table<N>) -> &[u8] {
    unsafe {
        core::slice::from_raw_parts(
            table as *const Table<N> as *const u8,
            size_of::<Table<N>>(),
        )
    }
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

    let usb_config = UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000);
    let mut port = usb_config.open().expect("Failed to initialize USB Port");

    loop {
        let payload = table_as_bytes(ass);
        let crc = TABLE_CRC.checksum(payload);

        let mut frame = Vec::with_capacity(payload.len() + 2);
        frame.extend_from_slice(payload);
        frame.extend_from_slice(&crc.to_le_bytes());

        port.write(&frame).expect("Failed to write table over serial port");
    }
}
