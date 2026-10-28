use std::fs::File;
use std::time::Duration;

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{SharedTable, ShmHeader};
use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod resources;

mod board;
use board::init;

mod usb_port;
mod config;
use config::{get_robot, robot_tick};

const SHM_OPEN_RETRY: Duration = Duration::from_millis(200);
const SHM_OPEN_TIMEOUT: Duration = Duration::from_secs(30);

fn node_name() -> String {
    std::env::args()
        .nth(1)
        .or_else(|| std::env::var("LUCY_NODE_NAME").ok())
        .filter(|n| !n.is_empty())
        .unwrap_or_else(|| "lucy".to_string())
}

fn open_shm(name: &str, min_len: usize) -> File {
    let c_name = CString::new(name).expect("shm name contains a NUL byte");
    let deadline = std::time::Instant::now() + SHM_OPEN_TIMEOUT;

    loop {
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666 as libc::c_uint) };
        if fd != -1 {
            let file = unsafe { File::from_raw_fd(fd) };
            let len = file
                .metadata()
                .unwrap_or_else(|e| panic!("stat on shm object \"{name}\" failed: {e}"))
                .len() as usize;
            if len < min_len {
                panic!("shm object \"{name}\" is {len} bytes, expected at least {min_len}");
            }
            return file;
        }
        let err = std::io::Error::last_os_error();
        if std::time::Instant::now() >= deadline {
            panic!(
                "shm_open(\"{name}\") failed after {SHM_OPEN_TIMEOUT:?}: {err}. \
                 Is the ROS 2 stack running with node_name={}?",
                name.trim_start_matches('/').split('.').next().unwrap_or(name),
            );
        }
        std::thread::sleep(SHM_OPEN_RETRY);
    }
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

    let mut robot = get_robot();
    let mut board = init();

    let mut last_command_seq = ass.header().command_seq.load(core::sync::atomic::Ordering::SeqCst);

    let period = Duration::from_millis(50);
    let mut next_tick = std::time::Instant::now();
    let mut tick_count: u64 = 0;

    loop {
        next_tick += period;

        let command_seq = ass.header().command_seq.load(core::sync::atomic::Ordering::SeqCst);
        if (command_seq % 2) == 0 && command_seq != last_command_seq {
            robot.sync_write_shm(&ass);
            last_command_seq = command_seq;
        }

        robot_tick(tick_count, &mut robot, &mut board);

        ass.header().state_seq.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
        robot.sync_read_shm(&mut ass);
        ass.header().state_seq.fetch_add(1, core::sync::atomic::Ordering::SeqCst);

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
