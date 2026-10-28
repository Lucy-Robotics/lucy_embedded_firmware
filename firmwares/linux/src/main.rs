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

fn die(message: impl std::fmt::Display) -> ! {
    let message = message.to_string();
    eprintln!("{message}");
    // The launcher TUI reads these when the Real Hardware row is CRASHED.
    let _ = std::fs::write("/tmp/.lucy_real_hardware_status", "1\n");
    let _ = std::fs::write("/tmp/.lucy_real_hardware_error", &message);
    std::process::exit(1);
}

struct HostArgs {
    node: String,
    device: String,
    vid: u16,
    pid: u16,
}

fn parse_u16_id(text: &str) -> Option<u16> {
    let text = text.trim();
    if let Some(hex) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16).ok()
    } else {
        text.parse().ok()
    }
}

fn host_args() -> HostArgs {
    let incomplete = "This robot configuration does not name the device to open. Fix it, then relaunch Lucy.";
    let mut args = std::env::args().skip(1);
    let Some(node) = args.next().filter(|s| !s.is_empty()) else {
        die(incomplete);
    };
    let Some(device) = args.next().filter(|s| !s.is_empty()) else {
        die(incomplete);
    };
    let Some(vid) = args.next().and_then(|s| parse_u16_id(&s)) else {
        die(incomplete);
    };
    let Some(pid) = args.next().and_then(|s| parse_u16_id(&s)) else {
        die(incomplete);
    };
    HostArgs { node, device, vid, pid }
}

fn open_shm(name: &str, min_len: usize) -> File {
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
            if len < min_len {
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
    let host = host_args();
    let node = host.node;
    println!("Attaching to Lucy shared memory for node_name={node}");

    let shm_file = open_shm(
        &format!("/{node}"),
        size_of::<ShmHeader>(),
    );

    let table_map = match unsafe { MmapMut::map_mut(&shm_file) } {
        Ok(map) => map,
        Err(_) => die("Lucy could not attach to the robot. Relaunch Lucy."),
    };
    let mut ass = unsafe { SharedTable::from_raw(table_map.as_ptr() as *mut u8, table_map.len()) }
        .unwrap_or_else(|_| die("This robot configuration does not match Lucy. Relaunch Lucy."));

    let mut robot = get_robot();
    let mut board = match init(host.vid, host.pid) {
        Ok(board) => board,
        Err(err) => die(usb_port::user_message(err.kind(), &host.device)),
    };

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
