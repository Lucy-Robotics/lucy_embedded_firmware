use std::fs::File;
use std::time::Duration;
use core::option::{Option};

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{Table};
use lucy_embedded_firmware_core::robot::{Robot};

use lucy_embedded_firmware_core::joint::{*};

//use lucy_embedded_firmware_pwm::pwm_servo::{PwmServoConfig, PwmServoDriver, PwmServoError};
use lucy_embedded_firmware_feetech::feetech::{FeetechServoConfig, FeetechBusDriver};
//use lucy_embedded_firmware_core::drivers::bus_servo::{BusServoDriver, BusServoConfig};
//use lucy_embedded_firmware_core::actuator::{JointTrajectoryPoint, JointTrajectoryInterface, TorqueStatus, TorqueEnableInterface, JointStateInterface, TemperatureInterface, TorqueInterface};

use core::f32::consts::{PI, TAU};

use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

mod resources;
use resources::Resources;

mod board;
use board::init;

mod usb_port;
use usb_port::UsbPort;
mod config;
use config::{get_robot, robot_tick};

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

    let mut robot = get_robot();
    let mut board = init();

    let last_command_seq = ass.commands.command_seq.load(core::sync::atomic::Ordering::SeqCst);
    let last_state_seq = ass.states.state_seq.load(core::sync::atomic::Ordering::SeqCst);

    let period = Duration::from_millis(50);
    let mut next_tick = std::time::Instant::now();
    let mut tick_count: u64 = 0;

    loop {
        next_tick += period;

        let command_seq = ass.commands.command_seq.load(core::sync::atomic::Ordering::SeqCst);
        if (command_seq % 2) == 0 && command_seq != last_command_seq {
            robot.sync_write(&ass.commands);
        }

        robot_tick(tick_count, &mut robot, &mut board);

        ass.states.state_seq.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
        robot.sync_read(&mut ass.states);
        ass.states.state_seq.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
        
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
