use std::io::Write;
use std::time::Duration;

use lucy_embedded_firmware_core::data::LinkState;
use lucy_embedded_firmware_host::shm::{Shm, node_name};
use lucy_embedded_firmware_host::signal::{install_stop_handler, stop_requested};

mod resources;

mod maintenance;

mod board;
use board::init;

mod config;
use config::{get_robot, robot_release, robot_tick};

fn main() {
    let mut ass = Shm::attach(&node_name());

    let mut robot = get_robot();
    let mut board = init();

    install_stop_handler();

    let mut last_command_seq = ass
        .header()
        .command_seq
        .load(core::sync::atomic::Ordering::SeqCst);

    let period = Duration::from_millis(50);
    let mut next_tick = std::time::Instant::now();
    let mut tick_count: u64 = 0;
    let mut faulted = false;

    while !stop_requested() {
        next_tick += period;

        let command_seq = ass
            .header()
            .command_seq
            .load(core::sync::atomic::Ordering::SeqCst);
        if command_seq.is_multiple_of(2) && command_seq != last_command_seq {
            robot.sync_write_shm(&ass);
            last_command_seq = command_seq;
        }

        robot_tick(tick_count, &mut robot, &mut board);

        ass.header().set_link_state(if board.connected() {
            LinkState::Connected
        } else {
            LinkState::Disconnected
        });
        ass.header().beat();

        if robot.faulted() != faulted {
            faulted = robot.faulted();
            let _ = writeln!(
                std::io::stderr(),
                "Warning: protection {}",
                if faulted {
                    "tripped, torque held off"
                } else {
                    "released"
                }
            );
        }

        ass.header()
            .state_seq
            .fetch_add(1, core::sync::atomic::Ordering::SeqCst);
        robot.sync_read_shm(&mut ass);
        ass.header()
            .state_seq
            .fetch_add(1, core::sync::atomic::Ordering::SeqCst);

        if stop_requested() {
            break;
        }

        let now = std::time::Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            let _ = writeln!(
                std::io::stderr(),
                "Warning: tick took longer than expected {:?}",
                now - next_tick
            );
            next_tick = now;
        }
        tick_count += 1;
    }

    robot_release(&mut board);
    ass.header().set_link_state(LinkState::Disconnected);
}
