use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use lucy_embedded_firmware_core::data::{LinkState, SharedTable};
use lucy_embedded_firmware_serial::SerialChannel;
use lucy_embedded_firmware_communication::{
    encode, frame_len, Body, Frame, FrameDecoder, JointCommands, JointStates,
};
use lucy_embedded_firmware_host::shm::{node_name, Shm};
use lucy_embedded_firmware_host::signal::{install_stop_handler, stop_requested};
use lucy_embedded_firmware_host::usb_port::{ClearBuffer, UsbPort, UsbPortConfig};

/// Must match `JOINTS` in `firmwares/rp2040/src/config.rs`.
const JOINTS: usize = 6;
const FRAME_LEN: usize = frame_len(JOINTS);

const REPLY_TIMEOUT: Duration = Duration::from_millis(40);
const LINK_TIMEOUT: Duration = Duration::from_millis(200);

fn read_commands(shm: &SharedTable, n: usize) -> JointCommands<JOINTS> {
    loop {
        let before = shm.header().command_seq.load(Ordering::Acquire);
        if before % 2 != 0 {
            std::hint::spin_loop();
            continue;
        }

        let mut commands = JointCommands::<JOINTS>::default();
        commands.positions[..n].copy_from_slice(&shm.hw_commands()[..n]);
        commands.velocities[..n].copy_from_slice(&shm.hw_velocities()[..n]);
        commands.accelerations[..n].copy_from_slice(&shm.hw_accelerations()[..n]);
        for (enabled, &t) in commands.torque_enabled.iter_mut().zip(&shm.hw_torque_enabled()[..n]) {
            *enabled = t >= 0.5;
        }

        if shm.header().command_seq.load(Ordering::Acquire) == before {
            return commands;
        }
    }
}

fn write_states(shm: &mut SharedTable, n: usize, states: &JointStates<JOINTS>) {
    shm.header().state_seq.fetch_add(1, Ordering::SeqCst);
    shm.hw_positions_mut()[..n].copy_from_slice(&states.positions[..n]);
    shm.hw_temperatures_mut()[..n].copy_from_slice(&states.temperatures[..n]);
    shm.hw_efforts_mut()[..n].copy_from_slice(&states.efforts[..n]);
    shm.hw_currents_mut()[..n].copy_from_slice(&states.currents[..n]);
    shm.hw_loads_mut()[..n].copy_from_slice(&states.loads[..n]);
    shm.header().state_seq.fetch_add(1, Ordering::SeqCst);
}

fn wait_reply(
    port: &mut UsbPort,
    decoder: &mut FrameDecoder<'_, JOINTS>,
    seq: u32,
) -> Option<JointStates<JOINTS>> {
    let deadline = Instant::now() + REPLY_TIMEOUT;
    let mut buf = [0u8; 256];
    while Instant::now() < deadline {
        let count = match port.read_available(&mut buf) {
            Ok(count) => count,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(_) => return None,
        };
        for &byte in &buf[..count] {
            match decoder.push_byte(byte) {
                Some(Ok(Frame { seq: s, body: Body::State(states), .. })) if s == seq => {
                    return Some(states);
                }
                Some(Ok(frame)) => eprintln!("Ignoring unexpected frame seq={} (waiting for {seq})", frame.seq),
                Some(Err(e)) => eprintln!("Dropping corrupt frame from rp2040: {e}"),
                None => {}
            }
        }
    }
    None
}

fn main() {
    let mut ass = Shm::attach(&node_name());
    let n = JOINTS.min(ass.max_actuators());
    if ass.max_actuators() != JOINTS {
        eprintln!(
            "Warning: shm declares {} joints but the rp2040 link carries {JOINTS}; only the first {n} are forwarded",
            ass.max_actuators()
        );
    }

    let mut port = UsbPort::new(UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000).with_clear_buffer(ClearBuffer::All));

    install_stop_handler();

    let mut rx_buf = [0u8; FRAME_LEN];
    let mut decoder = FrameDecoder::<JOINTS>::new(&mut rx_buf);
    let mut frame = [0u8; FRAME_LEN];
    let mut seq: u32 = 0;

    let period = Duration::from_millis(50);
    let mut next_tick = Instant::now();
    let mut last_reply: Option<Instant> = None;

    while !stop_requested() {
        next_tick += period;
        seq = seq.wrapping_add(1);

        let request = Frame::new(seq, Body::Command(read_commands(&ass, n)));
        let len = encode(&request, &mut frame).expect("command frame exceeds FRAME_LEN");

        match port.write(&frame[..len]) {
            Ok(()) => match wait_reply(&mut port, &mut decoder, seq) {
                Some(states) => {
                    write_states(&mut ass, n, &states);
                    last_reply = Some(Instant::now());
                }
                None if port.connected() => eprintln!("No state reply from rp2040 for seq={seq}, dropping tick"),
                None => {}
            },
            Err(e) if port.connected() => eprintln!("Failed to write commands over serial port: {e}, dropping tick"),
            Err(_) => {}
        }

        let linked = port.connected() && last_reply.is_some_and(|t| t.elapsed() < LINK_TIMEOUT);
        ass.header().set_link_state(if linked { LinkState::Connected } else { LinkState::Disconnected });
        ass.header().beat();

        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            eprintln!("Warning: tick took longer than expected {:?}", now - next_tick);
            next_tick = now;
        }
    }

    ass.header().set_link_state(LinkState::Disconnected);
}
