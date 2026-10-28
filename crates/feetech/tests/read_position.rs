use std::collections::VecDeque;

use lucy_embedded_firmware_core::joint::State;
use lucy_embedded_firmware_feetech::feetech::{FeetechBusConfig, FeetechBusDriver, FeetechServoConfig};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;

struct MockBus {
    rx: VecDeque<u8>,
}

impl SerialChannel for MockBus {
    type Error = ();

    fn write(&mut self, _bytes: &[u8]) -> Result<(), ()> {
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, ()> {
        for slot in buffer.iter_mut() {
            *slot = self.rx.pop_front().ok_or(())?;
        }
        Ok(buffer.len())
    }

    fn clear(&mut self) -> Result<(), ()> {
        Ok(())
    }
}

static CONFIG: FeetechBusConfig<3> = FeetechBusConfig {
    servos: [
        FeetechServoConfig { id: 1, amplitude: 4096.0, min_pulse: 0, max_pulse: 4096 },
        FeetechServoConfig { id: 2, amplitude: 4096.0, min_pulse: 0, max_pulse: 4096 },
        FeetechServoConfig { id: 3, amplitude: 4096.0, min_pulse: 0, max_pulse: 4096 },
    ],
};

fn status(id: u8, pulse: u16) -> Vec<u8> {
    let [l, h] = pulse.to_le_bytes();
    let body = [id, 4, 0, l, h];
    let sum = body.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
    let mut packet = vec![0xFF, 0xFF];
    packet.extend_from_slice(&body);
    packet.push(!sum);
    packet
}

fn read(rx: Vec<u8>) -> (bool, Vec<f64>) {
    let mut bus = MockBus { rx: rx.into() };
    let mut state = [State::default(); 3];
    let ok = FeetechBusDriver::<MockBus, 3>::new(&CONFIG)
        .read_position(&mut bus, &mut state)
        .is_ok();
    (ok, state.iter().map(|s| s.position).collect())
}

#[test]
fn reads_all_servos() {
    let rx = [status(1, 100), status(2, 200), status(3, 300)].concat();
    assert_eq!(read(rx), (true, vec![100.0, 200.0, 300.0]));
}

#[test]
fn skips_echo_and_garbage() {
    // Echo of our SYNC_READ request (ID 0xFE) plus a stray byte before the real replies.
    let echo = vec![0xFF, 0xFF, 0xFE, 0x07, 0x82, 0x38, 0x02, 0x01, 0x02, 0x03, 0x30];
    let rx = [echo, vec![0x42], status(1, 100), status(2, 200), status(3, 300)].concat();
    assert_eq!(read(rx), (true, vec![100.0, 200.0, 300.0]));
}

#[test]
fn skips_corrupt_packet() {
    let mut bad = status(1, 999);
    *bad.last_mut().unwrap() ^= 0xFF;
    let rx = [bad, status(1, 100), status(2, 200), status(3, 300)].concat();
    assert_eq!(read(rx), (true, vec![100.0, 200.0, 300.0]));
}

#[test]
fn missing_servo_does_not_block_others() {
    let rx = [status(1, 100), status(3, 300)].concat();
    let (ok, positions) = read(rx);
    assert!(!ok);
    assert_eq!(positions, vec![100.0, 0.0, 300.0]);
}

#[test]
fn accepts_out_of_order_replies() {
    let rx = [status(3, 300), status(1, 100), status(2, 200)].concat();
    assert_eq!(read(rx), (true, vec![100.0, 200.0, 300.0]));
}
