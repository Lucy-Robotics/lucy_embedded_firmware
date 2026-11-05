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

const TORQUE: f64 = 2.94;
const GEAR_RATIO: f64 = 1.0;
const STALL_CURRENT: f64 = 2.7;
const NO_LOAD_CURRENT: f64 = 0.18;
const CURRENT_LSB: f64 = 0.0065;

const fn servo(id: u8) -> FeetechServoConfig {
    FeetechServoConfig {
        id,
        amplitude: 4096.0,
        min_pulse: 0,
        max_pulse: 4096,
        torque: TORQUE,
        gear_ratio: GEAR_RATIO,
        stall_current: STALL_CURRENT,
        no_load_current: NO_LOAD_CURRENT,
        current_lsb: CURRENT_LSB,
    }
}

static CONFIG: FeetechBusConfig<3> = FeetechBusConfig {
    servos: [servo(1), servo(2), servo(3)],
};

/// Expected effort (N·m) for a raw Present Current magnitude, mirroring the driver's model.
fn expected_effort(raw: u16) -> f64 {
    let current = (raw as f64 * CURRENT_LSB - NO_LOAD_CURRENT).max(0.0);
    current * (TORQUE / STALL_CURRENT) * GEAR_RATIO
}

fn status(id: u8, pulse: u16) -> Vec<u8> {
    let [l, h] = pulse.to_le_bytes();
    let body = [id, 4, 0, l, h];
    let sum = body.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
    let mut packet = vec![0xFF, 0xFF];
    packet.extend_from_slice(&body);
    packet.push(!sum);
    packet
}

fn read_efforts(rx: Vec<u8>) -> (bool, Vec<f64>) {
    let mut bus = MockBus { rx: rx.into() };
    let mut state = [State::default(); 3];
    let ok = FeetechBusDriver::<MockBus, 3>::new(&CONFIG)
        .read_effort(&mut bus, &mut state)
        .is_ok();
    (ok, state.iter().map(|s| s.effort).collect())
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

#[test]
fn effort_uses_signed_present_current() {
    let rx = [status(1, 0), status(2, 200), status(3, 0x8000 | 200)].concat();
    let (ok, efforts) = read_efforts(rx);
    assert!(ok);
    assert_eq!(efforts[0], 0.0);
    // 200 * 6.5 mA = 1.3 A, minus 0.18 A no-load = 1.12 A, * (2.94 / 2.7) N·m/A
    assert!((efforts[1] - 1.12 * 2.94 / 2.7).abs() < 1e-9);
    assert!((efforts[1] - expected_effort(200)).abs() < 1e-9);
    assert!((efforts[2] + expected_effort(200)).abs() < 1e-9);
}

#[test]
fn effort_below_no_load_current_is_zero() {
    // 20 * 6.5 mA = 0.13 A < 0.18 A no-load (both signs) -> 0; 28 * 6.5 mA = 0.182 A -> just above.
    let rx = [status(1, 20), status(2, 0x8000 | 20), status(3, 28)].concat();
    let (ok, efforts) = read_efforts(rx);
    assert!(ok);
    assert_eq!(efforts[0], 0.0);
    assert_eq!(efforts[1].abs(), 0.0);
    assert!(efforts[2] > 0.0);
    assert!((efforts[2] - expected_effort(28)).abs() < 1e-9);
}

#[test]
fn effort_scales_with_gear_ratio() {
    static GEARED: FeetechBusConfig<1> = FeetechBusConfig {
        servos: [FeetechServoConfig { gear_ratio: 3.0, ..servo(1) }],
    };
    let mut bus = MockBus { rx: status(1, 200).into() };
    let mut state = [State::default(); 1];
    assert!(FeetechBusDriver::<MockBus, 1>::new(&GEARED)
        .read_effort(&mut bus, &mut state)
        .is_ok());
    assert!((state[0].effort - 3.0 * expected_effort(200)).abs() < 1e-9);
}
