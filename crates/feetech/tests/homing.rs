use std::collections::{HashMap, VecDeque};

use lucy_embedded_firmware_feetech::feetech::{
    FeetechBusConfig, FeetechBusDriver, FeetechBusError, FeetechServoConfig, HALF_TURN_TICK,
    HomingFault, HomingRecord, MAX_TICK, decode_homing_offset, encode_homing_offset,
    half_turn_offset,
};
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;

const REG_MIN: u8 = 9;
const REG_MAX: u8 = 11;
const REG_HOMING: u8 = 31;
const REG_LOCK: u8 = 55;
const REG_PRESENT: u8 = 56;

const SYNC_READ: u8 = 0x82;
const SYNC_WRITE: u8 = 0x83;

/// One servo: an EEPROM that ignores writes while locked, and an encoder.
struct FakeServo {
    actual: u16,
    homing: u16,
    min: u16,
    max: u16,
    locked: bool,
    answers: bool,
    /// Read-backs still to answer with the old value, as a servo busy writing EEPROM could.
    stale_reads: usize,
    /// Homing writes this servo still accepts; later ones are silently lost.
    homing_writes_left: usize,
}

impl FakeServo {
    fn new(actual: u16) -> Self {
        Self {
            actual,
            homing: 0,
            min: 0,
            max: MAX_TICK,
            locked: true,
            answers: true,
            stale_reads: 0,
            homing_writes_left: usize::MAX,
        }
    }

    fn present(&self) -> u16 {
        let offset = decode_homing_offset(self.homing) as i32;
        (self.actual as i32 - offset).rem_euclid(4096) as u16
    }

    fn write(&mut self, register: u8, data: &[u8]) {
        if register == REG_LOCK {
            self.locked = data[0] != 0;
            return;
        }
        if self.locked {
            return;
        }
        let value = u16::from_le_bytes([data[0], data[1]]);
        match register {
            REG_MIN => self.min = value,
            REG_MAX => self.max = value,
            REG_HOMING if self.homing_writes_left > 0 => {
                self.homing_writes_left -= 1;
                self.homing = value;
            }
            _ => {}
        }
    }

    fn read(&self, register: u8) -> u16 {
        match register {
            REG_MIN => self.min,
            REG_MAX => self.max,
            REG_HOMING => self.homing,
            REG_PRESENT => self.present(),
            _ => panic!("unexpected read of register {register}"),
        }
    }
}

#[derive(Default)]
struct FakeBus {
    servos: HashMap<u8, FakeServo>,
    rx: VecDeque<u8>,
    instructions: Vec<u8>,
}

impl FakeBus {
    fn with_positions(ticks: &[(u8, u16)]) -> Self {
        Self {
            servos: ticks.iter().map(|&(id, tick)| (id, FakeServo::new(tick))).collect(),
            ..Self::default()
        }
    }

    fn servo(&mut self, id: u8) -> &mut FakeServo {
        self.servos.get_mut(&id).unwrap()
    }

    fn reply(&mut self, id: u8, value: u16) {
        let [l, h] = value.to_le_bytes();
        let body = [id, 4, 0, l, h];
        let sum = body.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
        self.rx.extend([0xFF, 0xFF]);
        self.rx.extend(body);
        self.rx.push_back(!sum);
    }
}

impl SerialChannel for FakeBus {
    type Error = ();

    fn write(&mut self, bytes: &[u8]) -> Result<(), ()> {
        assert_eq!(&bytes[..3], &[0xFF, 0xFF, 0xFE], "homing must only use sync frames");
        let instruction = bytes[4];
        self.instructions.push(instruction);
        let register = bytes[5];
        let length = bytes[6] as usize;
        let params = &bytes[7..bytes.len() - 1];

        match instruction {
            SYNC_WRITE => {
                for chunk in params.chunks_exact(length + 1) {
                    if let Some(servo) = self.servos.get_mut(&chunk[0]) {
                        servo.write(register, &chunk[1..]);
                    }
                }
            }
            SYNC_READ => {
                for &id in params {
                    let Some(servo) = self.servos.get_mut(&id) else { continue };
                    if !servo.answers {
                        continue;
                    }
                    let value = servo.read(register);
                    let value = if servo.stale_reads > 0 {
                        servo.stale_reads -= 1;
                        value ^ 1
                    } else {
                        value
                    };
                    self.reply(id, value);
                }
            }
            other => panic!("unexpected instruction {other:#x}"),
        }
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, ()> {
        for slot in buffer.iter_mut() {
            *slot = self.rx.pop_front().ok_or(())?;
        }
        Ok(buffer.len())
    }

    fn clear(&mut self) -> Result<(), ()> {
        self.rx.clear();
        Ok(())
    }
}

const fn servo(id: u8) -> FeetechServoConfig {
    FeetechServoConfig {
        id,
        amplitude: core::f32::consts::TAU,
        min_pulse: 0,
        max_pulse: 4096,
        torque: 2.94,
        gear_ratio: 1.0,
        stall_current: 2.7,
        no_load_current: 0.18,
        current_lsb: 0.0065,
    }
}

static CONFIG: FeetechBusConfig<2> = FeetechBusConfig { servos: [servo(1), servo(2)] };

fn driver() -> FeetechBusDriver<FakeBus, 2> {
    FeetechBusDriver::new(&CONFIG)
}

fn no_delay() -> impl FnMut() {
    || {}
}

#[test]
fn encodes_negative_offset_as_sign_magnitude() {
    assert_eq!(encode_homing_offset(100), Some(100));
    assert_eq!(encode_homing_offset(-100), Some(100 | 1 << 11));
    assert_eq!(decode_homing_offset(100 | 1 << 11), -100);
    assert_eq!(decode_homing_offset(2047), 2047);
    assert_eq!(encode_homing_offset(2048), None);
}

#[test]
fn half_turn_offset_matches_lerobot_sign() {
    // LeRobot: homing = position - int((4096 - 1) / 2).
    assert_eq!(half_turn_offset(3000), Some(953));
    assert_eq!(half_turn_offset(1000), Some(-1047));
    assert_eq!(half_turn_offset(HALF_TURN_TICK), Some(0));
    assert_eq!(half_turn_offset(0), Some(-2047));
    assert_eq!(half_turn_offset(4094), Some(2047));
}

#[test]
fn half_turn_offset_refuses_the_one_tick_that_does_not_fit() {
    assert_eq!(half_turn_offset(MAX_TICK), None);
}

#[test]
fn set_half_turn_makes_the_held_pose_read_mid_and_relocks() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);

    let done = driver().set_half_turn(&mut bus, &mut no_delay()).unwrap();

    assert_eq!(done.old, [HomingRecord::CLEARED; 2]);
    assert_eq!(done.new[0].homing_offset, 953);
    assert_eq!(done.new[1].homing_offset, -1047);
    for id in [1, 2] {
        assert_eq!(bus.servo(id).present(), HALF_TURN_TICK);
        assert!(bus.servo(id).locked, "EEPROM must be re-locked");
    }
}

#[test]
fn set_half_turn_replaces_a_previous_calibration() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    bus.servo(1).homing = encode_homing_offset(500).unwrap();
    bus.servo(1).min = 700;
    bus.servo(1).max = 3300;

    let done = driver().set_half_turn(&mut bus, &mut no_delay()).unwrap();

    let previous = HomingRecord { homing_offset: 500, min_limit: 700, max_limit: 3300 };
    assert_eq!(done.old[0], previous);
    // The offset is computed from the raw position, not from the previous frame.
    assert_eq!(done.new[0], HomingRecord { homing_offset: 953, min_limit: 0, max_limit: MAX_TICK });
    assert_eq!(bus.servo(1).present(), HALF_TURN_TICK);
    assert_eq!((bus.servo(1).min, bus.servo(1).max), (0, MAX_TICK));
}

#[test]
fn position_with_no_valid_offset_restores_previous_values() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, MAX_TICK)]);
    bus.servo(1).homing = encode_homing_offset(-250).unwrap();

    let fault = driver().set_half_turn(&mut bus, &mut no_delay()).unwrap_err();

    assert_eq!(
        fault,
        HomingFault { error: FeetechBusError::OffsetOutOfRange(2), restored: true }
    );
    assert_eq!(decode_homing_offset(bus.servo(1).homing), -250);
    assert!(bus.servo(1).locked);
}

#[test]
fn silent_servo_changes_nothing() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    bus.servo(1).homing = encode_homing_offset(42).unwrap();
    bus.servo(2).answers = false;

    let fault = driver().set_half_turn(&mut bus, &mut no_delay()).unwrap_err();

    assert_eq!(
        fault,
        HomingFault { error: FeetechBusError::CommunicationError, restored: true }
    );
    assert_eq!(decode_homing_offset(bus.servo(1).homing), 42);
    assert!(bus.instructions.iter().all(|&i| i == SYNC_READ), "nothing may be written");
}

#[test]
fn write_that_never_sticks_leaves_the_previous_values_in_place() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    bus.servo(2).homing = encode_homing_offset(7).unwrap();
    bus.servo(2).homing_writes_left = 0;

    let fault = driver().set_half_turn(&mut bus, &mut no_delay()).unwrap_err();

    // Servo 2 never left its old value, so reading it back after the restore matches.
    assert_eq!(
        fault,
        HomingFault { error: FeetechBusError::ReadbackMismatch, restored: true }
    );
    assert_eq!(decode_homing_offset(bus.servo(2).homing), 7);
    assert!(bus.servo(1).locked);
}

#[test]
fn restore_that_fails_is_reported_as_not_restored() {
    // Servo 1 accepts the clear, then stops accepting writes, so its old value is lost.
    // Servo 2 sits on the one tick with no valid offset, which is what triggers the restore.
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, MAX_TICK)]);
    bus.servo(1).homing = encode_homing_offset(9).unwrap();
    bus.servo(1).homing_writes_left = 1;

    let fault = driver().set_half_turn(&mut bus, &mut no_delay()).unwrap_err();

    assert_eq!(
        fault,
        HomingFault { error: FeetechBusError::OffsetOutOfRange(2), restored: false }
    );
    assert_eq!(decode_homing_offset(bus.servo(1).homing), 0);
    assert!(bus.servo(1).locked, "EEPROM is re-locked even after a failed write");
}

#[test]
fn verify_tolerates_a_servo_still_busy_writing() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    let records = [HomingRecord { homing_offset: 5, min_limit: 10, max_limit: 4000 }; 2];
    // Answer the first read-back with stale data.
    bus.servo(1).stale_reads = 1;

    let mut delays = 0;
    driver()
        .write_homing(&mut bus, &records, &mut || delays += 1)
        .unwrap();

    assert_eq!(decode_homing_offset(bus.servo(1).homing), 5);
    assert!(delays > 0, "EEPROM writes must leave the servo time");
}

#[test]
fn invalid_record_is_refused_before_touching_eeprom() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    let records = [HomingRecord { homing_offset: 0, min_limit: 0, max_limit: 5000 }; 2];

    let result = driver().write_homing(&mut bus, &records, &mut no_delay());

    assert_eq!(result, Err(FeetechBusError::OutOfLimits));
    assert_eq!(bus.servo(1).max, MAX_TICK);
    assert!(bus.servo(1).locked);
}

#[test]
fn tracked_update_reports_which_servos_answered() {
    use lucy_embedded_firmware_core::joint::{Command, State};

    let mut bus = FakeBus::with_positions(&[(1, 100), (2, 200)]);
    bus.servo(2).answers = false;
    let mut command = [Command::default(); 2];
    let mut state = [State::default(); 2];
    let mut seen = [false; 2];

    // Tick 5 skips the temperature and effort reads.
    let result = driver().update_tracked(5, &mut bus, &mut command, &mut state, &mut seen);

    assert!(result.is_err());
    assert_eq!(seen, [true, false]);
}
