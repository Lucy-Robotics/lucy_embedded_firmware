use lucy_embedded_firmware_feetech::feetech::FeetechBusError;
use lucy_embedded_firmware_feetech::feetech::HALF_TURN_TICK;
use lucy_embedded_firmware_feetech::feetech::HomingFault;
use lucy_embedded_firmware_feetech::feetech::HomingRecord;
use lucy_embedded_firmware_feetech::feetech::MAX_TICK;
use lucy_embedded_firmware_feetech::feetech::decode_homing_offset;
use lucy_embedded_firmware_feetech::feetech::encode_homing_offset;

use crate::fake_bus::FakeBus;
use crate::fake_bus::SYNC_READ;
use crate::fixtures::driver;
use crate::fixtures::no_delay;

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

    let previous = HomingRecord {
        homing_offset: 500,
        min_limit: 700,
        max_limit: 3300,
    };
    assert_eq!(done.old[0], previous);
    // The offset is computed from the raw position, not from the previous frame.
    assert_eq!(
        done.new[0],
        HomingRecord {
            homing_offset: 953,
            min_limit: 0,
            max_limit: MAX_TICK
        }
    );
    assert_eq!(bus.servo(1).present(), HALF_TURN_TICK);
    assert_eq!((bus.servo(1).min, bus.servo(1).max), (0, MAX_TICK));
}

#[test]
fn position_with_no_valid_offset_restores_previous_values() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, MAX_TICK)]);
    bus.servo(1).homing = encode_homing_offset(-250).unwrap();

    let fault = driver()
        .set_half_turn(&mut bus, &mut no_delay())
        .unwrap_err();

    assert_eq!(
        fault,
        HomingFault {
            error: FeetechBusError::OffsetOutOfRange(2),
            restored: true
        }
    );
    assert_eq!(decode_homing_offset(bus.servo(1).homing), -250);
    assert!(bus.servo(1).locked);
}

#[test]
fn silent_servo_changes_nothing() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    bus.servo(1).homing = encode_homing_offset(42).unwrap();
    bus.servo(2).answers = false;

    let fault = driver()
        .set_half_turn(&mut bus, &mut no_delay())
        .unwrap_err();

    assert_eq!(
        fault,
        HomingFault {
            error: FeetechBusError::CommunicationError,
            restored: true
        }
    );
    assert_eq!(decode_homing_offset(bus.servo(1).homing), 42);
    assert!(
        bus.instructions.iter().all(|&i| i == SYNC_READ),
        "nothing may be written"
    );
}

#[test]
fn write_that_never_sticks_leaves_the_previous_values_in_place() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    bus.servo(2).homing = encode_homing_offset(7).unwrap();
    bus.servo(2).homing_writes_left = 0;

    let fault = driver()
        .set_half_turn(&mut bus, &mut no_delay())
        .unwrap_err();

    // Servo 2 never left its old value, so reading it back after the restore matches.
    assert_eq!(
        fault,
        HomingFault {
            error: FeetechBusError::ReadbackMismatch,
            restored: true
        }
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

    let fault = driver()
        .set_half_turn(&mut bus, &mut no_delay())
        .unwrap_err();

    assert_eq!(
        fault,
        HomingFault {
            error: FeetechBusError::OffsetOutOfRange(2),
            restored: false
        }
    );
    assert_eq!(decode_homing_offset(bus.servo(1).homing), 0);
    assert!(
        bus.servo(1).locked,
        "EEPROM is re-locked even after a failed write"
    );
}
