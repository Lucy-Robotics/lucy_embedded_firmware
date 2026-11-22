use lucy_embedded_firmware_feetech::feetech::FeetechBusError;
use lucy_embedded_firmware_feetech::feetech::HomingRecord;
use lucy_embedded_firmware_feetech::feetech::MAX_TICK;
use lucy_embedded_firmware_feetech::feetech::decode_homing_offset;

use crate::fake_bus::FakeBus;
use crate::fixtures::driver;
use crate::fixtures::no_delay;

#[test]
fn verify_tolerates_a_servo_still_busy_writing() {
    let mut bus = FakeBus::with_positions(&[(1, 3000), (2, 1000)]);
    let records = [HomingRecord {
        homing_offset: 5,
        min_limit: 10,
        max_limit: 4000,
    }; 2];
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
    let records = [HomingRecord {
        homing_offset: 0,
        min_limit: 0,
        max_limit: 5000,
    }; 2];

    let result = driver().write_homing(&mut bus, &records, &mut no_delay());

    assert_eq!(result, Err(FeetechBusError::OutOfLimits));
    assert_eq!(bus.servo(1).max, MAX_TICK);
    assert!(bus.servo(1).locked);
}
