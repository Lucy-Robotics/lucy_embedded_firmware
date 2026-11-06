use lucy_embedded_firmware_feetech::feetech::HALF_TURN_TICK;
use lucy_embedded_firmware_feetech::feetech::MAX_TICK;
use lucy_embedded_firmware_feetech::feetech::decode_homing_offset;
use lucy_embedded_firmware_feetech::feetech::encode_homing_offset;
use lucy_embedded_firmware_feetech::feetech::half_turn_offset;

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
