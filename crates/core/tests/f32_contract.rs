use core::f32::consts::TAU;

use lucy_embedded_firmware_core::data::ShmHeader;
use lucy_embedded_firmware_core::utils::map_range;

/// Mirrored by `kRustElementSize` in `lucy_ros2_control/test/test_shared_memory_channel.cpp`.
#[test]
fn shm_arrays_hold_four_byte_values() {
    assert_eq!(ShmHeader::size_for(0), 192);
    assert_eq!(ShmHeader::size_for(6), 192 + 9 * 6 * 4);
}

/// A held position must not drift: every Feetech tick has to survive tick -> rad -> tick in f32.
#[test]
fn every_feetech_tick_survives_a_radian_roundtrip() {
    const MAX_PULSE: f32 = 4096.0;
    for pulse in 0..=4096u16 {
        let rad = map_range(pulse as f32, 0.0, MAX_PULSE, 0.0, TAU);
        let back = (map_range(rad, 0.0, TAU, 0.0, MAX_PULSE) + 0.5) as u16;
        assert_eq!(back, pulse, "rad={rad}");
    }
}
