use lucy_embedded_firmware_core::utils::{
    deg_to_millirad, map_range, millirad_to_deg, millirad_to_pulse,
};

#[test]
fn map_range_endpoints() {
    assert!((map_range(0.0, 0.0, 180.0, 1250.0, 2500.0) - 1250.0).abs() < 0.01);
    assert!((map_range(180.0, 0.0, 180.0, 1250.0, 2500.0) - 2500.0).abs() < 0.01);
}

#[test]
fn map_range_midpoint_and_clamp() {
    let mid = map_range(90.0, 0.0, 180.0, 1250.0, 2500.0);
    assert!((mid - 1875.0).abs() < 0.01, "midpoint anomaly: {mid}");
    assert!((map_range(-10.0, 0.0, 180.0, 1250.0, 2500.0) - 1250.0).abs() < 0.01);
    assert!((map_range(200.0, 0.0, 180.0, 1250.0, 2500.0) - 2500.0).abs() < 0.01);
}

#[test]
fn millirad_encoding_roundtrip_90_deg() {
    let mr = deg_to_millirad(90.0);
    assert!((mr as i32 - 1571).abs() <= 1, "90° millirad anomaly: {mr}");
    let back = millirad_to_deg(mr);
    assert!((back - 90.0).abs() < 0.1, "roundtrip anomaly: {back}");
}

#[test]
fn millirad_to_pulse_midpoints() {
    let mid_180 = millirad_to_pulse(deg_to_millirad(90.0), 0, 180, 1250, 2500);
    assert_eq!(mid_180, 1875, "180° servo midpoint anomaly: {mid_180}");

    let mid_270 = millirad_to_pulse(deg_to_millirad(135.0), 0, 270, 1250, 2500);
    assert_eq!(mid_270, 1875, "270° servo midpoint anomaly: {mid_270}");

    let mid_300 = millirad_to_pulse(deg_to_millirad(150.0), 0, 300, 1250, 2500);
    assert_eq!(mid_300, 1875, "300° servo midpoint anomaly: {mid_300}");
}

#[test]
fn millirad_to_pulse_half_step_rounds() {
    let pulse = millirad_to_pulse(deg_to_millirad(45.0), 0, 180, 1250, 2500);
    assert_eq!(pulse, 1562, "half-step rounding anomaly: {pulse}");
}
