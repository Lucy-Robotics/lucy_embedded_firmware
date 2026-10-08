use lucy_embedded_firmware_core::utils::{clamp_pulse, map_range, rad_to_pulse};

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
fn rad_to_pulse_midpoints() {
    let mid_pi = rad_to_pulse(
        core::f32::consts::FRAC_PI_2,
        0.0,
        core::f32::consts::PI,
        1250,
        2500,
    );
    assert_eq!(mid_pi, 1875, "π servo midpoint anomaly: {mid_pi}");

    let mid_3pi2 = rad_to_pulse(
        3.0 * core::f32::consts::FRAC_PI_2 / 2.0,
        0.0,
        3.0 * core::f32::consts::FRAC_PI_2,
        1250,
        2500,
    );
    assert_eq!(mid_3pi2, 1875, "3π/2 servo midpoint anomaly: {mid_3pi2}");
}

#[test]
fn rad_to_pulse_half_step_rounds() {
    let pulse = rad_to_pulse(1.0, 0.0, 4.0, 1250, 2500);
    assert_eq!(pulse, 1563, "half-step rounding anomaly: {pulse}");
}

#[test]
fn clamp_pulse_window() {
    assert_eq!(clamp_pulse(100, 1250, 2500), 1250);
    assert_eq!(clamp_pulse(3000, 1250, 2500), 2500);
    assert_eq!(clamp_pulse(1800, 1250, 2500), 1800);
}
