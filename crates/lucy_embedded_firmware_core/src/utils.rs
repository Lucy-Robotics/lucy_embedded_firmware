pub fn map_range(val: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    let in_span = in_max - in_min;

    if in_span.abs() < f32::EPSILON {
        return out_min;
    }

    let result = out_min + (val - in_min) * (out_max - out_min) / in_span;

    // Output never exceeds the physical limits of the servo
    let (min_bound, max_bound) = if out_min <= out_max {
        (out_min, out_max)
    } else {
        (out_max, out_min)
    };

    result.clamp(min_bound, max_bound)
}

/// Convert milliradians (`rad * 1000`) to degrees.
pub fn millirad_to_deg(angle_millirad: u16) -> f32 {
    (angle_millirad as f32 / 1000.0).to_degrees()
}

/// Convert degrees to milliradians (`rad * 1000`).
///
/// `no_std` friendly: avoid `f32::round()` — use `(x + 0.5) as u16` for positive values.
pub fn deg_to_millirad(angle_deg: f32) -> u16 {
    (angle_deg.to_radians() * 1000.0 + 0.5) as u16
}

/// Map an angle in milliradians to a PWM duty count with rounding.
pub fn millirad_to_pulse(
    angle_millirad: u16,
    min_angle_deg: u16,
    max_angle_deg: u16,
    min_pulse: u16,
    max_pulse: u16,
) -> u16 {
    let angle_deg = millirad_to_deg(angle_millirad);
    let clamped = angle_deg.clamp(min_angle_deg as f32, max_angle_deg as f32);
    (map_range(
        clamped,
        min_angle_deg as f32,
        max_angle_deg as f32,
        min_pulse as f32,
        max_pulse as f32,
    ) + 0.5) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

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
        // 90° → π/2 rad → ~1571 millirad
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
        // 45° of 0..180 → 1562.5 → rounds to 1563 with +0.5 cast
        let pulse = millirad_to_pulse(deg_to_millirad(45.0), 0, 180, 1250, 2500);
        assert_eq!(pulse, 1563, "half-step rounding anomaly: {pulse}");
    }
}
