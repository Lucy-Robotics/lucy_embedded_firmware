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

/// Map an angle in **radians** to a PWM / bus pulse (µs or STS ticks) with rounding.
pub fn rad_to_pulse(
    angle_rad: f32,
    min_angle_rad: f32,
    max_angle_rad: f32,
    min_pulse: u16,
    max_pulse: u16,
) -> u16 {
    let lo = min_angle_rad.min(max_angle_rad);
    let hi = min_angle_rad.max(max_angle_rad);
    let clamped = angle_rad.clamp(lo, hi);
    (map_range(
        clamped,
        min_angle_rad,
        max_angle_rad,
        min_pulse as f32,
        max_pulse as f32,
    ) + 0.5) as u16
}

/// Clamp a wire pulse into the configured window.
pub fn clamp_pulse(pulse: u16, min_pulse: u16, max_pulse: u16) -> u16 {
    let lo = min_pulse.min(max_pulse);
    let hi = min_pulse.max(max_pulse);
    pulse.clamp(lo, hi)
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
        // Map 1 of 0..4 rad span → 1562.5 → rounds to 1563 with +0.5 cast
        let pulse = rad_to_pulse(1.0, 0.0, 4.0, 1250, 2500);
        assert_eq!(pulse, 1563, "half-step rounding anomaly: {pulse}");
    }

    #[test]
    fn clamp_pulse_window() {
        assert_eq!(clamp_pulse(100, 1250, 2500), 1250);
        assert_eq!(clamp_pulse(3000, 1250, 2500), 2500);
        assert_eq!(clamp_pulse(1800, 1250, 2500), 1800);
    }
}
