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

pub fn clamp_pulse(pulse: u16, min_pulse: u16, max_pulse: u16) -> u16 {
    let lo = min_pulse.min(max_pulse);
    let hi = min_pulse.max(max_pulse);
    pulse.clamp(lo, hi)
}
