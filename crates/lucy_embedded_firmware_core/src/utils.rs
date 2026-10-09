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

pub fn millirad_to_deg(angle_millirad: u16) -> f32 {
    (angle_millirad as f32 / 1000.0).to_degrees()
}

pub fn deg_to_millirad(angle_deg: f32) -> u16 {
    (angle_deg.to_radians() * 1000.0 + 0.5) as u16
}

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
