pub fn map_range(val: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    let in_span = in_max - in_min;

    if in_span.abs() < f32::EPSILON {
        return out_min;
    }

    out_min + (val - in_min) * (out_max - out_min) / in_span
}

pub fn millirad_to_deg(angle_millirad: u16) -> f32 {
    (angle_millirad as f32 / 1000.0).to_degrees()
}

pub fn deg_to_millirad(angle_deg: f32) -> u16 {
    (angle_deg.to_radians() * 1000.0 + 0.5) as u16
}
