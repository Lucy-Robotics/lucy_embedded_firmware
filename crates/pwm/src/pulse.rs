//! Radians to a servo pulse in microseconds. Header PWM and the PCA9685 share this.

/// Map `position` in `0..=amplitude` onto `min_pulse..=max_pulse` microseconds.
pub fn pulse_width_us(position: f64, amplitude: f64, min_pulse: u16, max_pulse: u16) -> u16 {
    if amplitude <= 0.0 || max_pulse <= min_pulse {
        return min_pulse;
    }
    let span = f64::from(max_pulse - min_pulse);
    let fraction = (position / amplitude).clamp(0.0, 1.0);
    let pulse = f64::from(min_pulse) + fraction * span;
    (pulse + 0.5) as u16
}

#[cfg(test)]
mod tests {
    use super::pulse_width_us;

    #[test]
    fn endpoints_and_midpoint() {
        assert_eq!(pulse_width_us(0.0, 3.141592653589793, 500, 2500), 500);
        assert_eq!(pulse_width_us(3.141592653589793, 3.141592653589793, 500, 2500), 2500);
        assert_eq!(pulse_width_us(1.5707963267948966, 3.141592653589793, 500, 2500), 1500);
    }
}
