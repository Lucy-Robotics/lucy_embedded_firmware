//! Parse a projected firmware spec into one static table the RP2040 image includes.

use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct Spec {
    #[serde(default)]
    pwm: Vec<PwmRowYaml>,
    #[serde(default)]
    i2c_pwm: Vec<I2cRowYaml>,
}

#[derive(Debug, Deserialize)]
struct PwmRowYaml {
    virtual_pin: usize,
    header: u8,
    amplitude_rad: f64,
    min_pulse_us: u16,
    max_pulse_us: u16,
}

#[derive(Debug, Deserialize)]
struct I2cRowYaml {
    virtual_pin: usize,
    address: u8,
    channel: u8,
    amplitude_rad: f64,
    min_pulse_us: u16,
    max_pulse_us: u16,
}

/// Rust source for `PWM_ROWS` and `I2C_ROWS`.
///
/// Empty tables are valid (board with no enabled actuators). Invalid YAML or a
/// hardware-mapping file mistaken for a projected spec still panics.
pub fn render_static_table(yaml: &str) -> String {
    let trimmed = yaml.trim();
    let spec = if trimmed.is_empty() {
        Spec {
            pwm: Vec::new(),
            i2c_pwm: Vec::new(),
        }
    } else {
        // Hardware mapping YAML uses `actuators:`; the projected firmware spec
        // uses `pwm:` / `i2c_pwm:`. Mixing them up must fail the build.
        if trimmed.contains("actuators:")
            && !trimmed.contains("\npwm:")
            && !trimmed.starts_with("pwm:")
        {
            panic!(
                "expected projected firmware spec (pwm:/i2c_pwm:), \
                 got a hardware mapping with actuators:"
            );
        }
        serde_yaml::from_str(yaml).unwrap_or_else(|err| {
            panic!("invalid firmware spec YAML: {err}")
        })
    };
    let mut out = String::from(
        "#[derive(Clone, Copy)]\n\
         pub struct PwmRow { pub virtual_pin: usize, pub gpio: u8, pub amplitude: f64, pub min_pulse: u16, pub max_pulse: u16 }\n\
         #[derive(Clone, Copy)]\n\
         pub struct I2cRow { pub virtual_pin: usize, pub address: u8, pub channel: u8, pub amplitude: f64, pub min_pulse: u16, pub max_pulse: u16 }\n",
    );
    out.push_str("pub static PWM_ROWS: &[PwmRow] = &[\n");
    for row in &spec.pwm {
        let gpio = row.header.saturating_sub(1);
        out.push_str(&format!(
            "    PwmRow {{ virtual_pin: {}, gpio: {}, amplitude: {:?}, min_pulse: {}, max_pulse: {} }},\n",
            row.virtual_pin, gpio, row.amplitude_rad, row.min_pulse_us, row.max_pulse_us
        ));
    }
    out.push_str("];\n");
    out.push_str("pub static I2C_ROWS: &[I2cRow] = &[\n");
    for row in &spec.i2c_pwm {
        out.push_str(&format!(
            "    I2cRow {{ virtual_pin: {}, address: {}, channel: {}, amplitude: {:?}, min_pulse: {}, max_pulse: {} }},\n",
            row.virtual_pin, row.address, row.channel, row.amplitude_rad, row.min_pulse_us, row.max_pulse_us
        ));
    }
    out.push_str("];\n");
    out
}

#[cfg(test)]
mod tests {
    use super::render_static_table;

    #[test]
    fn pwm_row_becomes_one_static_entry() {
        let yaml = r#"
pwm:
  - virtual_pin: 0
    header: 10
    identity: Servo10
    amplitude_rad: 4.71238898
    min_pulse_us: 500
    max_pulse_us: 2500
i2c_pwm: []
"#;
        let code = render_static_table(yaml);
        assert!(code.contains("gpio: 9"));
        assert!(code.contains("I2C_ROWS: &[I2cRow] = &[\n];"));
    }

    #[test]
    #[should_panic(expected = "invalid firmware spec YAML")]
    fn invalid_yaml_fails_closed() {
        let _ = render_static_table("pwm: [");
    }

    #[test]
    fn empty_tables_are_allowed() {
        let code = render_static_table("pwm: []\ni2c_pwm: []\n");
        assert!(code.contains("PWM_ROWS: &[PwmRow] = &[\n];"));
        assert!(code.contains("I2C_ROWS: &[I2cRow] = &[\n];"));
    }
}
