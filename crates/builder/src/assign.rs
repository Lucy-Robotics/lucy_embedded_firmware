//! Virtual-pin / register assignment from parsed firmware YAML.

use crate::schema::{
    AssignmentPlan, BuildError, ConfigValue, DeviceAssignment, DeviceKind, FirmwareConfig,
    BUS_SERVO_REGS, PRESSURE_SENSOR_REGS, PWM_SERVO_REGS,
};
use lucy_embedded_firmware_core::board_layout::{
    layout_for_board, BoardLayout, HardwareIdentity,
};
use std::collections::{BTreeMap, BTreeSet};

/// Servo2040 `HAS_BUS` claims GPIO0–2 for UART0 TX/RX/DIR (Servo1–3 silk).
const SERVO2040_UART_GPIO: [u8; 3] = [0, 1, 2];

/// PWM slice aliases on Servo2040: `(gpio/2)%8` + A/B bit collide for these pairs.
const SERVO2040_PWM_ALIAS_PAIRS: [(u8, u8); 2] = [(0, 16), (1, 17)];

/// Reject YAML that would silently lose PWM pads or steal UART0 with an empty bank.
fn validate_servo2040_uart_pinmux(plan: &AssignmentPlan) -> Result<(), BuildError> {
    let mut bus_devices: Vec<&DeviceAssignment> = Vec::new();
    let mut pwm_on_uart_pads: Vec<&DeviceAssignment> = Vec::new();
    for d in &plan.devices {
        match d.hardware {
            HardwareIdentity::UartBus { .. } => bus_devices.push(d),
            HardwareIdentity::PwmGpio { gpio, .. }
                if SERVO2040_UART_GPIO.contains(&gpio) =>
            {
                pwm_on_uart_pads.push(d);
            }
            _ => {}
        }
    }
    if bus_devices.is_empty() {
        return Ok(());
    }
    for d in &bus_devices {
        if let HardwareIdentity::UartBus { uart, .. } = d.hardware {
            if uart != 0 {
                return Err(BuildError::PinmuxConflict {
                    id: d.id.clone(),
                    detail: format!(
                        "Servo2040 only wires UART0 on GPIO0–2; got UART{uart} (channel `{}`)",
                        d.channel
                    ),
                });
            }
        }
    }
    if let Some(pwm) = pwm_on_uart_pads.first() {
        let bus = bus_devices[0];
        return Err(BuildError::PinmuxConflict {
            id: pwm.id.clone(),
            detail: format!(
                "UART bus (`{}`) owns GPIO0–2; cannot also enable PWM on `{}` (Servo1–3)",
                bus.channel, pwm.channel
            ),
        });
    }
    Ok(())
}

/// Reject bare `UART0` / `UART0:0` (device id 0 is the empty-slot sentinel).
fn validate_uart_bus_device_ids(plan: &AssignmentPlan) -> Result<(), BuildError> {
    for d in &plan.devices {
        if let HardwareIdentity::UartBus { device_id, .. } = d.hardware {
            match device_id {
                None => {
                    return Err(BuildError::PinmuxConflict {
                        id: d.id.clone(),
                        detail: format!(
                            "UART bus channel `{}` needs a Feetech id (`UART0:1`), not a bare UART lane",
                            d.channel
                        ),
                    });
                }
                Some(0) => {
                    return Err(BuildError::PinmuxConflict {
                        id: d.id.clone(),
                        detail: format!(
                            "Feetech device id 0 is reserved (channel `{}`); use UART0:1..N",
                            d.channel
                        ),
                    });
                }
                Some(_) => {}
            }
        }
    }
    Ok(())
}

/// Reject PWM pairs that share a RP2040 slice channel (GPIO0/16, GPIO1/17).
fn validate_servo2040_pwm_aliases(plan: &AssignmentPlan) -> Result<(), BuildError> {
    let mut by_gpio: BTreeMap<u8, &DeviceAssignment> = BTreeMap::new();
    for d in &plan.devices {
        if let HardwareIdentity::PwmGpio { gpio, .. } = d.hardware {
            by_gpio.insert(gpio, d);
        }
    }
    for &(a, b) in &SERVO2040_PWM_ALIAS_PAIRS {
        if let (Some(da), Some(db)) = (by_gpio.get(&a), by_gpio.get(&b)) {
            return Err(BuildError::PinmuxConflict {
                id: db.id.clone(),
                detail: format!(
                    "PWM GPIO{a} (`{}`) and GPIO{b} (`{}`) share a slice channel; enable only one",
                    da.channel, db.channel
                ),
            });
        }
    }
    Ok(())
}

fn validate_assignment_plan(plan: &AssignmentPlan) -> Result<(), BuildError> {
    validate_uart_bus_device_ids(plan)?;
    validate_servo2040_uart_pinmux(plan)?;
    validate_servo2040_pwm_aliases(plan)?;
    Ok(())
}

fn driver_block_size(driver: &str) -> Option<u16> {
    let d = driver.to_ascii_lowercase();
    if d.contains("pwmservo") || d.contains("pwm_servo") {
        Some(PWM_SERVO_REGS)
    } else if d.contains("busservo") || d.contains("bus_servo") {
        Some(BUS_SERVO_REGS)
    } else if d.contains("pressure") {
        Some(PRESSURE_SENSOR_REGS)
    } else {
        None
    }
}

pub(crate) fn config_type_for_driver(driver: &str) -> &'static str {
    let d = driver.to_ascii_lowercase();
    if d.contains("pwmservo") || d.contains("pwm_servo") {
        "PwmServoConfig"
    } else if d.contains("busservo") || d.contains("bus_servo") {
        "BusServoConfig"
    } else if d.contains("pressure") {
        "PressureSensorConfig"
    } else {
        "UnknownConfig"
    }
}
fn yaml_f64(value: &serde_yaml::Value) -> Option<f64> {
    match value {
        serde_yaml::Value::Number(n) => n.as_f64().or_else(|| n.as_i64().map(|i| i as f64)),
        serde_yaml::Value::String(s) => s.parse().ok(),
        _ => None,
    }
}

fn yaml_u16(value: &serde_yaml::Value) -> Option<u16> {
    match value {
        serde_yaml::Value::Number(n) => {
            if let Some(i) = n.as_u64() {
                u16::try_from(i).ok()
            } else if let Some(f) = n.as_f64() {
                if !f.is_finite() || f < 0.0 || f > f64::from(u16::MAX) {
                    None
                } else {
                    Some((f + 0.5) as u16)
                }
            } else if let Some(i) = n.as_i64() {
                u16::try_from(i).ok()
            } else {
                None
            }
        }
        serde_yaml::Value::String(s) => s.parse().ok(),
        _ => None,
    }
}

#[cfg(test)]
mod yaml_u16_tests {
    use super::yaml_u16;
    use serde_yaml::Value;

    #[test]
    fn rejects_out_of_range_integers() {
        assert_eq!(yaml_u16(&Value::from(70_000u64)), None);
        assert_eq!(yaml_u16(&Value::from(-1i64)), None);
        assert_eq!(yaml_u16(&Value::from(4095u64)), Some(4095));
    }

    #[test]
    fn rejects_out_of_range_floats() {
        assert_eq!(yaml_u16(&Value::from(1.0e9_f64)), None);
        assert_eq!(yaml_u16(&Value::from(-0.5_f64)), None);
        assert_eq!(yaml_u16(&Value::from(2000.4_f64)), Some(2000));
    }
}

/// Convert actuator/sensor `config` map: angle keys stay **radians (f32)**;
/// pulse / value keys → u16.
pub fn normalize_config_fields(
    kind: DeviceKind,
    raw: &BTreeMap<String, serde_yaml::Value>,
) -> BTreeMap<String, ConfigValue> {
    let mut out = BTreeMap::new();
    for (k, v) in raw {
        let is_angle = matches!(
            k.as_str(),
            "min_angle" | "max_angle" | "default_angle"
        );
        if is_angle {
            if let Some(rad) = yaml_f64(v) {
                out.insert(k.clone(), ConfigValue::F32(rad as f32));
            }
        } else if let Some(u) = yaml_u16(v) {
            out.insert(k.clone(), ConfigValue::U16(u));
        }
    }
    // Sensors default min/max if omitted
    if kind == DeviceKind::Sensor {
        out.entry("min_value".into())
            .or_insert(ConfigValue::U16(0));
        out.entry("max_value".into())
            .or_insert(ConfigValue::U16(4095));
    }
    out
}

/// Assign virtual pins / register bases using the board layout.
///
/// When YAML carries host ``virtual_pin``, that value is kept and
/// ``base_register = virtual_pin * block_size`` so firmware matches
/// ``LucySystemHardware``. Otherwise pins are assigned densely 0..K-1 among
/// enabled devices (legacy local builds).
pub fn assign_devices(
    config: &FirmwareConfig,
    layout: &dyn BoardLayout,
) -> Result<AssignmentPlan, BuildError> {
    let mut devices = Vec::new();
    let mut used_bases: BTreeSet<u16> = BTreeSet::new();
    let mut used_channels: BTreeSet<String> = BTreeSet::new();
    let mut used_virtual_pins: BTreeSet<u16> = BTreeSet::new();
    let mut next_auto_pin: u16 = 0;

    let mut push = |id: String,
                    kind: DeviceKind,
                    driver: String,
                    channel: String,
                    host_virtual_pin: Option<u16>,
                    raw_config: &BTreeMap<String, serde_yaml::Value>|
     -> Result<(), BuildError> {
        if !used_channels.insert(channel.clone()) {
            return Err(BuildError::RegisterCollision {
                id: id.clone(),
                base: 0,
            });
        }
        let hardware = layout.resolve(&channel).ok_or_else(|| BuildError::UnknownChannel {
            id: id.clone(),
            channel: channel.clone(),
        })?;
        let nb = driver_block_size(&driver).ok_or_else(|| BuildError::UnknownDriver {
            id: id.clone(),
            driver: driver.clone(),
        })?;

        let virtual_pin = if let Some(vp) = host_virtual_pin {
            vp
        } else {
            let vp = next_auto_pin;
            next_auto_pin = next_auto_pin.saturating_add(1);
            vp
        };
        if !used_virtual_pins.insert(virtual_pin) {
            return Err(BuildError::RegisterCollision {
                id: id.clone(),
                base: virtual_pin.saturating_mul(nb),
            });
        }
        let next_base = virtual_pin.saturating_mul(nb);

        for r in next_base..next_base.saturating_add(nb) {
            if used_bases.contains(&r) {
                return Err(BuildError::RegisterCollision {
                    id: id.clone(),
                    base: next_base,
                });
            }
        }

        let fields = normalize_config_fields(kind, raw_config);
        let assignment = DeviceAssignment {
            id,
            kind,
            driver,
            channel,
            hardware,
            virtual_pin,
            base_register: next_base,
            nb_registers: nb,
            fields,
        };
        for r in next_base..next_base.saturating_add(nb) {
            used_bases.insert(r);
        }
        devices.push(assignment);
        Ok(())
    };

    for a in config.actuators.iter().filter(|a| a.enabled) {
        push(
            a.id.clone(),
            DeviceKind::Actuator,
            a.hardware.driver.clone(),
            a.hardware.channel.clone(),
            a.virtual_pin,
            &a.config,
        )?;
    }
    for s in config.sensors.iter().filter(|s| s.enabled) {
        push(
            s.id.clone(),
            DeviceKind::Sensor,
            s.hardware.driver.clone(),
            s.hardware.channel.clone(),
            s.virtual_pin,
            &s.config,
        )?;
    }

    let plan = AssignmentPlan {
        board_id: config
            .board_id
            .clone()
            .unwrap_or_else(|| "unknown".into()),
        board_class: config
            .board_class
            .clone()
            .unwrap_or_else(|| "unknown".into()),
        slave_address: config.slave_address,
        serial_id: config
            .serial_id
            .clone()
            .unwrap_or_default()
            .trim()
            .to_string(),
        devices,
    };
    validate_assignment_plan(&plan)?;
    Ok(plan)
}

fn resolve_layout(config: &FirmwareConfig) -> Result<&'static dyn BoardLayout, BuildError> {
    let class = config.board_class.as_deref().unwrap_or("");
    let crate_path = config.firmware_crate.as_deref();
    layout_for_board(class, crate_path).ok_or_else(|| {
        BuildError::UnknownBoardClass(format!(
            "board_class={:?} firmware_crate={:?}",
            config.board_class, config.firmware_crate
        ))
    })
}

pub fn plan_config(config: &FirmwareConfig) -> Result<AssignmentPlan, BuildError> {
    let layout = resolve_layout(config)?;
    assign_devices(config, layout)
}
