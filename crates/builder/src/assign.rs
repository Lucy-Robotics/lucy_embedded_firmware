//! Virtual-pin / register assignment from parsed firmware YAML.

use crate::schema::{
    AssignmentPlan, BuildError, DeviceAssignment, DeviceKind, FirmwareConfig,
    BUS_SERVO_REGS, PRESSURE_SENSOR_REGS, PWM_SERVO_REGS,
};
use lucy_embedded_firmware_core::board_layout::{layout_for_board, BoardLayout};
use lucy_embedded_firmware_core::utils::rad_to_millirad;
use std::collections::{BTreeMap, BTreeSet};

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
                Some(i as u16)
            } else if let Some(f) = n.as_f64() {
                Some((f + 0.5) as u16)
            } else {
                None
            }
        }
        serde_yaml::Value::String(s) => s.parse().ok(),
        _ => None,
    }
}

/// Convert actuator/sensor `config` map: angle keys → millirad; others → u16.
pub fn normalize_config_fields(
    kind: DeviceKind,
    raw: &BTreeMap<String, serde_yaml::Value>,
) -> BTreeMap<String, u16> {
    let mut out = BTreeMap::new();
    for (k, v) in raw {
        let is_angle = matches!(
            k.as_str(),
            "min_angle" | "max_angle" | "default_angle"
        );
        if is_angle {
            if let Some(rad) = yaml_f64(v) {
                out.insert(k.clone(), rad_to_millirad(rad as f32));
            }
        } else if let Some(u) = yaml_u16(v) {
            out.insert(k.clone(), u);
        }
    }
    // Sensors default min/max if omitted
    if kind == DeviceKind::Sensor {
        out.entry("min_value".into()).or_insert(0);
        out.entry("max_value".into()).or_insert(4095);
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

    Ok(AssignmentPlan {
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
    })
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
