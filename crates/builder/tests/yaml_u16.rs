use builder::{normalize_config_fields, ConfigValue, DeviceKind};
use serde_yaml::Value;
use std::collections::BTreeMap;

fn raw(map: &[(&str, Value)]) -> BTreeMap<String, Value> {
    map.iter().map(|(k, v)| ((*k).to_string(), v.clone())).collect()
}

#[test]
fn rejects_out_of_range_integers() {
    let fields = normalize_config_fields(
        DeviceKind::Actuator,
        &raw(&[("min_pulse", Value::from(70_000u64))]),
    );
    assert!(!fields.contains_key("min_pulse"));

    let fields = normalize_config_fields(
        DeviceKind::Actuator,
        &raw(&[("min_pulse", Value::from(-1i64))]),
    );
    assert!(!fields.contains_key("min_pulse"));

    let fields = normalize_config_fields(
        DeviceKind::Actuator,
        &raw(&[("min_pulse", Value::from(4095u64))]),
    );
    assert_eq!(fields.get("min_pulse"), Some(&ConfigValue::U16(4095)));
}

#[test]
fn rejects_out_of_range_floats() {
    let fields = normalize_config_fields(
        DeviceKind::Actuator,
        &raw(&[("min_pulse", Value::from(1.0e9_f64))]),
    );
    assert!(!fields.contains_key("min_pulse"));

    let fields = normalize_config_fields(
        DeviceKind::Actuator,
        &raw(&[("min_pulse", Value::from(-0.5_f64))]),
    );
    assert!(!fields.contains_key("min_pulse"));

    let fields = normalize_config_fields(
        DeviceKind::Actuator,
        &raw(&[("min_pulse", Value::from(2000.4_f64))]),
    );
    assert_eq!(fields.get("min_pulse"), Some(&ConfigValue::U16(2000)));
}
