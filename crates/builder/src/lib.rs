//! YAML → Rust config codegen for RP2040 firmware builds.
//!
//! Architecture-shaped `config.yaml` (per board instance):
//! ```yaml
//! board_id: rp2040_left_arm
//! board: rp2040
//! board_class: internal_servo_only
//! slave_address: 1
//! firmware_crate: firmwares/rp2040_internal_pwm
//! actuators:
//!   - id: left_shoulder_z
//!     enabled: true
//!     urdf: { joint: left_shoulder_z_link_joint }
//!     config:
//!       min_angle: 0.0          # radians
//!       max_angle: 3.14159
//!       default_angle: 1.5708
//!       min_pulse: 1250
//!       max_pulse: 2500
//!     hardware:
//!       board: rp2040
//!       driver: PwmServoDriver
//!       channel: Servo10
//! sensors:
//!   - id: left_gripper_pressure
//!     enabled: true
//!     config: { min_value: 0, max_value: 4095 }
//!     hardware:
//!       board: rp2040
//!       driver: PressureSensorDriver
//!       channel: ADC0
//! ```
//!
//! `virtual_pin` is **not** required in YAML: the builder walks enabled
//! actuators then sensors in order, resolves channels via [`BoardLayout`],
//! and assigns contiguous Modbus blocks.

use heck::ToShoutySnakeCase;
use heck::ToSnakeCase;
use lucy_embedded_firmware_core::board_layout::{
    layout_for_board, BoardLayout, HardwareIdentity,
};
use lucy_embedded_firmware_core::utils::rad_to_millirad;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

/// Modbus register block sizes by driver family.
pub const PWM_SERVO_REGS: u16 = 2;
pub const BUS_SERVO_REGS: u16 = 3;
pub const PRESSURE_SENSOR_REGS: u16 = 2;

#[derive(Debug, Deserialize)]
pub struct FirmwareConfig {
    #[serde(default = "default_slave")]
    pub slave_address: u8,
    #[serde(default)]
    pub board_id: Option<String>,
    #[serde(default)]
    pub board: Option<String>,
    #[serde(default)]
    pub board_class: Option<String>,
    #[serde(default)]
    pub firmware_crate: Option<String>,
    /// USB CDC serial string; must match host YAML ``serial_id`` when set.
    #[serde(default)]
    pub serial_id: Option<String>,
    #[serde(default)]
    pub actuators: Vec<ActuatorConfig>,
    #[serde(default)]
    pub sensors: Vec<SensorConfig>,
}

fn default_slave() -> u8 {
    1
}

#[derive(Debug, Deserialize)]
pub struct ActuatorConfig {
    pub id: String,
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Host ros2_control ``virtual_pin`` (register slot). When set, firmware
    /// keeps the same index so Modbus bases match LucySystemHardware.
    #[serde(default)]
    pub virtual_pin: Option<u16>,
    #[serde(default)]
    pub urdf: Option<UrdfRef>,
    #[serde(default)]
    pub config: BTreeMap<String, serde_yaml::Value>,
    pub hardware: HardwareRef,
}

#[derive(Debug, Deserialize)]
pub struct SensorConfig {
    pub id: String,
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub virtual_pin: Option<u16>,
    #[serde(default)]
    pub config: BTreeMap<String, serde_yaml::Value>,
    pub hardware: HardwareRef,
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Deserialize)]
pub struct UrdfRef {
    #[serde(default)]
    pub joint: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct HardwareRef {
    #[serde(default)]
    pub board: Option<String>,
    pub driver: String,
    pub channel: String,
}

/// Assigned Modbus slot for one enabled device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceAssignment {
    pub id: String,
    pub kind: DeviceKind,
    pub driver: String,
    pub channel: String,
    pub hardware: HardwareIdentity,
    pub virtual_pin: u16,
    pub base_register: u16,
    pub nb_registers: u16,
    /// Angle fields converted to milliradians; pulse/value fields as u16.
    pub fields: BTreeMap<String, u16>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceKind {
    Actuator,
    Sensor,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssignmentPlan {
    pub board_id: String,
    pub board_class: String,
    pub slave_address: u8,
    pub serial_id: String,
    pub devices: Vec<DeviceAssignment>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BuildError {
    UnknownBoardClass(String),
    UnknownChannel { id: String, channel: String },
    RegisterCollision { id: String, base: u16 },
    UnknownDriver { id: String, driver: String },
}

impl std::fmt::Display for BuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BuildError::UnknownBoardClass(c) => write!(f, "unknown board_class/firmware_crate: {c}"),
            BuildError::UnknownChannel { id, channel } => {
                write!(f, "unknown channel `{channel}` for `{id}`")
            }
            BuildError::RegisterCollision { id, base } => {
                write!(f, "register collision at base {base} for `{id}`")
            }
            BuildError::UnknownDriver { id, driver } => {
                write!(f, "unknown driver `{driver}` for `{id}`")
            }
        }
    }
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

fn config_type_for_driver(driver: &str) -> &'static str {
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

/// Generate Rust source for enabled devices and write to `$OUT_DIR/config.rs`.
pub fn build_config(config_path: impl AsRef<Path>) {
    let config_path = config_path.as_ref();
    println!("cargo:rerun-if-changed={}", config_path.display());

    let out_dir = env::var_os("OUT_DIR").expect("OUT_DIR not set");
    let filepath = PathBuf::from(out_dir).join("config.rs");

    let contents = fs::read_to_string(config_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", config_path.display()));

    let parsed: FirmwareConfig = serde_yaml::from_str(&contents)
        .unwrap_or_else(|e| panic!("failed to parse {}: {e}", config_path.display()));

    let plan = plan_config(&parsed).unwrap_or_else(|e| {
        panic!("config assignment failed for {}: {e}", config_path.display())
    });
    let code = generate_config_tokens(&plan);
    fs::write(&filepath, code.to_string())
        .unwrap_or_else(|e| panic!("failed to write {}: {e}", filepath.display()));
}

/// Write a minimal empty generated config (when YAML has no actuators yet).
pub fn write_empty_config() {
    let out_dir = env::var_os("OUT_DIR").expect("OUT_DIR not set");
    let filepath = PathBuf::from(out_dir).join("config.rs");
    let code = quote! {
        /// Auto-generated stub (no actuators: in config.yaml).
        pub const GENERATED_BOARD_ID: &str = "unset";
        pub const GENERATED_SLAVE_ADDRESS: u8 = 1;
        pub const GENERATED_USB_SERIAL_ID: &str = "";
        pub const GENERATED_DEVICE_COUNT: usize = 0;
        #[derive(Clone, Copy)]
        pub struct GeneratedPwmDevice {
            pub gpio: u8,
            pub base_register: u16,
            pub config: lucy_embedded_firmware_core::drivers::PwmServoConfig,
        }
        pub const GENERATED_PWM_DEVICE_COUNT: usize = 0;
        pub const GENERATED_PWM_DEVICES: &[GeneratedPwmDevice] = &[];
        pub fn init_generated_configs() {}
    };
    fs::write(&filepath, code.to_string())
        .unwrap_or_else(|e| panic!("failed to write {}: {e}", filepath.display()));
}

pub fn plan_config(config: &FirmwareConfig) -> Result<AssignmentPlan, BuildError> {
    let layout = resolve_layout(config)?;
    assign_devices(config, layout)
}

fn hw_identity_tokens(hw: &HardwareIdentity) -> TokenStream {
    match hw {
        HardwareIdentity::PwmGpio { servo_index, gpio } => quote! {
            lucy_embedded_firmware_core::board_layout::HardwareIdentity::PwmGpio {
                servo_index: #servo_index,
                gpio: #gpio,
            }
        },
        HardwareIdentity::Adc { channel } => quote! {
            lucy_embedded_firmware_core::board_layout::HardwareIdentity::Adc {
                channel: #channel,
            }
        },
        HardwareIdentity::UartBus { uart, device_id } => {
            let dev = match device_id {
                Some(d) => quote! { Some(#d) },
                None => quote! { None },
            };
            quote! {
                lucy_embedded_firmware_core::board_layout::HardwareIdentity::UartBus {
                    uart: #uart,
                    device_id: #dev,
                }
            }
        }
        HardwareIdentity::I2cPwm {
            bus,
            device,
            channel,
        } => quote! {
            lucy_embedded_firmware_core::board_layout::HardwareIdentity::I2cPwm {
                bus: #bus,
                device: #device,
                channel: #channel,
            }
        },
    }
}

pub fn generate_config_tokens(plan: &AssignmentPlan) -> TokenStream {
    let slave = plan.slave_address;
    let board = plan.board_id.as_str();
    let board_class = plan.board_class.as_str();
    let serial_id = plan.serial_id.as_str();
    let device_count = plan.devices.len();

    let mut const_blocks = Vec::new();
    let mut init_lets = Vec::new();

    for dev in &plan.devices {
        let snake = dev.id.to_snake_case();
        let shouty = dev.id.to_shouty_snake_case();
        let pin_const = format_ident!("{}_VIRTUAL_PIN", shouty);
        let base_const = format_ident!("{}_BASE_REGISTER", shouty);
        let nb_const = format_ident!("{}_NB_REGISTERS", shouty);
        let channel_const = format_ident!("{}_CHANNEL", shouty);
        let config_const = format_ident!("{}_CONFIG", shouty);
        let hw_fn = format_ident!("{}_hardware", snake);
        let config_type = format_ident!("{}", config_type_for_driver(&dev.driver));
        let vp = dev.virtual_pin;
        let base = dev.base_register;
        let nb = dev.nb_registers;
        let channel = dev.channel.as_str();
        let hw_tok = hw_identity_tokens(&dev.hardware);

        let field_tokens: Vec<TokenStream> = dev
            .fields
            .iter()
            .map(|(k, v)| {
                let key = format_ident!("{}", k);
                quote! { #key: #v }
            })
            .collect();

        const_blocks.push(quote! {
            #[allow(dead_code)]
            pub const #pin_const: u16 = #vp;
            #[allow(dead_code)]
            pub const #base_const: u16 = #base;
            #[allow(dead_code)]
            pub const #nb_const: u16 = #nb;
            #[allow(dead_code)]
            pub const #channel_const: &str = #channel;
            #[allow(dead_code)]
            pub const #config_const: lucy_embedded_firmware_core::drivers::#config_type =
                lucy_embedded_firmware_core::drivers::#config_type {
                    #(#field_tokens),*
                };
            #[allow(dead_code)]
            pub const fn #hw_fn() -> lucy_embedded_firmware_core::board_layout::HardwareIdentity {
                #hw_tok
            }
        });

        let config_var = format_ident!("{}_config", snake);
        init_lets.push(quote! {
            let #config_var = #config_const;
            let _ = (#base_const, #pin_const, #config_var);
        });
    }

    let mut pwm_entries = Vec::new();
    for dev in &plan.devices {
        if let HardwareIdentity::PwmGpio { gpio, .. } = &dev.hardware {
            let shouty = dev.id.to_shouty_snake_case();
            let config_const = format_ident!("{}_CONFIG", shouty);
            let base = dev.base_register;
            pwm_entries.push(quote! {
                GeneratedPwmDevice {
                    gpio: #gpio,
                    base_register: #base,
                    config: #config_const,
                }
            });
        }
    }
    let pwm_count = pwm_entries.len();

    quote! {
        /// Auto-generated by `builder::build_config`. Do not edit.
        pub const GENERATED_BOARD_ID: &str = #board;
        pub const GENERATED_BOARD_CLASS: &str = #board_class;
        pub const GENERATED_SLAVE_ADDRESS: u8 = #slave;
        /// USB CDC serial; must match host ``serial_id`` (flash unique id hex).
        pub const GENERATED_USB_SERIAL_ID: &str = #serial_id;
        pub const GENERATED_DEVICE_COUNT: usize = #device_count;

        #(#const_blocks)*

        /// One enabled on-board PWM servo (Servo2040 silk = gpio+1).
        #[derive(Clone, Copy)]
        pub struct GeneratedPwmDevice {
            pub gpio: u8,
            pub base_register: u16,
            pub config: lucy_embedded_firmware_core::drivers::PwmServoConfig,
        }

        pub const GENERATED_PWM_DEVICE_COUNT: usize = #pwm_count;
        pub const GENERATED_PWM_DEVICES: &[GeneratedPwmDevice] = &[
            #(#pwm_entries),*
        ];

        #[allow(unused_variables, unused_mut)]
        pub fn init_generated_configs() {
            #(#init_lets)*
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lucy_embedded_firmware_core::board_layout::Rp2040InternalPwmLayout;

    fn sample_yaml() -> &'static str {
        r#"
board_id: rp2040_left_arm
board: rp2040
board_class: internal_servo_only
slave_address: 1
firmware_crate: firmwares/rp2040_internal_pwm
actuators:
  - id: left_shoulder_z
    enabled: true
    urdf: { joint: left_shoulder_z_link_joint }
    config:
      min_angle: 0.0
      max_angle: 3.1415926535
      default_angle: 1.57079632679
      min_pulse: 1250
      max_pulse: 2500
    hardware:
      board: rp2040
      driver: PwmServoDriver
      channel: Servo10
  - id: disabled_joint
    enabled: false
    config:
      min_angle: 0.0
      max_angle: 1.0
      default_angle: 0.0
      min_pulse: 1250
      max_pulse: 2500
    hardware:
      board: rp2040
      driver: PwmServoDriver
      channel: Servo1
sensors:
  - id: left_gripper_pressure
    enabled: true
    config:
      min_value: 0
      max_value: 4095
    hardware:
      board: rp2040
      driver: PressureSensorDriver
      channel: ADC0
"#
    }

    #[test]
    fn pin_table_servo10_is_gpio9() {
        let layout = Rp2040InternalPwmLayout;
        let hw = layout.resolve("Servo10").unwrap();
        assert_eq!(
            hw,
            HardwareIdentity::PwmGpio {
                servo_index: 10,
                gpio: 9
            }
        );
    }

    #[test]
    fn enabled_only_pwm_devices_in_codegen() {
        let cfg: FirmwareConfig = serde_yaml::from_str(sample_yaml()).unwrap();
        let plan = plan_config(&cfg).unwrap();
        let src = generate_config_tokens(&plan).to_string();
        assert!(src.contains("GENERATED_PWM_DEVICE_COUNT : usize = 1usize"));
        assert!(src.contains("gpio : 9u8"));
        // Disabled Servo1 must not appear in the PWM device table.
        assert!(!src.contains("gpio : 0u8"));
        assert!(src.contains("Servo10"));
    }

    #[test]
    fn servo18_maps_to_gpio17() {
        let yaml = r#"
board_class: internal_servo_only
actuators:
  - id: tip
    enabled: true
    config: { min_angle: 0.0, max_angle: 1.0, default_angle: 0.0, min_pulse: 1, max_pulse: 2 }
    hardware: { driver: PwmServoDriver, channel: Servo18 }
"#;
        let cfg: FirmwareConfig = serde_yaml::from_str(yaml).unwrap();
        let plan = plan_config(&cfg).unwrap();
        assert_eq!(
            plan.devices[0].hardware,
            HardwareIdentity::PwmGpio {
                servo_index: 18,
                gpio: 17
            }
        );
    }

    #[test]
    fn virtual_pin_stability_and_blocks() {
        let cfg: FirmwareConfig = serde_yaml::from_str(sample_yaml()).unwrap();
        let plan = plan_config(&cfg).unwrap();
        assert_eq!(plan.devices.len(), 2);

        let act = &plan.devices[0];
        assert_eq!(act.id, "left_shoulder_z");
        assert_eq!(act.virtual_pin, 0);
        assert_eq!(act.base_register, 0);
        assert_eq!(act.nb_registers, PWM_SERVO_REGS);
        assert_eq!(act.channel, "Servo10");

        let sens = &plan.devices[1];
        assert_eq!(sens.id, "left_gripper_pressure");
        assert_eq!(sens.virtual_pin, 1);
        assert_eq!(sens.base_register, PWM_SERVO_REGS);
        assert_eq!(sens.nb_registers, PRESSURE_SENSOR_REGS);
    }

    #[test]
    fn rad_to_millirad_in_fields() {
        let cfg: FirmwareConfig = serde_yaml::from_str(sample_yaml()).unwrap();
        let plan = plan_config(&cfg).unwrap();
        let act = &plan.devices[0];
        // π/2 ≈ 1571
        assert!((act.fields["default_angle"] as i32 - 1571).abs() <= 1);
        // π ≈ 3142
        assert!((act.fields["max_angle"] as i32 - 3142).abs() <= 1);
        assert_eq!(act.fields["min_pulse"], 1250);
    }

    #[test]
    fn unknown_channel_fails() {
        let yaml = r#"
board_class: internal_servo_only
actuators:
  - id: bad
    enabled: true
    config: { min_angle: 0.0, max_angle: 1.0, default_angle: 0.0, min_pulse: 1, max_pulse: 2 }
    hardware: { driver: PwmServoDriver, channel: Servo99 }
"#;
        let cfg: FirmwareConfig = serde_yaml::from_str(yaml).unwrap();
        let err = plan_config(&cfg).unwrap_err();
        assert!(matches!(err, BuildError::UnknownChannel { .. }));
    }

    #[test]
    fn sensor_block_sizing() {
        let yaml = r#"
board_class: internal_servo_only
sensors:
  - id: p0
    enabled: true
    config: { min_value: 0, max_value: 100 }
    hardware: { driver: PressureSensorDriver, channel: ADC1 }
"#;
        let cfg: FirmwareConfig = serde_yaml::from_str(yaml).unwrap();
        let plan = plan_config(&cfg).unwrap();
        assert_eq!(plan.devices[0].nb_registers, PRESSURE_SENSOR_REGS);
        assert_eq!(plan.devices[0].base_register, 0);
        assert_eq!(plan.devices[0].fields["max_value"], 100);
    }

    #[test]
    fn register_collision_on_duplicate_channel() {
        let yaml = r#"
board_class: internal_servo_only
actuators:
  - id: a
    enabled: true
    config: { min_angle: 0.0, max_angle: 1.0, default_angle: 0.0, min_pulse: 1, max_pulse: 2 }
    hardware: { driver: PwmServoDriver, channel: Servo1 }
  - id: b
    enabled: true
    config: { min_angle: 0.0, max_angle: 1.0, default_angle: 0.0, min_pulse: 1, max_pulse: 2 }
    hardware: { driver: PwmServoDriver, channel: Servo1 }
"#;
        let cfg: FirmwareConfig = serde_yaml::from_str(yaml).unwrap();
        let err = plan_config(&cfg).unwrap_err();
        assert!(matches!(err, BuildError::RegisterCollision { .. }));
    }

    #[test]
    fn contiguous_blocks_no_overlap() {
        let yaml = r#"
board_class: internal_servo_only
actuators:
  - id: a
    enabled: true
    config: { min_angle: 0.0, max_angle: 1.0, default_angle: 0.0, min_pulse: 1, max_pulse: 2 }
    hardware: { driver: PwmServoDriver, channel: Servo1 }
  - id: b
    enabled: true
    config: { min_angle: 0.0, max_angle: 1.0, default_angle: 0.0, min_pulse: 1, max_pulse: 2 }
    hardware: { driver: BusServoDriver, channel: UART0:1 }
"#;
        let cfg: FirmwareConfig = serde_yaml::from_str(yaml).unwrap();
        let plan = plan_config(&cfg).unwrap();
        let pwm = plan.devices.iter().find(|d| d.id == "a").unwrap();
        let bus = plan.devices.iter().find(|d| d.id == "b").unwrap();
        assert_eq!(pwm.virtual_pin, 0);
        assert_eq!(pwm.nb_registers, 2);
        assert_eq!(pwm.base_register, 0);
        assert_eq!(bus.virtual_pin, 1);
        assert_eq!(bus.nb_registers, 3);
        // base = virtual_pin * driver block size (matches host LucySystemHardware).
        assert_eq!(bus.base_register, 3);
        let occupied: BTreeSet<u16> = plan
            .devices
            .iter()
            .flat_map(|d| d.base_register..d.base_register + d.nb_registers)
            .collect();
        assert_eq!(occupied.len(), 5);
        assert!(!occupied.contains(&2)); // hole between PWM block and bus block
    }

    #[test]
    fn generates_tokens_with_consts() {
        let cfg: FirmwareConfig = serde_yaml::from_str(sample_yaml()).unwrap();
        let plan = plan_config(&cfg).unwrap();
        let tokens = generate_config_tokens(&plan).to_string();
        assert!(tokens.contains("GENERATED_BOARD_ID"));
        assert!(tokens.contains("LEFT_SHOULDER_Z_VIRTUAL_PIN"));
        assert!(tokens.contains("LEFT_GRIPPER_PRESSURE_BASE_REGISTER"));
        assert!(tokens.contains("PwmServoConfig"));
        assert!(tokens.contains("PressureSensorConfig"));
        assert!(!tokens.contains("disabled_joint"));
    }

    #[test]
    fn disabled_devices_skipped() {
        let cfg: FirmwareConfig = serde_yaml::from_str(sample_yaml()).unwrap();
        let plan = plan_config(&cfg).unwrap();
        assert!(!plan.devices.iter().any(|d| d.id == "disabled_joint"));
    }
}
