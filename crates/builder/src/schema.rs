//! YAML schema types for architecture-shaped firmware config.

use lucy_embedded_firmware_core::board_layout::HardwareIdentity;
use serde::Deserialize;
use std::collections::BTreeMap;

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
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceAssignment {
    pub id: String,
    pub kind: DeviceKind,
    pub driver: String,
    pub channel: String,
    pub hardware: HardwareIdentity,
    pub virtual_pin: u16,
    pub base_register: u16,
    pub nb_registers: u16,
    /// Pulse / sensor value fields as u16; angle fields as radians (f32).
    pub fields: BTreeMap<String, ConfigValue>,
}

/// One codegen field: integer pulse/value or floating-point radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConfigValue {
    U16(u16),
    F32(f32),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceKind {
    Actuator,
    Sensor,
}

#[derive(Debug, Clone, PartialEq)]
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
    /// Servo2040 UART0 owns GPIO0–2; incompatible with PWM on those pads / non-UART0 bus.
    PinmuxConflict { id: String, detail: String },
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
            BuildError::PinmuxConflict { id, detail } => {
                write!(f, "pinmux conflict for `{id}`: {detail}")
            }
        }
    }
}
