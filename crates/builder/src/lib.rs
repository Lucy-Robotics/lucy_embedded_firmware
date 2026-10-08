//! YAML → Rust config codegen for RP2040 firmware builds.
//!
//! Architecture-shaped `config.yaml` (per board instance). `virtual_pin` is
//! **not** required in YAML: the builder walks enabled actuators then sensors
//! in order, resolves channels via [`BoardLayout`], and assigns contiguous
//! Modbus blocks.
//!
//! See module docs in [`schema`], [`assign`], and [`codegen`].

pub mod board_layouts;
mod schema;
mod assign;
mod codegen;

pub use schema::{
    ActuatorConfig, AssignmentPlan, BuildError, ConfigValue, DeviceAssignment, DeviceKind,
    FirmwareConfig, HardwareRef, SensorConfig, UrdfRef, BUS_SERVO_REGS, PRESSURE_SENSOR_REGS,
    PWM_SERVO_REGS,
};
pub use assign::{assign_devices, normalize_config_fields, plan_config};
pub use codegen::{build_config, generate_config_tokens, write_empty_config};
