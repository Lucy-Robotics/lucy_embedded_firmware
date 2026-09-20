//! YAML → Rust config codegen for RP2040 firmware builds.
//!
//! Architecture-shaped `config.yaml` (per board instance). `virtual_pin` is
//! **not** required in YAML: the builder walks enabled actuators then sensors
//! in order, resolves channels via [`BoardLayout`], and assigns contiguous
//! Modbus blocks.
//!
//! See module docs in [`schema`], [`assign`], and [`codegen`].

mod schema;
mod assign;
mod codegen;

pub use schema::{
    ActuatorConfig, AssignmentPlan, BuildError, DeviceAssignment, DeviceKind, FirmwareConfig,
    HardwareRef, SensorConfig, UrdfRef, BUS_SERVO_REGS, PRESSURE_SENSOR_REGS, PWM_SERVO_REGS,
};
pub use assign::{assign_devices, normalize_config_fields, plan_config};
pub use codegen::{build_config, generate_config_tokens, write_empty_config};

#[cfg(test)]
mod tests {
    use super::*;
    use lucy_embedded_firmware_core::board_layout::{
        BoardLayout, HardwareIdentity, Rp2040Servo2040Layout,
    };
    use std::collections::BTreeSet;

    fn sample_yaml() -> &'static str {
        r#"
board_id: rp2040_left_arm
board: rp2040
board_class: internal_servo_only
slave_address: 1
firmware_crate: firmwares/rp2040_servo2040
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
        let layout = Rp2040Servo2040Layout;
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

    #[test]
    fn pressure_sensor_yaml_codegens_placeholder_config() {
        let yaml = r#"
board_class: internal_servo_only
firmware_crate: firmwares/rp2040_servo2040
actuators: []
sensors:
  - id: grip_pressure
    enabled: true
    config: { min_value: 0, max_value: 4095 }
    hardware: { driver: PressureSensorDriver, channel: ADC0 }
"#;
        let cfg: FirmwareConfig = serde_yaml::from_str(yaml).unwrap();
        let plan = plan_config(&cfg).unwrap();
        assert_eq!(plan.devices.len(), 1);
        assert_eq!(plan.devices[0].nb_registers, PRESSURE_SENSOR_REGS);
        let src = generate_config_tokens(&plan).to_string();
        assert!(src.contains("PressureSensorConfig"));
        assert!(src.contains("GRIP_PRESSURE_BASE_REGISTER"));
        assert!(src.contains("GENERATED_PWM_DEVICE_COUNT : usize = 0usize"));
    }

    #[test]
    fn bus_servo_yaml_codegens_bus_device_table() {
        let yaml = r#"
board_class: bus_servo_only
firmware_crate: firmwares/rp2040_bus_servo
actuators:
  - id: rotation
    enabled: true
    virtual_pin: 0
    config:
      min_angle: 1.0
      max_angle: 5.0
      default_angle: 3.14
      min_pulse: 0
      max_pulse: 4095
    hardware: { driver: BusServoDriver, channel: UART0:1 }
  - id: pitch
    enabled: true
    virtual_pin: 1
    config:
      min_angle: 1.0
      max_angle: 5.0
      default_angle: 3.14
      min_pulse: 0
      max_pulse: 4095
    hardware: { driver: BusServoDriver, channel: UART0:2 }
"#;
        let cfg: FirmwareConfig = serde_yaml::from_str(yaml).unwrap();
        let plan = plan_config(&cfg).unwrap();
        assert_eq!(plan.devices.len(), 2);
        let src = generate_config_tokens(&plan).to_string();
        assert!(src.contains("GENERATED_BUS_DEVICE_COUNT : usize = 2usize"));
        assert!(src.contains("device_id : 1u8"));
        assert!(src.contains("device_id : 2u8"));
        assert!(src.contains("BusServoConfig"));
        assert!(src.contains("GENERATED_PWM_DEVICE_COUNT : usize = 0usize"));
    }
}
