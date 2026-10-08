use crate::pwm::PwmChannel;
use crate::{
    modbus::{ModbusAdapter, RegisterView},
    utils::{clamp_pulse, rad_to_pulse},
};

/// PWM hobby-servo configuration.
///
/// Angle fields are **radians** (from YAML). The Modbus angle register carries
/// a **pulse** (duty count); host HI converts rad → pulse before SHM.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub struct PwmServoConfig {
    pub min_pulse: u16,
    pub max_pulse: u16,
    /// Minimum angle in radians.
    pub min_angle: f32,
    /// Maximum angle in radians.
    pub max_angle: f32,
    /// Default angle in radians.
    pub default_angle: f32,
}

/// Historical aliases (amplitude is carried in config angles, not the type).
pub type PwmServo180Driver<C> = PwmServoDriver<C>;
pub type PwmServo270Driver<C> = PwmServoDriver<C>;
pub type PwmServo300Driver<C> = PwmServoDriver<C>;

pub struct PwmServoDriver<C> {
    pub config: PwmServoConfig,
    pub channel: C,
}

impl<C: PwmChannel> PwmServoDriver<C> {
    /// Apply a wire **pulse** (already rad→pulse on the host).
    pub fn apply_pulse(&mut self, pulse: u16) {
        let pulse = clamp_pulse(pulse, self.config.min_pulse, self.config.max_pulse);
        let _ = self.channel.set_pwm(pulse);
    }

    /// Local rad → pulse (defaults / tests). Runtime Modbus path uses [`Self::apply_pulse`].
    pub fn move_angle(&mut self, angle_rad: f32) {
        let pulse = rad_to_pulse(
            angle_rad,
            self.config.min_angle,
            self.config.max_angle,
            self.config.min_pulse,
            self.config.max_pulse,
        );
        self.apply_pulse(pulse);
    }

    pub fn reset_angle(&mut self) {
        self.move_angle(self.config.default_angle);
    }
}

pub struct PwmServoModbusAdapter<C> {
    pub base_register: u16,
    pub cmd_reg_off: u16,
    pub angle_reg_off: u16,
    pub driver: PwmServoDriver<C>,
}

impl<C: PwmChannel> ModbusAdapter for PwmServoModbusAdapter<C> {
    fn tick(&mut self, rv: &RegisterView) {
        let cmd = rv.read_register(self.cmd_reg_off);
        rv.write_register(self.cmd_reg_off, 0);
        match cmd {
            1 => {
                let pulse = rv.read_register(self.angle_reg_off);
                self.driver.apply_pulse(pulse);
            }
            2 => {
                self.driver.reset_angle();
            }
            _ => {}
        }
    }

    fn get_nb_register(&self) -> u16 {
        2
    }

    fn get_base_register(&self) -> u16 {
        self.base_register
    }
}
