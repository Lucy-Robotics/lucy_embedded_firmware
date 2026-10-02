use crate::{pwm::PwmChannel};
use crate::{modbus::RegisterView, modbus::ModbusAdapter, utils::map_range};
use core::f32::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub struct PwmServoConfig {
    pub min_pulse: u16,
    pub max_pulse: u16,
    pub min_angle: u16,
    pub max_angle: u16,
    pub default_angle: u16
}

type PwmServo90Driver<C> = PwmServoDriver<90, C>;
type PwmServo180Driver<C> = PwmServoDriver<180, C>;
type PwmServo270Driver<C> = PwmServoDriver<270, C>;
type PwmServo360Driver<C> = PwmServoDriver<360, C>;

pub struct PwmServoDriver<const A: u16, C> {
    pub config: PwmServoConfig,
    pub channel: C,
}

impl<const A: u16, C: PwmChannel> PwmServoDriver<A, C> {
    pub const AMPLITUDE: u16 = A;

    pub fn move_angle(&mut self, angle_rad: u16) {
        let angle_deg = (angle_rad as f32 / 1000.0).to_degrees();
        let clamped_deg = angle_deg.clamp(self.config.min_angle as f32, self.config.max_angle as f32);
        let pulse = (map_range(
            clamped_deg,
            0f32,
            Self::AMPLITUDE as f32,
            self.config.min_pulse as f32,
            self.config.max_pulse as f32,
        ) + 0.5) as u16;

        self.channel.set_pwm(pulse);
    }

    pub fn reset_angle(&mut self) {
        self.move_angle(self.config.default_angle);
    }
}



pub struct PwmServoModbusAdapter<const A: u16, C> {
    pub base_register: u16,
    pub cmd_reg_off: u16,
    pub angle_reg_off: u16,
    pub driver: PwmServoDriver<A, C>,
}

impl<const A: u16, C: PwmChannel> ModbusAdapter for PwmServoModbusAdapter<A, C> {
    fn tick(&mut self, rv: &RegisterView) {
        let cmd = rv.read_register(self.cmd_reg_off);
        rv.write_register(self.cmd_reg_off, 0);
        match cmd {
            1 => {
                let angle = rv.read_register(self.angle_reg_off);
                self.driver.move_angle(angle);
            },
            2 => {
                self.driver.reset_angle();
            },
            _ => {

            }
        }
    }

    fn get_nb_register(&self) -> u16 {
        2
    }

    fn get_base_register(&self) -> u16 {
        self.base_register
    }
}
