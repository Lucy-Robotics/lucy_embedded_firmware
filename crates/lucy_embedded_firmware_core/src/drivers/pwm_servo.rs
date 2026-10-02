use crate::pwm::PwmChannel;
use crate::{
    modbus::{ModbusAdapter, RegisterView},
    utils::millirad_to_pulse,
};

/// PWM hobby-servo configuration.
///
/// Angle fields are **milliradians** (`rad × 1000`) after codegen from radian YAML.
/// Mechanical range comes from YAML `min_angle`/`max_angle` (host `servo_type`
/// is `180` | `270` | `300` only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub struct PwmServoConfig {
    pub min_pulse: u16,
    pub max_pulse: u16,
    /// Minimum angle in milliradians.
    pub min_angle: u16,
    /// Maximum angle in milliradians.
    pub max_angle: u16,
    /// Default angle in milliradians.
    pub default_angle: u16,
}

/// Servo class tagged by mechanical amplitude in milliradians.
/// Matches host `servo_type`: 180° / 270° / 300° (π ≈ 3142 for 180°).
pub type PwmServo180Driver<C> = PwmServoDriver<3142, C>;
pub type PwmServo270Driver<C> = PwmServoDriver<4712, C>;
/// 300° ≈ 300 × π/180 ≈ 5.236 rad → 5236 millirad.
pub type PwmServo300Driver<C> = PwmServoDriver<5236, C>;

pub struct PwmServoDriver<const A: u16, C> {
    pub config: PwmServoConfig,
    pub channel: C,
}

impl<const A: u16, C: PwmChannel> PwmServoDriver<A, C> {
    /// Mechanical amplitude of this servo class in milliradians.
    pub const AMPLITUDE_MILLIRAD: u16 = A;

    /// `angle_millirad` is radians × 1000 (matches LucySystemHardware SHM encoding).
    pub fn move_angle(&mut self, angle_millirad: u16) {
        let pulse = millirad_to_pulse(
            angle_millirad,
            self.config.min_angle,
            self.config.max_angle,
            self.config.min_pulse,
            self.config.max_pulse,
        );
        let _ = self.channel.set_pwm(pulse);
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
