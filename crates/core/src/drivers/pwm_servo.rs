use crate::{pwm::PwmChannel};
use crate::{utils::map_range};
use crate::actuator::{*};
use crate::joint::{JointCommand, JointState, JointConfig};
use core::f32::consts::PI;

pub enum PwmServoError {
    CommunicationError
}

pub struct PwmServoConfig {
    pub amplitude: f64,
    pub min_pulse: u16,
    pub max_pulse: u16,
}

pub struct PwmServoDriver<'a, A: PwmChannel> {
    pub channel: &'a mut A,
    pub actuator_config: &'a PwmServoConfig,
    pub joint_command: &'a JointCommand,
    pub joint_state: &'a JointState,
}

impl<'a, A: PwmChannel> Actuator for PwmServoDriver<'a, A> {
    type Error = PwmServoError;

    fn enable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn disable(&mut self) {
    }

    fn apply(&mut self) -> Result<(), Self::Error> {
        let pulse = (map_range(
            self.joint_command.position,
            0f64,
            self.actuator_config.amplitude,
            self.actuator_config.min_pulse as f64,
            self.actuator_config.max_pulse as f64,
        ) + 0.5) as u16;

        self.channel
            .set_pwm(pulse)
            .map_err(|_| PwmServoError::CommunicationError)?;
        Ok(())
    }
}

/*impl<'cfg, 'bus, C: PwmChannel> TorqueEnableInterface for PwmServoDriver<'cfg, 'bus, C> {
    type Error = PwmServoError;

    fn set_torque_enable(&mut self, status: TorqueStatus) -> Result<(), Self::Error> {
        if status == TorqueStatus::Disabled {
            self.channel
                .set_pwm(0)
                .map_err(|_| PwmServoError::CommunicationError)?;
        }

        Ok(())
    }
}*/

