use crate::pwm_channel::PwmChannel;
use lucy_embedded_firmware_core::joint::{Command, State, CommandConfig};
use lucy_embedded_firmware_core::actuator::{ActuatorGroup};
use lucy_embedded_firmware_core::utils::map_range;
use core::f32::consts::PI;
use core::marker::PhantomData;

pub enum PwmServoError {
    CommunicationError
}

pub struct PwmServoConfig {
    pub amplitude: f64,
    pub min_pulse: u16,
    pub max_pulse: u16,
}

impl Default for PwmServoConfig {
    fn default() -> Self {
        PwmServoConfig {
            amplitude: PI as f64,
            min_pulse: 544,
            max_pulse: 2400,
        }
    }
}

pub struct PwmServoDriver<C: PwmChannel> {
    _state: PhantomData<fn() -> C>,
    pub config: &'static PwmServoConfig,
}

impl<C: PwmChannel> PwmServoDriver<C> {
    pub fn new(config: &'static PwmServoConfig) -> Self {
        Self {
            _state: PhantomData,
            config,
        }
    }
}

impl<C: PwmChannel> ActuatorGroup for PwmServoDriver<C> {
    type Error = PwmServoError;
    type Bus = C;

    fn update(&mut self, tick: u64, bus: &mut Self::Bus, command: &mut [Command], state: &mut [State]) -> Result<(), Self::Error> {

        let pulse = (map_range(
            command[0].position,
            0f64,
            self.config.amplitude,
            self.config.min_pulse as f64,
            self.config.max_pulse as f64,
        ) + 0.5) as u16;

        bus.set_pwm(pulse)
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

