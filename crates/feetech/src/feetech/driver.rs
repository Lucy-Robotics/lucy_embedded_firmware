use lucy_embedded_firmware_serial::SerialChannel;

use lucy_embedded_firmware_core::actuator::ActuatorGroup;
use lucy_embedded_firmware_core::joint::*;

use core::f32::consts::TAU;
use core::marker::PhantomData;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeetechServoConfig {
    pub id: u8,

    pub amplitude: f32,
    pub min_pulse: u16,
    pub max_pulse: u16,

    pub torque: f32,
    pub gear_ratio: f32,
    pub stall_current: f32,
    pub no_load_current: f32,
    pub current_lsb: f32,
}

impl Default for FeetechServoConfig {
    fn default() -> Self {
        FeetechServoConfig {
            id: 1,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4095,

            torque: 2.94,
            gear_ratio: 1f32,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeetechBusError {
    OutOfLimits,
    CommunicationError,
    /// An EEPROM register did not read back the value just written to it.
    ReadbackMismatch,
    /// This servo's position has no half-turn offset that fits `Homing_Offset`.
    OffsetOutOfRange(u8),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeetechBusConfig<const N: usize> {
    pub servos: [FeetechServoConfig; N],
}

pub struct FeetechBusDriver<S: SerialChannel, const N: usize> {
    _state: PhantomData<fn() -> S>,
    pub config: &'static FeetechBusConfig<N>,
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    pub fn new(config: &'static FeetechBusConfig<N>) -> Self {
        Self {
            _state: PhantomData,
            config,
        }
    }
}

impl<S: SerialChannel, const N: usize> ActuatorGroup for FeetechBusDriver<S, N> {
    type Error = FeetechBusError;
    type Bus = S;

    fn update(
        &mut self,
        tick: u64,
        bus: &mut Self::Bus,
        command: &mut [Command],
        state: &mut [State],
    ) -> Result<(), Self::Error> {
        let mut seen = [false; N];
        self.update_tracked(tick, bus, command, state, &mut seen)
    }
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    /// [`ActuatorGroup::update`] that also reports which servos answered the position read.
    pub fn update_tracked(
        &mut self,
        tick: u64,
        bus: &mut S,
        command: &mut [Command],
        state: &mut [State],
        seen: &mut [bool; N],
    ) -> Result<(), FeetechBusError> {
        let torque = self.set_torque(bus, command);
        let position = self.set_position(bus, command);
        let feedback = self.read_position_tracked(bus, state, seen);
        let extra = match (tick % 2, tick % 10) {
            (1, _) => self.read_effort(bus, state),
            (_, 0) => self.read_temperature(bus, state),
            _ => Ok(()),
        };
        torque.and(position).and(feedback).and(extra)
    }
}
