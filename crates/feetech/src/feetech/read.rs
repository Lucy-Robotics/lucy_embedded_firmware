use super::driver::{FeetechBusDriver, FeetechBusError};
use super::protocol::{RequestBuilder, SYNC_READ, read_replies, read_replies_tracked};
use lucy_embedded_firmware_serial::SerialChannel;

use lucy_embedded_firmware_core::joint::*;
use lucy_embedded_firmware_core::utils::map_range;

/// Shared with `homing`: `read_position_tracked` and `read_present_ticks` both read it.
pub(super) const REG_PRESENT_POSITION: u8 = 0x38;

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    pub fn read_temperature(
        &mut self,
        bus: &mut S,
        state: &mut [State],
    ) -> Result<(), FeetechBusError> {
        const REG_TEMP: u8 = 0x3F;

        if state.len() != N {
            return Err(FeetechBusError::OutOfLimits);
        }

        let mut ids = [0u8; 64];
        if N > ids.len() {
            return Err(FeetechBusError::OutOfLimits);
        }
        for (slot, cfg) in ids.iter_mut().zip(self.config.servos.iter()) {
            *slot = cfg.id;
        }
        let ids = &ids[..N];

        let (frame, size) = RequestBuilder::<64>::new()
            .set_id(0xFE)
            .set_instruction(SYNC_READ)
            .set_register(REG_TEMP)
            .set_sync_length(1)
            .build(ids)
            .map_err(|_| FeetechBusError::OutOfLimits)?;

        bus.clear()
            .map_err(|_| FeetechBusError::CommunicationError)?;

        bus.write(&frame[..size])
            .map_err(|_| FeetechBusError::CommunicationError)?;

        read_replies(bus, self.config, |i, params: &[u8; 1]| {
            state[i].temperature = params[0] as f32;
        })
    }

    pub fn read_effort(&mut self, bus: &mut S, state: &mut [State]) -> Result<(), FeetechBusError> {
        const REG_PRESENT_LOAD: u8 = 0x3C;
        const REG_PRESENT_CURRENT: u8 = 0x45;

        if state.len() != N {
            return Err(FeetechBusError::OutOfLimits);
        }

        let servos = &self.config.servos;

        self.request_sync_read(bus, REG_PRESENT_CURRENT, 2)?;
        let current = read_replies(bus, self.config, |i, params: &[u8; 2]| {
            let cfg = &servos[i];
            let raw = u16::from_le_bytes(*params);
            let sign = if raw & 0x8000 != 0 { -1.0 } else { 1.0 };
            let amps = (raw & 0x7FFF) as f32 * cfg.current_lsb;

            let kt = cfg.torque / cfg.stall_current;
            let torque_current = (amps - cfg.no_load_current).max(0.0);

            state[i].current = sign * amps;
            state[i].effort = sign * torque_current * kt * cfg.gear_ratio;
        });

        self.request_sync_read(bus, REG_PRESENT_LOAD, 2)?;
        let load = read_replies(bus, self.config, |i, params: &[u8; 2]| {
            let raw = u16::from_le_bytes(*params);
            let sign = if raw & 0x0400 != 0 { -1.0 } else { 1.0 };
            state[i].load = sign * (raw & 0x03FF) as f32 / 1000.0;
        });

        current.and(load)
    }

    /// Shared with `homing`: `read_u16_all` uses it for the EEPROM registers too.
    pub(super) fn request_sync_read(
        &self,
        bus: &mut S,
        register: u8,
        length: u8,
    ) -> Result<(), FeetechBusError> {
        let mut ids = [0u8; 64];
        if N > ids.len() {
            return Err(FeetechBusError::OutOfLimits);
        }
        for (slot, cfg) in ids.iter_mut().zip(self.config.servos.iter()) {
            *slot = cfg.id;
        }

        let (frame, size) = RequestBuilder::<64>::new()
            .set_id(0xFE)
            .set_instruction(SYNC_READ)
            .set_register(register)
            .set_sync_length(length)
            .build(&ids[..N])
            .map_err(|_| FeetechBusError::OutOfLimits)?;

        bus.clear()
            .map_err(|_| FeetechBusError::CommunicationError)?;

        bus.write(&frame[..size])
            .map_err(|_| FeetechBusError::CommunicationError)
    }

    pub fn read_position(
        &mut self,
        bus: &mut S,
        state: &mut [State],
    ) -> Result<(), FeetechBusError> {
        let mut seen = [false; N];
        self.read_position_tracked(bus, state, &mut seen)
    }

    pub(super) fn read_position_tracked(
        &mut self,
        bus: &mut S,
        state: &mut [State],
        seen: &mut [bool; N],
    ) -> Result<(), FeetechBusError> {
        if state.len() != N {
            return Err(FeetechBusError::OutOfLimits);
        }

        self.request_sync_read(bus, REG_PRESENT_POSITION, 2)?;

        let servos = &self.config.servos;
        read_replies_tracked(bus, self.config, seen, |i, params: &[u8; 2]| {
            let cfg = &servos[i];
            state[i].position = map_range(
                u16::from_le_bytes(*params) as f32,
                cfg.min_pulse as f32,
                cfg.max_pulse as f32,
                0f32,
                cfg.amplitude,
            );
        })
    }
}
