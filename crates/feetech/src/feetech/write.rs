use super::driver::{FeetechBusDriver, FeetechBusError};
use super::protocol::{RequestBuilder, SYNC_WRITE};
use crate::serial_channel::SerialChannel;

use lucy_embedded_firmware_core::joint::*;
use lucy_embedded_firmware_core::utils::map_range;

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    pub fn set_position(
        &mut self,
        bus: &mut S,
        command: &[Command],
    ) -> Result<(), FeetechBusError> {
        const SYNC_WRITE_CHUNK_LEN: usize = 7;
        const SYNC_WRITE_DATA_LEN: u8 = 6;

        let mut payload = [0u8; 64];

        let max_payload_len = N * SYNC_WRITE_CHUNK_LEN;
        if max_payload_len > payload.len() {
            return Err(FeetechBusError::OutOfLimits);
        }

        let mut count = 0usize;
        for (&cfg, &cmd) in self.config.servos.iter().zip(command.iter()) {
            if cmd.torque_enabled != TorqueStatus::Enabled {
                continue;
            }

            let time: u16 = 0;
            let velocity = cmd.velocity as u16;
            let pulse = (map_range(
                cmd.position,
                0 as f32,
                cfg.amplitude,
                cfg.min_pulse as f32,
                cfg.max_pulse as f32,
            ) + 0.5) as u16;

            let [pos_l, pos_h] = pulse.to_le_bytes();
            let [time_l, time_h] = time.to_le_bytes();
            let [spd_l, spd_h] = velocity.to_le_bytes();

            let offset = count * SYNC_WRITE_CHUNK_LEN;
            payload[offset..offset + SYNC_WRITE_CHUNK_LEN]
                .copy_from_slice(&[cfg.id, pos_l, pos_h, time_l, time_h, spd_l, spd_h]);
            count += 1;
        }

        if count == 0 {
            return Ok(());
        }

        let payload = &payload[..count * SYNC_WRITE_CHUNK_LEN];

        const REG_TARGET_POSITION: u8 = 0x2A;

        let (frame, size) = RequestBuilder::<64>::new()
            .set_id(0xFE)
            .set_instruction(SYNC_WRITE)
            .set_register(REG_TARGET_POSITION)
            .set_sync_length(SYNC_WRITE_DATA_LEN)
            .build(payload)
            .map_err(|_| FeetechBusError::OutOfLimits)?;

        bus.clear()
            .map_err(|_| FeetechBusError::CommunicationError)?;

        bus.write(&frame[..size])
            .map_err(|_| FeetechBusError::CommunicationError)?;

        Ok(())
    }

    pub fn set_torque(&mut self, bus: &mut S, command: &[Command]) -> Result<(), FeetechBusError> {
        const REG_TORQUE_ENABLE: u8 = 0x28;

        let mut values = [[0u8; 1]; N];
        for (cmd, value) in command.iter().zip(values.iter_mut()) {
            value[0] = match cmd.torque_enabled {
                TorqueStatus::Enabled => 1,
                TorqueStatus::Disabled => 0,
            };
        }

        self.sync_write_all(bus, REG_TORQUE_ENABLE, &values)
    }

    /// Shared by the control loop (`set_torque`) and EEPROM homing (`write_homing`).
    pub(super) fn sync_write_all<const D: usize>(
        &self,
        bus: &mut S,
        register: u8,
        values: &[[u8; D]; N],
    ) -> Result<(), FeetechBusError> {
        let chunk = D + 1;
        let mut payload = [0u8; 64];
        if N * chunk > payload.len() {
            return Err(FeetechBusError::OutOfLimits);
        }
        for (i, (cfg, value)) in self.config.servos.iter().zip(values.iter()).enumerate() {
            payload[i * chunk] = cfg.id;
            payload[i * chunk + 1..(i + 1) * chunk].copy_from_slice(value);
        }

        let (frame, size) = RequestBuilder::<80>::new()
            .set_id(0xFE)
            .set_instruction(SYNC_WRITE)
            .set_register(register)
            .set_sync_length(D as u8)
            .build(&payload[..N * chunk])
            .map_err(|_| FeetechBusError::OutOfLimits)?;

        bus.clear()
            .map_err(|_| FeetechBusError::CommunicationError)?;
        bus.write(&frame[..size])
            .map_err(|_| FeetechBusError::CommunicationError)
    }
}
