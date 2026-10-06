use crate::serial_channel::SerialChannel;
//use crate::link::{AnyLink, Link, Connected, Disconnected};

use lucy_embedded_firmware_core::actuator::{Actuator, ActuatorGroup};
use lucy_embedded_firmware_core::joint::{*};

use lucy_embedded_firmware_core::utils::map_range;

use core::f32::consts::{TAU, PI};
use core::marker::PhantomData;

// === Request builder ===

const INST_READ: u8 = 0x02;
const INST_WRITE: u8 = 0x03;
const SYNC_READ: u8 = 0x82;
const SYNC_WRITE: u8 = 0x83;

enum BuildError {
    PayloadTooLarge,
}

struct RequestBuilder<const N: usize> {
    id: u8,
    instruction: u8,
    register: u8,
    sync_length: Option<u8>,
}

impl<const N: usize> RequestBuilder<N> {
    fn new() -> Self {
        RequestBuilder {
            id: 0,
            instruction: 0,
            register: 0,
            sync_length: None,
        }
    }

    fn set_id(self, id: u8) -> Self {
        RequestBuilder {
            id,
            ..self
        }
    }

    fn set_instruction(self, instruction: u8) -> Self {
        RequestBuilder {
            instruction,
            ..self
        }
    }

    fn set_register(self, register: u8) -> Self {
        RequestBuilder {
            register,
            ..self
        }
    }

    fn set_sync_length(self, length: u8) -> Self {
        RequestBuilder {
            sync_length: Some(length),
            ..self
        }
    }

    fn build(self, payload: &[u8]) -> Result<([u8; N], usize), BuildError> {
        let prefix_len = self.sync_length.is_some() as usize;
        let total = 7 + prefix_len + payload.len();
        if total > N {
            return Err(BuildError::PayloadTooLarge);
        }

        let payload_start = 6 + prefix_len;
        let end = payload_start + payload.len();

        let mut frame = [0u8; N];
        frame[0] = 0xFF;
        frame[1] = 0xFF;
        frame[2] = self.id;
        frame[3] = (prefix_len + payload.len()) as u8 + 3;
        frame[4] = self.instruction;
        frame[5] = self.register;
        if let Some(sync_length) = self.sync_length {
            frame[6] = sync_length;
        }
        frame[payload_start..end].copy_from_slice(payload);

        let checksum = compute_checksum(&frame[2..end]);
        frame[end] = checksum;

        Ok((frame, total))
    }
}

fn compute_checksum(payload: &[u8]) -> u8 {
    let sum: u8 = payload.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
    !sum
}





// === Individual servo

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeetechServoConfig {
    pub id: u8,
    pub amplitude: f64,
    pub min_pulse: u16,
    pub max_pulse: u16,
}

impl Default for FeetechServoConfig {
    fn default() -> Self {
        FeetechServoConfig {
            id: 1,
            amplitude: TAU as f64,
            min_pulse: 0,
            max_pulse: 4095,
        }
    }
}

pub enum FeetechBusError {
    OutOfLimits,
    CommunicationError,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeetechBusConfig<const N: usize> {
    pub servos: [FeetechServoConfig; N],
}

pub struct FeetechBusDriver<S: SerialChannel, const N: usize> {
    _state: PhantomData<fn() -> S>,
    pub config: &'static FeetechBusConfig<N>
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

    fn update(&mut self, tick: u64, bus: &mut Self::Bus, command: &mut [Command], state: &mut [State]) -> Result<(), Self::Error> {
        self.set_torque(bus, command);
        self.set_position(bus, command, state);
        //self.read_positions(bus, state);
        Ok(())
    }
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    pub fn set_position(&mut self, bus: &mut S, command: &[Command], state: &mut [State]) -> Result<(), FeetechBusError> {
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
                cmd.position as f64,
                0 as f64,
                cfg.amplitude,
                cfg.min_pulse as f64,
                cfg.max_pulse as f64,
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
        const SYNC_WRITE_CHUNK_LEN: usize = 2;
        const SYNC_WRITE_DATA_LEN: u8 = 1;

        let mut payload = [0u8; 32];

        let payload_len = N * SYNC_WRITE_CHUNK_LEN;
        if payload_len > payload.len() {
            return Err(FeetechBusError::OutOfLimits);
        }

        let iter = self.config.servos
            .iter()
            .zip(command.iter())
            .zip(payload.chunks_exact_mut(SYNC_WRITE_CHUNK_LEN));

        for ((&cfg, &cmd), slot) in iter {
            let value: u8 = match cmd.torque_enabled {
                TorqueStatus::Enabled => 1,
                TorqueStatus::Disabled => 0,
            };
            slot.copy_from_slice(&[cfg.id, value]);
        }

        let payload = &payload[..payload_len];

        let (frame, size) = RequestBuilder::<32>::new()
            .set_id(0xFE)
            .set_instruction(SYNC_WRITE)
            .set_register(REG_TORQUE_ENABLE)
            .set_sync_length(SYNC_WRITE_DATA_LEN)
            .build(payload)
            .map_err(|_| FeetechBusError::OutOfLimits)?;

        bus.clear()
            .map_err(|_| FeetechBusError::CommunicationError)?;

        bus.write(&frame[..size])
            .map_err(|_| FeetechBusError::CommunicationError)?;

        Ok(())
    }

    pub fn read_temperatures(&mut self, bus: &mut S, state: &mut [State]) -> Result<(), FeetechBusError> {
        const REG_TEMP: u8 = 0x3F;
        const RESPONSE_LEN: usize = 7;

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

        // Servos answer one after another, in the same order as `ids` above.
        for s in state.iter_mut() {
            let mut rx_frame = [0u8; RESPONSE_LEN];
            bus.read(&mut rx_frame)
                .map_err(|_| FeetechBusError::CommunicationError)?;

            let rx_payload = &rx_frame[2..RESPONSE_LEN - 1];
            if rx_frame[RESPONSE_LEN - 1] != compute_checksum(rx_payload) {
                return Err(FeetechBusError::CommunicationError);
            }

            s.temperature = rx_frame[5] as f64;
        }

        Ok(())
    }

    pub fn read_positions(&mut self, bus: &mut S, state: &mut [State]) -> Result<(), FeetechBusError> {
        const REG_PRESENT_POSITION: u8 = 0x38;
        const RESPONSE_LEN: usize = 8;

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
            .set_register(REG_PRESENT_POSITION)
            .set_sync_length(2)
            .build(ids)
            .map_err(|_| FeetechBusError::OutOfLimits)?;

        bus.clear()
            .map_err(|_| FeetechBusError::CommunicationError)?;

        bus.write(&frame[..size])
            .map_err(|_| FeetechBusError::CommunicationError)?;

        for (cfg, s) in self.config.servos.iter().zip(state.iter_mut()) {
            let mut rx_frame = [0u8; RESPONSE_LEN];
            bus.read(&mut rx_frame)
                .map_err(|_| FeetechBusError::CommunicationError)?;

            let rx_payload = &rx_frame[2..RESPONSE_LEN - 1];
            if rx_frame[RESPONSE_LEN - 1] != compute_checksum(rx_payload) {
                return Err(FeetechBusError::CommunicationError);
            }

            let pulse = u16::from_le_bytes([rx_frame[5], rx_frame[6]]);
            s.position = map_range(
                pulse as f64,
                cfg.min_pulse as f64,
                cfg.max_pulse as f64,
                0f64,
                cfg.amplitude,
            );
        }

        Ok(())
    }
}

/*
impl<'cfg, 'bus, C: SerialChannel> TorqueEnableInterface for BusServoDriver<'cfg, 'bus, C> {
    type Error = BusServoError;

    fn set_torque_enable(&mut self, state: TorqueStatus) -> Result<(), Self::Error>{
        const REG_TORQUE_ENABLE: u8 = 0x28;
        let value = match state {
            TorqueStatus::Enabled => 1u8,
            TorqueStatus::Disabled => 0u8,
        };

        let frame: [u8; 8] = RequestBuilder::new()
            .set_id(self.config.id)
            .set_instruction(INST_WRITE)
            .set_register(REG_TORQUE_ENABLE)
            .set_payload([value])
            .build();

        self.link.channel
            .clear()
            .map_err(|_| BusServoError::CommunicationError)?;

        self.link.channel
            .write(&frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let mut echo_frame = [0u8; 6];
        self.link.channel
            .read(&mut echo_frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        Ok(())
    }
}

impl<'cfg, 'bus, C: SerialChannel> StateInterface for BusServoDriver<'cfg, 'bus, C> {
    type Error = BusServoError;

    fn get_position(&mut self) -> Result<f64, Self::Error> {
        const REG_STATE_POSITION: u8 = 0x38;
        let length = 4u8;
        let bytes_to_read = 2u8;

        let mut tx_frame = [
            0xFF,
            0xFF,
            self.config.id,
            length,
            INST_READ,
            REG_STATE_POSITION,
            bytes_to_read,
            0x00,
        ];
        let payload = &tx_frame[2..tx_frame.len() - 1];
        tx_frame[tx_frame.len() - 1] = compute_checksum(payload);

        self.link.channel
            .clear()
            .map_err(|_| BusServoError::CommunicationError)?;

        self.link.channel
            .write(&tx_frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let mut rx_frame = [0u8; 8];
        let n = self.link.channel
            .read(&mut rx_frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let rx_payload = &rx_frame[2..rx_frame.len() - 1];
        let expected_checksum = compute_checksum(rx_payload);
        if rx_frame[rx_frame.len() - 1] != expected_checksum {
            return Err(BusServoError::CommunicationError.into());
        }

        let error_status = rx_frame[4];
        let pulse = u16::from_le_bytes([rx_frame[5], rx_frame[6]]);
        let position = map_range(
            pulse as f64,
            0f64,
            4096f64,
            0f64,
            TAU as f64
        );

        Ok(position)
    }
}

impl<'cfg, 'bus, C: SerialChannel> TemperatureInterface for BusServoDriver<'cfg, 'bus, C> {
    type Error = BusServoError;

    fn get_temperature(&mut self) -> Result<f64, Self::Error> {
        const REG_TEMP: u8 = 0x3F;
        let length = 4u8;
        let bytes_to_read = 1u8;

        let mut tx_frame = [
            0xFF,
            0xFF,
            self.config.id,
            length,
            INST_READ,
            REG_TEMP,
            bytes_to_read,
            0x00,
        ];
        let payload = &tx_frame[2..tx_frame.len() - 1];
        tx_frame[tx_frame.len() - 1] = compute_checksum(payload);

        self.link.channel
            .clear()
            .map_err(|_| BusServoError::CommunicationError)?;

        self.link.channel
            .write(&tx_frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let mut rx_frame = [0u8; 7];
        let n = self.link.channel
            .read(&mut rx_frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let rx_payload = &rx_frame[2..rx_frame.len() - 1];
        let expected_checksum = compute_checksum(rx_payload);
        if rx_frame[rx_frame.len() - 1] != expected_checksum {
            return Err(BusServoError::CommunicationError.into());
        }

        let error_status = rx_frame[4];
        let temperature = rx_frame[5];

        Ok(temperature as f64)
    }
}

impl<'cfg, 'bus, C: SerialChannel> TorqueInterface for BusServoDriver<'cfg, 'bus, C> {
    type Error = BusServoError;

    fn get_torque(&mut self) -> Result<f64, Self::Error> {
        const REG_LOAD: u8 = 0x3c;
        let length = 4u8;
        let bytes_to_read = 2u8;

        let mut tx_frame = [
            0xFF,
            0xFF,
            self.config.id,
            length,
            INST_READ,
            REG_LOAD,
            bytes_to_read,
            0x00,
        ];
        let payload = &tx_frame[2..tx_frame.len() - 1];
        tx_frame[tx_frame.len() - 1] = compute_checksum(payload);

        self.link.channel
            .clear()
            .map_err(|_| BusServoError::CommunicationError)?;

        self.link.channel
            .write(&tx_frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let mut rx_frame = [0u8; 8];
        let n = self.link.channel
            .read(&mut rx_frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let rx_payload = &rx_frame[2..rx_frame.len() - 1];
        let expected_checksum = compute_checksum(rx_payload);
        if rx_frame[rx_frame.len() - 1] != expected_checksum {
            return Err(BusServoError::CommunicationError.into());
        }

        let error_status = rx_frame[4];

        let raw = u16::from_le_bytes([rx_frame[5], rx_frame[6]]);
        let magnitude = (raw & 0x3FF) as f64 / 1000.0;
        let sign = if raw & 0x400 != 0 { -1.0 } else { 1.0 };
        let torque = sign * magnitude;

        Ok(torque)
    }
}





impl<'cfg, 'bus, C: SerialChannel> BusServoPortDriver<'cfg, 'bus, C> {

    fn set_joint_trajectory(&mut self, jtp: JointTrajectoryPoint) -> Result<(), BusServoError> {

        let payload = [0u8; 3];
        let position = 0;
        for servo in self.config.servos.iter() {
            let pulse = (map_range(
                position as f64,
                0 as f64,
                servo.config.amplitude,
                servo.config.min_pulse as f64,
                servo.config.max_pulse as f64,
            ) + 0.5) as u16;
            let time: u16 = 0;
            let velocity: u16 = jtp.velocity as u16;
        }

        const REG_TARGET_POSITION: u8 = 0x2A;

        let [pos_l, pos_h] = pulse.to_le_bytes();
        let [time_l, time_h] = time.to_le_bytes();
        let [spd_l, spd_h] = velocity.to_le_bytes();
        let payload = [pos_l, pos_h, time_l, time_h, spd_l, spd_h];

        let frame: [u8; 13] = RequestBuilder::new()
            .set_id(self.config.id)
            .set_instruction(INST_WRITE)
            .set_register(REG_TARGET_POSITION)
            .set_payload(payload)
            .build();

        self.link.channel
            .clear()
            .map_err(|_| BusServoError::CommunicationError)?;

        self.link.channel
            .write(&frame)
            .map_err(|_| BusServoError::CommunicationError)?;

        let mut echo_frame = [0u8; 8];
        self.link.channel
            .read(&mut echo_frame)
            .map_err(|_| BusServoError::CommunicationError)?;
        Ok(())
    }
}
*/
