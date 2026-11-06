use crate::serial_channel::SerialChannel;

use lucy_embedded_firmware_core::actuator::{ActuatorGroup};
use lucy_embedded_firmware_core::joint::{*};

use lucy_embedded_firmware_core::utils::map_range;

use core::f32::consts::{TAU};
use core::marker::PhantomData;

// === Request builder ===

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

fn read_status<S: SerialChannel>(bus: &mut S, params: &mut [u8]) -> Result<u8, FeetechBusError> {
    const MAX_SKIPPED_BYTES: usize = 64;

    let mut skipped = 0usize;
    let mut byte = [0u8; 1];
    let mut prev = 0u8;
    loop {
        if skipped > MAX_SKIPPED_BYTES {
            return Err(FeetechBusError::CommunicationError);
        }

        bus.read(&mut byte).map_err(|_| FeetechBusError::CommunicationError)?;
        if !(prev == 0xFF && byte[0] == 0xFF) {
            prev = byte[0];
            skipped += 1;
            continue;
        }

        let mut id = 0xFFu8;
        while id == 0xFF && skipped <= MAX_SKIPPED_BYTES {
            bus.read(&mut byte).map_err(|_| FeetechBusError::CommunicationError)?;
            id = byte[0];
            skipped += 1;
        }
        bus.read(&mut byte).map_err(|_| FeetechBusError::CommunicationError)?;
        let len = byte[0] as usize;
        prev = 0;

        let mut body = [0u8; 64];
        if len < 2 || len > body.len() {
            skipped += 2;
            continue;
        }
        bus.read(&mut body[..len]).map_err(|_| FeetechBusError::CommunicationError)?;

        let sum = [id, len as u8]
            .iter()
            .chain(&body[..len - 1])
            .fold(0u8, |acc, &x| acc.wrapping_add(x));
        if !sum != body[len - 1] || id == 0xFE || len - 2 != params.len() {
            skipped += 2 + len;
            continue;
        }

        params.copy_from_slice(&body[1..len - 1]);
        return Ok(id);
    }
}

fn read_replies<S: SerialChannel, const N: usize, const P: usize>(
    bus: &mut S,
    config: &FeetechBusConfig<N>,
    apply: impl FnMut(usize, &[u8; P]),
) -> Result<(), FeetechBusError> {
    let mut seen = [false; N];
    read_replies_tracked(bus, config, &mut seen, apply)
}

/// Like [`read_replies`], but leaves in `seen` which servos answered, also on failure.
fn read_replies_tracked<S: SerialChannel, const N: usize, const P: usize>(
    bus: &mut S,
    config: &FeetechBusConfig<N>,
    seen: &mut [bool; N],
    mut apply: impl FnMut(usize, &[u8; P]),
) -> Result<(), FeetechBusError> {
    let mut params = [0u8; P];
    while seen.iter().any(|s| !s) {
        let id = read_status(bus, &mut params)?;
        if let Some(i) = config.servos.iter().position(|cfg| cfg.id == id) {
            seen[i] = true;
            apply(i, &params);
        }
    }
    Ok(())
}

// === Individual servo

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
        let extra = match tick % 10 {
            0 => self.read_temperature(bus, state),
            1 => self.read_effort(bus, state),
            _ => Ok(()),
        };
        torque.and(position).and(feedback).and(extra)
    }
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    pub fn set_position(&mut self, bus: &mut S, command: &[Command]) -> Result<(), FeetechBusError> {
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
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    pub fn read_temperature(&mut self, bus: &mut S, state: &mut [State]) -> Result<(), FeetechBusError> {
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

    fn request_sync_read(&self, bus: &mut S, register: u8, length: u8) -> Result<(), FeetechBusError> {
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

    pub fn read_position(&mut self, bus: &mut S, state: &mut [State]) -> Result<(), FeetechBusError> {
        let mut seen = [false; N];
        self.read_position_tracked(bus, state, &mut seen)
    }

    fn read_position_tracked(
        &mut self,
        bus: &mut S,
        state: &mut [State],
        seen: &mut [bool; N],
    ) -> Result<(), FeetechBusError> {
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

// === EEPROM homing (calibration only, never from the control loop) ===
//
// Everything here uses SYNC_READ / SYNC_WRITE. Unlike a unicast READ, whose echo on a half-duplex
// adapter parses as a valid reply from that servo, their echo carries id 0xFE and is skipped.

const REG_MIN_POSITION_LIMIT: u8 = 0x09;
const REG_MAX_POSITION_LIMIT: u8 = 0x0B;
const REG_HOMING_OFFSET: u8 = 0x1F;
const REG_LOCK: u8 = 0x37;
const REG_PRESENT_POSITION: u8 = 0x38;

const HOMING_SIGN_BIT: u16 = 1 << 11;
const HOMING_MAGNITUDE_MAX: i16 = 2047;
const VERIFY_ATTEMPTS: usize = 5;

/// `Present_Position` of the mid pose once homed, as LeRobot's half-turn homing sets it.
pub const HALF_TURN_TICK: u16 = 2047;
/// Highest raw position of the 12-bit encoder.
pub const MAX_TICK: u16 = 4095;

/// The per-servo EEPROM values calibration owns.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct HomingRecord {
    pub homing_offset: i16,
    pub min_limit: u16,
    pub max_limit: u16,
}

impl HomingRecord {
    /// No offset and the whole encoder range, so raw positions read back unmodified.
    pub const CLEARED: Self = Self { homing_offset: 0, min_limit: 0, max_limit: MAX_TICK };

    /// Whether every field fits its register.
    pub fn is_valid(&self) -> bool {
        self.homing_offset.abs() <= HOMING_MAGNITUDE_MAX
            && self.min_limit <= MAX_TICK
            && self.max_limit <= MAX_TICK
    }
}

/// EEPROM values before and after a half-turn homing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HalfTurn<const N: usize> {
    pub old: [HomingRecord; N],
    pub new: [HomingRecord; N],
}

/// A failed homing. `restored` tells whether the servos still hold their previous values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HomingFault {
    pub error: FeetechBusError,
    pub restored: bool,
}

/// `Homing_Offset` is sign-magnitude with the sign in bit 11. `None` if it does not fit.
pub fn encode_homing_offset(offset: i16) -> Option<u16> {
    if offset.abs() > HOMING_MAGNITUDE_MAX {
        return None;
    }
    let magnitude = offset.unsigned_abs();
    Some(if offset < 0 { magnitude | HOMING_SIGN_BIT } else { magnitude })
}

pub fn decode_homing_offset(raw: u16) -> i16 {
    let magnitude = (raw & (HOMING_SIGN_BIT - 1)) as i16;
    if raw & HOMING_SIGN_BIT != 0 { -magnitude } else { magnitude }
}

/// The offset making `tick` read [`HALF_TURN_TICK`], from
/// `Present_Position = Actual_Position - Homing_Offset`. `None` if it does not fit.
pub fn half_turn_offset(tick: u16) -> Option<i16> {
    let offset = tick as i32 - HALF_TURN_TICK as i32;
    if offset.abs() > HOMING_MAGNITUDE_MAX as i32 { None } else { Some(offset as i16) }
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    fn sync_write_all<const D: usize>(
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

    fn read_u16_all(&self, bus: &mut S, register: u8) -> Result<[u16; N], FeetechBusError> {
        let mut values = [0u16; N];
        self.request_sync_read(bus, register, 2)?;
        read_replies(bus, self.config, |i, params: &[u8; 2]| {
            values[i] = u16::from_le_bytes(*params);
        })?;
        Ok(values)
    }

    /// Raw `Present_Position` of every servo. Fails unless all of them answer.
    pub fn read_present_ticks(&mut self, bus: &mut S) -> Result<[u16; N], FeetechBusError> {
        self.read_u16_all(bus, REG_PRESENT_POSITION)
    }

    /// The EEPROM values of every servo. Fails unless all of them answer.
    pub fn read_homing(&mut self, bus: &mut S) -> Result<[HomingRecord; N], FeetechBusError> {
        let offsets = self.read_u16_all(bus, REG_HOMING_OFFSET)?;
        let mins = self.read_u16_all(bus, REG_MIN_POSITION_LIMIT)?;
        let maxes = self.read_u16_all(bus, REG_MAX_POSITION_LIMIT)?;

        let mut records = [HomingRecord::default(); N];
        for (i, record) in records.iter_mut().enumerate() {
            *record = HomingRecord {
                homing_offset: decode_homing_offset(offsets[i]),
                min_limit: mins[i],
                max_limit: maxes[i],
            };
        }
        Ok(records)
    }

    /// Write `records` to EEPROM and confirm by reading them back, then re-lock the EEPROM.
    /// `delay` runs after each write and between read-backs, as EEPROM writes take time.
    pub fn write_homing(
        &mut self,
        bus: &mut S,
        records: &[HomingRecord; N],
        delay: &mut impl FnMut(),
    ) -> Result<(), FeetechBusError> {
        self.sync_write_all(bus, REG_LOCK, &[[0u8]; N])?;
        delay();
        let written = self.write_and_verify(bus, records, delay);
        delay();
        let locked = self.sync_write_all(bus, REG_LOCK, &[[1u8]; N]);
        written.and(locked)
    }

    fn write_and_verify(
        &mut self,
        bus: &mut S,
        records: &[HomingRecord; N],
        delay: &mut impl FnMut(),
    ) -> Result<(), FeetechBusError> {
        let mut offsets = [[0u8; 2]; N];
        let mut mins = [[0u8; 2]; N];
        let mut maxes = [[0u8; 2]; N];
        for (i, record) in records.iter().enumerate() {
            if !record.is_valid() {
                return Err(FeetechBusError::OutOfLimits);
            }
            let encoded = encode_homing_offset(record.homing_offset)
                .ok_or(FeetechBusError::OutOfLimits)?;
            offsets[i] = encoded.to_le_bytes();
            mins[i] = record.min_limit.to_le_bytes();
            maxes[i] = record.max_limit.to_le_bytes();
        }

        for (register, values) in [
            (REG_HOMING_OFFSET, &offsets),
            (REG_MIN_POSITION_LIMIT, &mins),
            (REG_MAX_POSITION_LIMIT, &maxes),
        ] {
            self.sync_write_all(bus, register, values)?;
            delay();
        }

        let mut outcome = Err(FeetechBusError::ReadbackMismatch);
        for _ in 0..VERIFY_ATTEMPTS {
            match self.read_homing(bus) {
                Ok(read) if read == *records => return Ok(()),
                Ok(_) => outcome = Err(FeetechBusError::ReadbackMismatch),
                Err(e) => outcome = Err(e),
            }
            delay();
        }
        outcome
    }

    /// LeRobot's half-turn homing: make the pose the arm holds now read [`HALF_TURN_TICK`], with
    /// the whole encoder range allowed. Torque must already be off; the caller checks that.
    ///
    /// On failure the previous values are written back, and the fault says if that worked.
    pub fn set_half_turn(
        &mut self,
        bus: &mut S,
        delay: &mut impl FnMut(),
    ) -> Result<HalfTurn<N>, HomingFault> {
        let old = self
            .read_homing(bus)
            .map_err(|error| HomingFault { error, restored: true })?;

        self.home_from_cleared(bus, delay)
            .map(|new| HalfTurn { old, new })
            .map_err(|error| HomingFault {
                error,
                restored: self.write_homing(bus, &old, delay).is_ok(),
            })
    }

    fn home_from_cleared(
        &mut self,
        bus: &mut S,
        delay: &mut impl FnMut(),
    ) -> Result<[HomingRecord; N], FeetechBusError> {
        self.write_homing(bus, &[HomingRecord::CLEARED; N], delay)?;

        let ticks = self.read_present_ticks(bus)?;
        let mut new = [HomingRecord::CLEARED; N];
        for (i, record) in new.iter_mut().enumerate() {
            record.homing_offset = half_turn_offset(ticks[i])
                .ok_or(FeetechBusError::OffsetOutOfRange(self.config.servos[i].id))?;
        }

        self.write_homing(bus, &new, delay)?;
        Ok(new)
    }
}
