//! EEPROM homing (calibration only, never from the control loop).
//!
//! Everything here uses SYNC_READ / SYNC_WRITE. Unlike a unicast READ, whose echo on a
//! half-duplex adapter parses as a valid reply from that servo, their echo carries id 0xFE
//! and is skipped.

use super::driver::{FeetechBusDriver, FeetechBusError};
use super::protocol::read_replies;
use super::read::REG_PRESENT_POSITION;
use lucy_embedded_firmware_serial::SerialChannel;

const REG_MIN_POSITION_LIMIT: u8 = 0x09;
const REG_MAX_POSITION_LIMIT: u8 = 0x0B;
const REG_HOMING_OFFSET: u8 = 0x1F;
const REG_LOCK: u8 = 0x37;

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
    pub const CLEARED: Self = Self {
        homing_offset: 0,
        min_limit: 0,
        max_limit: MAX_TICK,
    };

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
    Some(if offset < 0 {
        magnitude | HOMING_SIGN_BIT
    } else {
        magnitude
    })
}

pub fn decode_homing_offset(raw: u16) -> i16 {
    let magnitude = (raw & (HOMING_SIGN_BIT - 1)) as i16;
    if raw & HOMING_SIGN_BIT != 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// The offset making `tick` read [`HALF_TURN_TICK`], from
/// `Present_Position = Actual_Position - Homing_Offset`. `None` if it does not fit.
pub fn half_turn_offset(tick: u16) -> Option<i16> {
    let offset = tick as i32 - HALF_TURN_TICK as i32;
    if offset.abs() > HOMING_MAGNITUDE_MAX as i32 {
        None
    } else {
        Some(offset as i16)
    }
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
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
            let encoded =
                encode_homing_offset(record.homing_offset).ok_or(FeetechBusError::OutOfLimits)?;
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
        let old = self.read_homing(bus).map_err(|error| HomingFault {
            error,
            restored: true,
        })?;

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
