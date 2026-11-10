use std::collections::HashMap;
use std::collections::VecDeque;

use lucy_embedded_firmware_feetech::feetech::MAX_TICK;
use lucy_embedded_firmware_feetech::feetech::decode_homing_offset;
use lucy_embedded_firmware_serial::SerialChannel;

pub(crate) const REG_MIN: u8 = 9;
pub(crate) const REG_MAX: u8 = 11;
pub(crate) const REG_HOMING: u8 = 31;
pub(crate) const REG_LOCK: u8 = 55;
pub(crate) const REG_PRESENT: u8 = 56;
pub(crate) const REG_P_COEFFICIENT: u8 = 21;
pub(crate) const REG_LOAD: u8 = 0x3C;
pub(crate) const REG_TEMPERATURE: u8 = 0x3F;
pub(crate) const REG_CURRENT: u8 = 0x45;

pub(crate) const SYNC_READ: u8 = 0x82;
pub(crate) const SYNC_WRITE: u8 = 0x83;

/// One servo: an EEPROM that ignores writes while locked, and an encoder.
pub(crate) struct FakeServo {
    pub(crate) actual: u16,
    pub(crate) homing: u16,
    pub(crate) min: u16,
    pub(crate) max: u16,
    pub(crate) locked: bool,
    pub(crate) answers: bool,
    /// Read-backs still to answer with the old value, as a servo busy writing EEPROM could.
    pub(crate) stale_reads: usize,
    /// Homing writes this servo still accepts; later ones are silently lost.
    pub(crate) homing_writes_left: usize,
    pub(crate) gains: [u8; 3],
}

impl FakeServo {
    pub(crate) fn new(actual: u16) -> Self {
        Self {
            actual,
            homing: 0,
            min: 0,
            max: MAX_TICK,
            locked: true,
            answers: true,
            stale_reads: 0,
            homing_writes_left: usize::MAX,
            gains: [32, 32, 0],
        }
    }

    pub(crate) fn present(&self) -> u16 {
        let offset = decode_homing_offset(self.homing) as i32;
        (self.actual as i32 - offset).rem_euclid(4096) as u16
    }

    fn write(&mut self, register: u8, data: &[u8]) {
        if register == REG_LOCK {
            self.locked = data[0] != 0;
            return;
        }
        if self.locked {
            return;
        }
        if register == REG_P_COEFFICIENT {
            self.gains.copy_from_slice(&data[..3]);
            return;
        }
        let value = u16::from_le_bytes([data[0], data[1]]);
        match register {
            REG_MIN => self.min = value,
            REG_MAX => self.max = value,
            REG_HOMING if self.homing_writes_left > 0 => {
                self.homing_writes_left -= 1;
                self.homing = value;
            }
            _ => {}
        }
    }

    fn read(&self, register: u8) -> u16 {
        match register {
            REG_MIN => self.min,
            REG_MAX => self.max,
            REG_HOMING => self.homing,
            REG_PRESENT => self.present(),
            REG_LOAD | REG_TEMPERATURE | REG_CURRENT => 0,
            _ => panic!("unexpected read of register {register}"),
        }
    }
}

#[derive(Default)]
pub(crate) struct FakeBus {
    pub(crate) servos: HashMap<u8, FakeServo>,
    rx: VecDeque<u8>,
    pub(crate) instructions: Vec<u8>,
    pub(crate) read_registers: Vec<u8>,
}

impl FakeBus {
    pub(crate) fn with_positions(ticks: &[(u8, u16)]) -> Self {
        Self {
            servos: ticks
                .iter()
                .map(|&(id, tick)| (id, FakeServo::new(tick)))
                .collect(),
            ..Self::default()
        }
    }

    pub(crate) fn servo(&mut self, id: u8) -> &mut FakeServo {
        self.servos.get_mut(&id).unwrap()
    }

    fn reply(&mut self, id: u8, value: u16) {
        let [l, h] = value.to_le_bytes();
        let body = [id, 4, 0, l, h];
        let sum = body.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
        self.rx.extend([0xFF, 0xFF]);
        self.rx.extend(body);
        self.rx.push_back(!sum);
    }
}

impl SerialChannel for FakeBus {
    type Error = ();

    fn write(&mut self, bytes: &[u8]) -> Result<(), ()> {
        assert_eq!(
            &bytes[..3],
            &[0xFF, 0xFF, 0xFE],
            "homing must only use sync frames"
        );
        let instruction = bytes[4];
        self.instructions.push(instruction);
        let register = bytes[5];
        let length = bytes[6] as usize;
        let params = &bytes[7..bytes.len() - 1];

        match instruction {
            SYNC_WRITE => {
                for chunk in params.chunks_exact(length + 1) {
                    if let Some(servo) = self.servos.get_mut(&chunk[0]) {
                        servo.write(register, &chunk[1..]);
                    }
                }
            }
            SYNC_READ => {
                self.read_registers.push(register);
                for &id in params {
                    let Some(servo) = self.servos.get_mut(&id) else {
                        continue;
                    };
                    if !servo.answers {
                        continue;
                    }
                    let value = servo.read(register);
                    let value = if servo.stale_reads > 0 {
                        servo.stale_reads -= 1;
                        value ^ 1
                    } else {
                        value
                    };
                    self.reply(id, value);
                }
            }
            other => panic!("unexpected instruction {other:#x}"),
        }
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, ()> {
        for slot in buffer.iter_mut() {
            *slot = self.rx.pop_front().ok_or(())?;
        }
        Ok(buffer.len())
    }

    fn clear(&mut self) -> Result<(), ()> {
        self.rx.clear();
        Ok(())
    }
}
