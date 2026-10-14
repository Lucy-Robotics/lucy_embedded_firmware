#![no_std]

use crc::{Crc, CRC_16_MODBUS};

pub const FRAME_MAGIC: u8 = 0xAA;

const FRAME_CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

pub const fn framed_len(payload_len: usize) -> usize {
    payload_len + 1 + 2
}

pub fn encode_frame(payload: &[u8], frame: &mut [u8]) {
    assert_eq!(frame.len(), framed_len(payload.len()));

    frame[0] = FRAME_MAGIC;
    frame[1..1 + payload.len()].copy_from_slice(payload);

    let crc = FRAME_CRC.checksum(payload);
    frame[1 + payload.len()..].copy_from_slice(&crc.to_le_bytes());
}

pub struct FrameDecoder<const N: usize> {
    payload: [u8; N],
    crc_bytes: [u8; 2],
    pos: usize,
    synced: bool,
}

impl<const N: usize> FrameDecoder<N> {
    pub const fn new() -> Self {
        Self {
            payload: [0u8; N],
            crc_bytes: [0u8; 2],
            pos: 0,
            synced: false,
        }
    }

    pub fn push_byte(&mut self, byte: u8) -> Option<&[u8; N]> {
        if !self.synced {
            if byte == FRAME_MAGIC {
                self.synced = true;
                self.pos = 0;
            }
            return None;
        }

        if self.pos < N {
            self.payload[self.pos] = byte;
        } else {
            self.crc_bytes[self.pos - N] = byte;
        }
        self.pos += 1;

        if self.pos == N + 2 {
            self.synced = false;
            let crc = FRAME_CRC.checksum(&self.payload);
            if crc == u16::from_le_bytes(self.crc_bytes) {
                return Some(&self.payload);
            }
        }

        None
    }
}

impl<const N: usize> Default for FrameDecoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub fn read_frame<E>(
    mut read_exact: impl FnMut(&mut [u8]) -> Result<(), E>,
    frame: &mut [u8],
) -> Result<bool, E> {
    let mut magic = [0u8];
    loop {
        read_exact(&mut magic)?;
        if magic[0] == FRAME_MAGIC {
            break;
        }
    }

    read_exact(frame)?;

    let payload_len = frame.len() - 2;
    let crc = FRAME_CRC.checksum(&frame[..payload_len]);
    let crc_received = u16::from_le_bytes([frame[payload_len], frame[payload_len + 1]]);
    Ok(crc == crc_received)
}
