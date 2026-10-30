#![no_std]

//! Frame: `COBS(postcard(Frame<N>) ++ CRC-16/MODBUS LE) ++ 0x00`.

use crc::{Crc, Digest, CRC_16_MODBUS};
use postcard::ser_flavors::{Cobs, Flavor, Slice};
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

pub const PROTOCOL_VERSION: u8 = 1;

static FRAME_CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

pub const fn frame_len(joints: usize) -> usize {
    // version + joints (varint u32) + seq (varint u32) + body tag, then the largest body
    // (Command: 3 f64 arrays + 1 bool array), then the CRC.
    let raw = 1 + 5 + 5 + 1 + joints * (3 * 8 + 1) + 2;
    cobs::max_encoding_length(raw) + 1
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct JointCommands<const N: usize> {
    #[serde(with = "BigArray")]
    pub positions: [f64; N],
    #[serde(with = "BigArray")]
    pub velocities: [f64; N],
    #[serde(with = "BigArray")]
    pub accelerations: [f64; N],
    #[serde(with = "BigArray")]
    pub torque_enabled: [bool; N],
}

impl<const N: usize> Default for JointCommands<N> {
    fn default() -> Self {
        Self {
            positions: [0.0; N],
            velocities: [0.0; N],
            accelerations: [0.0; N],
            torque_enabled: [false; N],
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct JointStates<const N: usize> {
    #[serde(with = "BigArray")]
    pub positions: [f64; N],
}

impl<const N: usize> Default for JointStates<N> {
    fn default() -> Self {
        Self { positions: [0.0; N] }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub enum Body<const N: usize> {
    Command(JointCommands<N>),
    State(JointStates<N>),
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct Frame<const N: usize> {
    pub version: u8,
    pub joints: u32,
    pub seq: u32,
    pub body: Body<N>,
}

impl<const N: usize> Frame<N> {
    pub const fn new(seq: u32, body: Body<N>) -> Self {
        Self { version: PROTOCOL_VERSION, joints: N as u32, seq, body }
    }
}

#[derive(Deserialize)]
struct Header {
    version: u8,
    joints: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Serialize,
    Deserialize,
    Cobs,
    Crc,
    Overflow,
    Version(u8),
    JointCount { expected: u32, received: u32 },
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Serialize => write!(f, "frame does not fit in the output buffer"),
            Error::Deserialize => write!(f, "malformed frame payload"),
            Error::Cobs => write!(f, "invalid COBS encoding"),
            Error::Crc => write!(f, "CRC mismatch"),
            Error::Overflow => write!(f, "frame longer than the decoder buffer"),
            Error::Version(v) => write!(f, "protocol version {v}, expected {PROTOCOL_VERSION}"),
            Error::JointCount { expected, received } => {
                write!(f, "frame carries {received} joints, expected {expected}")
            }
        }
    }
}

struct CrcCobs<'a> {
    cobs: Cobs<Slice<'a>>,
    digest: Digest<'static, u16>,
}

impl<'a> Flavor for CrcCobs<'a> {
    type Output = &'a mut [u8];

    fn try_push(&mut self, data: u8) -> postcard::Result<()> {
        self.digest.update(&[data]);
        self.cobs.try_push(data)
    }

    fn try_extend(&mut self, data: &[u8]) -> postcard::Result<()> {
        self.digest.update(data);
        self.cobs.try_extend(data)
    }

    fn finalize(mut self) -> postcard::Result<Self::Output> {
        let crc = self.digest.finalize();
        self.cobs.try_extend(&crc.to_le_bytes())?;
        self.cobs.finalize()
    }
}

pub fn encode<const N: usize>(frame: &Frame<N>, out: &mut [u8]) -> Result<usize, Error> {
    let cobs = Cobs::try_new(Slice::new(out)).map_err(|_| Error::Serialize)?;
    let flavor = CrcCobs { cobs, digest: FRAME_CRC.digest() };
    postcard::serialize_with_flavor(frame, flavor)
        .map(|written| written.len())
        .map_err(|_| Error::Serialize)
}

pub fn decode<const N: usize>(encoded: &mut [u8]) -> Result<Frame<N>, Error> {
    let n = cobs::decode_in_place(encoded).map_err(|_| Error::Cobs)?;
    if n < 2 {
        return Err(Error::Cobs);
    }
    let (payload, crc) = encoded[..n].split_at(n - 2);
    if FRAME_CRC.checksum(payload) != u16::from_le_bytes([crc[0], crc[1]]) {
        return Err(Error::Crc);
    }

    let (header, _) = postcard::take_from_bytes::<Header>(payload).map_err(|_| Error::Deserialize)?;
    if header.version != PROTOCOL_VERSION {
        return Err(Error::Version(header.version));
    }
    if header.joints != N as u32 {
        return Err(Error::JointCount { expected: N as u32, received: header.joints });
    }

    match postcard::take_from_bytes::<Frame<N>>(payload) {
        Ok((frame, [])) => Ok(frame),
        _ => Err(Error::Deserialize),
    }
}

pub struct FrameDecoder<'a, const N: usize> {
    buf: &'a mut [u8],
    len: usize,
    overflow: bool,
}

impl<'a, const N: usize> FrameDecoder<'a, N> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0, overflow: false }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.overflow = false;
    }

    pub fn push_byte(&mut self, byte: u8) -> Option<Result<Frame<N>, Error>> {
        if byte != 0 {
            if self.len < self.buf.len() {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }

        let len = core::mem::take(&mut self.len);
        if core::mem::take(&mut self.overflow) {
            return Some(Err(Error::Overflow));
        }
        if len == 0 {
            return None;
        }
        Some(decode(&mut self.buf[..len]))
    }
}
