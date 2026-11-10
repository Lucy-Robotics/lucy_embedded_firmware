use super::driver::{FeetechBusConfig, FeetechBusError};
use lucy_embedded_firmware_serial::SerialChannel;

pub(super) const SYNC_READ: u8 = 0x82;
pub(super) const SYNC_WRITE: u8 = 0x83;

pub(super) enum BuildError {
    PayloadTooLarge,
}

pub(super) struct RequestBuilder<const N: usize> {
    id: u8,
    instruction: u8,
    register: u8,
    sync_length: Option<u8>,
}

impl<const N: usize> RequestBuilder<N> {
    pub(super) fn new() -> Self {
        RequestBuilder {
            id: 0,
            instruction: 0,
            register: 0,
            sync_length: None,
        }
    }

    pub(super) fn set_id(self, id: u8) -> Self {
        RequestBuilder { id, ..self }
    }

    pub(super) fn set_instruction(self, instruction: u8) -> Self {
        RequestBuilder {
            instruction,
            ..self
        }
    }

    pub(super) fn set_register(self, register: u8) -> Self {
        RequestBuilder { register, ..self }
    }

    pub(super) fn set_sync_length(self, length: u8) -> Self {
        RequestBuilder {
            sync_length: Some(length),
            ..self
        }
    }

    pub(super) fn build(self, payload: &[u8]) -> Result<([u8; N], usize), BuildError> {
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

        bus.read(&mut byte)
            .map_err(|_| FeetechBusError::CommunicationError)?;
        if !(prev == 0xFF && byte[0] == 0xFF) {
            prev = byte[0];
            skipped += 1;
            continue;
        }

        let mut id = 0xFFu8;
        while id == 0xFF && skipped <= MAX_SKIPPED_BYTES {
            bus.read(&mut byte)
                .map_err(|_| FeetechBusError::CommunicationError)?;
            id = byte[0];
            skipped += 1;
        }
        bus.read(&mut byte)
            .map_err(|_| FeetechBusError::CommunicationError)?;
        let len = byte[0] as usize;
        prev = 0;

        let mut body = [0u8; 64];
        if len < 2 || len > body.len() {
            skipped += 2;
            continue;
        }
        bus.read(&mut body[..len])
            .map_err(|_| FeetechBusError::CommunicationError)?;

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

pub(super) fn read_replies<S: SerialChannel, const N: usize, const P: usize>(
    bus: &mut S,
    config: &FeetechBusConfig<N>,
    apply: impl FnMut(usize, &[u8; P]),
) -> Result<(), FeetechBusError> {
    let mut seen = [false; N];
    read_replies_tracked(bus, config, &mut seen, apply)
}

/// Like [`read_replies`], but leaves in `seen` which servos answered, also on failure.
pub(super) fn read_replies_tracked<S: SerialChannel, const N: usize, const P: usize>(
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
