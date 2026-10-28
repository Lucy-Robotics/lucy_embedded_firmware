//! One I2C0 bus and every PCA9685 channel on it.

use embedded_hal::i2c::I2c;
use lucy_embedded_firmware_core::joint::{Command, TorqueStatus};
use lucy_embedded_firmware_pwm::pca9685::{
    channel_base, channel_bytes, sleep_config_writes, wake_write, WAKE_DELAY_US,
};
use lucy_embedded_firmware_pwm::pulse::pulse_width_us;

use crate::table::{I2cRow, I2C_ROWS};

pub fn rows() -> &'static [I2cRow] {
    I2C_ROWS
}

pub struct I2cBank<I> {
    bus: I,
    last: [u32; 32],
}

impl<I: I2c> I2cBank<I> {
    /// Initialize every distinct PCA9685 address. `delay_us` must wait after
    /// clearing SLEEP (see PCA9685 datasheet oscillator start).
    pub fn new(mut bus: I, mut delay_us: impl FnMut(u32)) -> Self {
        let mut seen = [0u8; 8];
        let mut count = 0usize;
        for row in I2C_ROWS {
            if seen[..count].contains(&row.address) {
                continue;
            }
            if count < seen.len() {
                seen[count] = row.address;
                count += 1;
            }
            for bytes in sleep_config_writes() {
                let _ = bus.write(row.address, &bytes);
            }
            let wake = wake_write();
            let _ = bus.write(row.address, &wake);
            // Oscillator starts when SLEEP clears; wait before first channel write.
            delay_us(WAKE_DELAY_US);
        }
        Self {
            bus,
            last: [u32::MAX; 32],
        }
    }

    pub fn write_rows(&mut self, commands: &[Command]) {
        for row in I2C_ROWS {
            let command = commands.get(row.virtual_pin).copied().unwrap_or_default();
            let torque = command.torque_enabled == TorqueStatus::Enabled;
            let pulse = if torque {
                pulse_width_us(command.position, row.amplitude, row.min_pulse, row.max_pulse)
            } else {
                0
            };
            let signature = if torque { u32::from(pulse) } else { 0x1_0000 };
            if self.last.get(row.virtual_pin).copied() == Some(signature) {
                continue;
            }
            let mut payload = [0u8; 5];
            payload[0] = channel_base(row.channel);
            payload[1..].copy_from_slice(&channel_bytes(pulse, torque));
            // Only cache on success so a dropped write is retried next tick.
            if self.bus.write(row.address, &payload).is_ok() {
                if row.virtual_pin < self.last.len() {
                    self.last[row.virtual_pin] = signature;
                }
            }
        }
    }
}
