//! PCA9685 register protocol. No RP2040 HAL: the caller writes the bytes.

pub const MODE1: u8 = 0x00;
pub const MODE2: u8 = 0x01;
pub const PRE_SCALE: u8 = 0xFE;
/// `round(25e6 / (4096 * 50)) - 1`.
pub const PRE_SCALE_50HZ: u8 = 121;
pub const MODE1_SLEEP: u8 = 0x10;
pub const MODE1_AI: u8 = 0x20;
pub const MODE2_TOTEM: u8 = 0x04;
pub const LED0_ON_L: u8 = 0x06;
/// Datasheet: wait after clearing SLEEP before the oscillator is ready.
/// Call this delay after ``wake_write()``, not before.
pub const WAKE_DELAY_US: u32 = 500;

/// Writes that must run while the chip is asleep (SLEEP, PRE_SCALE, MODE2).
pub fn sleep_config_writes() -> [[u8; 2]; 3] {
    [
        [MODE1, MODE1_SLEEP],
        [PRE_SCALE, PRE_SCALE_50HZ],
        [MODE2, MODE2_TOTEM],
    ]
}

/// MODE1 value that clears SLEEP and enables auto-increment.
pub fn wake_write() -> [u8; 2] {
    [MODE1, MODE1_AI]
}

/// Full init sequence for host tests. Firmware must delay after index 2.
pub fn init_writes() -> [[u8; 2]; 4] {
    [
        [MODE1, MODE1_SLEEP],
        [PRE_SCALE, PRE_SCALE_50HZ],
        [MODE2, MODE2_TOTEM],
        [MODE1, MODE1_AI],
    ]
}

pub fn channel_base(channel: u8) -> u8 {
    LED0_ON_L + 4 * channel
}

/// OFF count for a pulse at 50 Hz over 4096 steps of a 20 ms period.
pub fn off_count(pulse_us: u16) -> u16 {
    (u32::from(pulse_us) * 4096 / 20_000) as u16
}

/// Four LED registers. Torque-off sets the full-OFF bit and leaves the count at 0.
pub fn channel_bytes(pulse_us: u16, torque_on: bool) -> [u8; 4] {
    if !torque_on {
        return [0, 0, 0, 0x10];
    }
    let off = off_count(pulse_us);
    [0, 0, (off & 0xff) as u8, (off >> 8) as u8]
}

#[cfg(test)]
mod tests {
    use super::{
        channel_base, channel_bytes, init_writes, off_count, sleep_config_writes, wake_write,
        PRE_SCALE_50HZ, WAKE_DELAY_US,
    };

    #[test]
    fn init_sleeps_before_prescale_then_wakes_totem() {
        let writes = init_writes();
        assert_eq!(writes[0], [0x00, 0x10]);
        assert_eq!(writes[1], [0xFE, PRE_SCALE_50HZ]);
        assert_eq!(writes[2], [0x01, 0x04]);
        assert_eq!(writes[3], [0x00, 0x20]);
        assert_eq!(sleep_config_writes().len(), 3);
        assert_eq!(wake_write(), [0x00, 0x20]);
        assert_eq!(WAKE_DELAY_US, 500);
    }

    #[test]
    fn channel_and_off_count() {
        assert_eq!(channel_base(0), 0x06);
        assert_eq!(channel_base(5), 0x06 + 20);
        assert_eq!(off_count(1500), (1500u32 * 4096 / 20_000) as u16);
        assert_eq!(channel_bytes(1500, false), [0, 0, 0, 0x10]);
        let on = channel_bytes(1500, true);
        assert_eq!(on[0], 0);
        assert_eq!(on[1], 0);
        assert_ne!(on[3] & 0x10, 0x10);
    }
}
