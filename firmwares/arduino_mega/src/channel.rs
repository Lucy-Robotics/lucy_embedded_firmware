use lucy_embedded_firmware_pwm::pwm_channel::PwmChannel;

pub enum PwmError {
    #[cfg(feature = "nervo")]
    I2c,
}

/// Servo straight on a Mega pin: Timer4 channel A (OC4A = D6), 50 Hz, 0.5 us per tick.
#[cfg(not(feature = "nervo"))]
pub struct MegaPwmChannel {
    pub tc4: arduino_hal::pac::TC4,
}

#[cfg(not(feature = "nervo"))]
impl MegaPwmChannel {
    /// Starts the timer with the output disconnected: no pulses until `set_pwm`.
    pub fn new(tc4: arduino_hal::pac::TC4) -> Self {
        // Fast PWM, TOP = ICR4 (WGM4 = 0b1110).
        tc4.tccr4a().write(|w| w.wgm4().set(0b10).com4a().disconnected());
        tc4.icr4().write(|w| w.set(39_999));
        tc4.tccr4b().write(|w| w.wgm4().set(0b11).cs4().prescale_8());
        Self { tc4 }
    }
}

#[cfg(not(feature = "nervo"))]
impl PwmChannel for MegaPwmChannel {
    type Error = PwmError;

    /// `pulse` in microseconds; 0 stops pulses so the servo goes limp.
    fn set_pwm(&mut self, pulse: u16) -> Result<(), Self::Error> {
        if pulse == 0 {
            self.tc4.tccr4a().modify(|_, w| w.com4a().disconnected());
            return Ok(());
        }
        self.tc4.ocr4a().write(|w| w.set(pulse.saturating_mul(2).min(39_999)));
        self.tc4.tccr4a().modify(|_, w| w.com4a().match_clear());
        Ok(())
    }
}

#[cfg(feature = "nervo")]
mod pca9685 {
    use super::{PwmChannel, PwmError};
    use embedded_hal::i2c::I2c;

    const MODE1: u8 = 0x00;
    const MODE2: u8 = 0x01;
    const LED0_ON_L: u8 = 0x06;
    const ALL_LED_ON_L: u8 = 0xFA;
    const PRESCALE: u8 = 0xFE;

    /// One PCA9685 output at 50 Hz (internal 25 MHz oscillator, 4096 steps per 20 ms).
    pub struct Pca9685Channel<I> {
        pub i2c: I,
        pub address: u8,
        pub channel: u8,
    }

    impl<I: I2c> Pca9685Channel<I> {
        /// Sets the 50 Hz frame and turns every output off.
        pub fn init(&mut self) -> Result<(), PwmError> {
            self.write(&[MODE1, 0x10])?; // sleep: prescale is only writable while asleep
            self.write(&[PRESCALE, 121])?; // round(25 MHz / (4096 * 50 Hz)) - 1
            self.write(&[MODE1, 0x20])?; // wake, register auto-increment
            arduino_hal::delay_us(500); // oscillator start-up
            self.write(&[MODE2, 0x04])?; // totem-pole outputs
            // All 16 outputs full-off: the PCA keeps old pulses across Mega resets, and
            // nothing should drive until it's commanded.
            self.write(&[ALL_LED_ON_L, 0, 0, 0, 0x10])
        }

        fn write(&mut self, bytes: &[u8]) -> Result<(), PwmError> {
            self.i2c.write(self.address, bytes).map_err(|_| PwmError::I2c)
        }
    }

    impl<I: I2c> PwmChannel for Pca9685Channel<I> {
        type Error = PwmError;

        /// `pulse` in microseconds; 0 = full off so the servo goes limp.
        fn set_pwm(&mut self, pulse: u16) -> Result<(), Self::Error> {
            let reg = LED0_ON_L + 4 * self.channel;
            if pulse == 0 {
                return self.write(&[reg, 0, 0, 0, 0x10]);
            }
            let off = (pulse as u32 * 4096 / 20_000).min(4095) as u16;
            self.write(&[reg, 0, 0, off as u8, (off >> 8) as u8])
        }
    }
}

#[cfg(feature = "nervo")]
pub use pca9685::Pca9685Channel;
