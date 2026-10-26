use embedded_hal::i2c::I2c;
use lucy_embedded_firmware_pwm::pwm_channel::PwmChannel;

pub enum PwmError {
    I2c,
}

const MODE1: u8 = 0x00;
const MODE2: u8 = 0x01;
const LED0_ON_L: u8 = 0x06;
const ALL_LED_ON_L: u8 = 0xFA;
const PRESCALE: u8 = 0xFE;

/// PCA9685s chained on one I2C bus at 50 Hz (internal 25 MHz oscillator, 4096 steps per 20 ms).
/// Outputs are numbered across chips: 0..15 on `addresses[0]`, 16..31 on `addresses[1]`, ...
pub struct Pca9685Bank<I> {
    pub i2c: I,
    pub addresses: &'static [u8],
}

impl<I: I2c> Pca9685Bank<I> {
    /// Sets the 50 Hz frame and turns every output off, on every chip.
    /// A chip that does not answer does not stop the others; the last error is returned.
    pub fn init(&mut self) -> Result<(), PwmError> {
        let mut result = Ok(());
        for &address in self.addresses {
            if let Err(e) = self.init_chip(address) {
                result = Err(e);
            }
        }
        result
    }

    fn init_chip(&mut self, address: u8) -> Result<(), PwmError> {
        self.write(address, &[MODE1, 0x10])?; // sleep: prescale is only writable while asleep
        self.write(address, &[PRESCALE, 121])?; // round(25 MHz / (4096 * 50 Hz)) - 1
        self.write(address, &[MODE1, 0x20])?; // wake, register auto-increment
        arduino_hal::delay_us(500); // oscillator start-up
        self.write(address, &[MODE2, 0x04])?; // totem-pole outputs
        // All 16 outputs full-off: the PCA keeps old pulses across Mega resets, and
        // nothing should drive until it's commanded.
        self.write(address, &[ALL_LED_ON_L, 0, 0, 0, 0x10])
    }

    /// Bit i set = `addresses[i]` did not answer. A chip found asleep lost power since its
    /// setup (MODE1 resets to sleep), so it is set up again and the next tick's pulses land.
    pub fn check(&mut self) -> u8 {
        let mut missing = 0;
        for (i, &address) in self.addresses.iter().enumerate() {
            let mut mode1 = [0u8];
            match self.i2c.write_read(address, &[MODE1], &mut mode1) {
                Err(_) => missing |= 1 << i,
                Ok(()) if mode1[0] & 0x10 != 0 => {
                    if self.init_chip(address).is_err() {
                        missing |= 1 << i;
                    }
                }
                Ok(()) => {}
            }
        }
        missing
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), PwmError> {
        self.i2c.write(address, bytes).map_err(|_| PwmError::I2c)
    }

    /// Borrows one output for a servo update.
    pub fn output(&mut self, output: u8) -> Pca9685Output<'_, I> {
        Pca9685Output {
            i2c: &mut self.i2c,
            address: self.addresses[(output / 16) as usize],
            channel: output % 16,
        }
    }
}

/// One PCA9685 output on the bank's shared bus.
pub struct Pca9685Output<'a, I> {
    i2c: &'a mut I,
    address: u8,
    channel: u8,
}

impl<I: I2c> PwmChannel for Pca9685Output<'_, I> {
    type Error = PwmError;

    /// `pulse` in microseconds; 0 = full off so the servo goes limp.
    fn set_pwm(&mut self, pulse: u16) -> Result<(), Self::Error> {
        let reg = LED0_ON_L + 4 * self.channel;
        let bytes = if pulse == 0 {
            [reg, 0, 0, 0, 0x10]
        } else {
            let off = (pulse as u32 * 4096 / 20_000).min(4095) as u16;
            [reg, 0, 0, off as u8, (off >> 8) as u8]
        };
        self.i2c.write(self.address, &bytes).map_err(|_| PwmError::I2c)
    }
}
