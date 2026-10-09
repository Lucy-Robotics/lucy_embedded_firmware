use lucy_embedded_firmware_pwm::pwm_channel::PwmChannel;
use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;

use embedded_hal::{
    digital::OutputPin,
    pwm::SetDutyCycle,
};

use rp2040_hal::{
    uart,
    timer::Timer,
};

pub enum PwmError {
    WriteError,
}

pub struct Rp2040PwmChannel<C> 
where
    C: SetDutyCycle,
{
    pub channel: C
}

impl<C> PwmChannel for Rp2040PwmChannel<C>
where
    C: SetDutyCycle,
{
    type Error = PwmError;

    fn set_pwm(&mut self, pulse: u16) -> Result<(), Self::Error> {
        self.channel
            .set_duty_cycle(pulse)
            .map_err(|_| PwmError::WriteError)?;
        Ok(())
    }
}

pub enum UartError {
    WriteError,
    ReadError,
    ReadTimeout,
}

const READ_TIMEOUT_US_PER_BYTE: u64 = 20;
const READ_TIMEOUT_MARGIN_US: u64 = 2_000;

pub struct Rp2040UartChannel<D, C, P>
where
    D: OutputPin,
    C: uart::UartDevice,
    P: uart::ValidUartPinout<C>
    {
    pub dir: D,
    pub uart: uart::UartPeripheral<uart::Enabled, C, P>,
    pub timer: Timer,
}

impl<D, C, P> SerialChannel for Rp2040UartChannel<D, C, P>
where
    D: OutputPin,
    C: uart::UartDevice,
    P: uart::ValidUartPinout<C>
{
    type Error = UartError;

    fn write(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.dir.set_high().map_err(|_| UartError::WriteError)?;
        self.uart.write_full_blocking(buffer);
        while self.uart.uart_is_busy() {}
        self.dir.set_low().map_err(|_| UartError::WriteError)?;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let timeout_us = READ_TIMEOUT_MARGIN_US + READ_TIMEOUT_US_PER_BYTE * buffer.len() as u64;
        let deadline = self.timer.get_counter().ticks() + timeout_us;

        let mut offset = 0;
        while offset < buffer.len() {
            match self.uart.read_raw(&mut buffer[offset..]) {
                Ok(n) => offset += n,
                Err(nb::Error::WouldBlock) => {
                    if self.timer.get_counter().ticks() >= deadline {
                        return Err(UartError::ReadTimeout);
                    }
                }
                Err(nb::Error::Other(_)) => return Err(UartError::ReadError),
            }
        }
        Ok(offset)
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
