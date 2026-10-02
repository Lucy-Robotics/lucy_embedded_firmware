//! Half-duplex UART channel for Feetech/STS bus servos.
//!
//! PoC pinout (Servo2040 / SO-ARM adapter): TX=GPIO0, RX=GPIO1, DIR=GPIO2.
//! DIR high = drive bus; low = listen.

use embedded_hal::digital::OutputPin;
use lucy_embedded_firmware_core::uart::UartChannel;
use rp2040_hal::{
    gpio::{bank0, FunctionUart, Pin, PullDown},
    pac,
    uart::{Enabled, UartPeripheral},
};

pub enum UartError {}

type Uart0Pins = (
    Pin<bank0::Gpio0, FunctionUart, PullDown>,
    Pin<bank0::Gpio1, FunctionUart, PullDown>,
);

pub struct Rp2040UartChannel<D>
where
    D: OutputPin,
{
    pub uart: UartPeripheral<Enabled, pac::UART0, Uart0Pins>,
    pub dir: D,
}

impl<D> UartChannel for Rp2040UartChannel<D>
where
    D: OutputPin,
{
    type Error = UartError;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        let _ = self.dir.set_high();
        self.uart.write_full_blocking(bytes);
        while self.uart.uart_is_busy() {}
        let _ = self.dir.set_low();
        Ok(())
    }

    fn read(&mut self, _buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}
