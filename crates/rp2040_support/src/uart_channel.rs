//! Half-duplex UART channel for bus servos (RP2040 UART0).
//!
//! MCU-generic: DIR high while transmitting, low while listening. Board
//! firmwares own pinmux; the Servo2040 UART-bus feature wires TX/RX to
//! GPIO0/1 (this type) and DIR to an [`OutputPin`] (GPIO2 on that board).

use embedded_hal::digital::OutputPin;
use lucy_embedded_firmware_core::uart::UartChannel;
use rp2040_hal::{
    gpio::{bank0, FunctionUart, Pin, PullDown},
    pac,
    uart::{Enabled, UartPeripheral},
};

pub enum UartError {}

/// UART0 pin pair used by the Servo2040 bus-servo feature (GPIO0 TX, GPIO1 RX).
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
        // Drop any echo / residual RX so the FIFO cannot overflow under bus traffic.
        let mut discard = [0u8; 32];
        while self.uart.uart_is_readable() {
            match self.uart.read_raw(&mut discard) {
                Ok(0) | Err(_) => break,
                Ok(_) => {}
            }
        }
        Ok(())
    }

    fn read(&mut self, _buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}
