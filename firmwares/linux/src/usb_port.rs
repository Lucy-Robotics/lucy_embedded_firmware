use serialport::{SerialPortType, UsbPortInfo};
use std::io;
use std::io::{Read, Write};
use std::time::Duration;

use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct UsbPortConfig {
    pub target_vid: u16,
    pub target_pid: u16,
    pub baud_rate: u32,
}

/// The VID/PID/baud rate of the USB-to-serial adapter this robot ships with.
pub(crate) fn lucy_usb_config() -> UsbPortConfig {
    UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000)
}

/// A connected USB serial adapter with the configured VID/PID.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Adapter {
    pub port_name: String,
    /// USB iSerial, absent on some clone adapters.
    pub serial: Option<String>,
}

/// Which adapter to drive, given what is connected and what was asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdapterChoice<'a> {
    One(&'a Adapter),
    None,
    /// Several adapters and no selection. Guessing could drive the wrong arm.
    Ambiguous,
    SelectedMissing,
}

pub fn choose_adapter<'a>(adapters: &'a [Adapter], selected: Option<&str>) -> AdapterChoice<'a> {
    if let Some(wanted) = selected {
        return match adapters
            .iter()
            .find(|a| a.serial.as_deref() == Some(wanted))
        {
            Some(adapter) => AdapterChoice::One(adapter),
            None => AdapterChoice::SelectedMissing,
        };
    }
    match adapters {
        [] => AdapterChoice::None,
        [only] => AdapterChoice::One(only),
        _ => AdapterChoice::Ambiguous,
    }
}

impl UsbPortConfig {
    pub fn new(target_vid: u16, target_pid: u16, baud_rate: u32) -> Self {
        Self {
            target_vid,
            target_pid,
            baud_rate,
        }
    }

    /// Connected adapters with this VID/PID. Empty if the ports cannot be listed.
    pub fn list_adapters(&self) -> Vec<Adapter> {
        let ports = serialport::available_ports().unwrap_or_default();
        ports
            .into_iter()
            .filter_map(|port| match port.port_type {
                SerialPortType::UsbPort(UsbPortInfo {
                    vid,
                    pid,
                    serial_number,
                    ..
                }) if vid == self.target_vid && pid == self.target_pid => Some(Adapter {
                    port_name: port.port_name,
                    serial: serial_number,
                }),
                _ => None,
            })
            .collect()
    }

    pub fn open(&self, adapter: &Adapter) -> Result<UsbPort, io::Error> {
        let port = serialport::new(&adapter.port_name, self.baud_rate)
            .timeout(Duration::from_millis(50))
            .open()?;
        Ok(UsbPort { port })
    }
}

pub struct UsbPort {
    pub port: Box<dyn serialport::SerialPort>,
}

impl SerialChannel for UsbPort {
    type Error = std::io::Error;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.port.write_all(bytes)?;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        self.port.read_exact(buffer)?;
        Ok(buffer.len())
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        self.port.clear(serialport::ClearBuffer::Input)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adapter(port: &str, serial: Option<&str>) -> Adapter {
        Adapter {
            port_name: port.to_string(),
            serial: serial.map(str::to_string),
        }
    }

    #[test]
    fn single_adapter_needs_no_selection() {
        let adapters = [adapter("/dev/ttyACM0", Some("A"))];
        assert_eq!(
            choose_adapter(&adapters, None),
            AdapterChoice::One(&adapters[0])
        );
    }

    #[test]
    fn single_adapter_without_iserial_still_opens() {
        let adapters = [adapter("/dev/ttyACM0", None)];
        assert_eq!(
            choose_adapter(&adapters, None),
            AdapterChoice::One(&adapters[0])
        );
    }

    #[test]
    fn no_adapter() {
        assert_eq!(choose_adapter(&[], None), AdapterChoice::None);
    }

    #[test]
    fn several_adapters_are_never_guessed() {
        let adapters = [
            adapter("/dev/ttyACM0", Some("A")),
            adapter("/dev/ttyACM1", Some("B")),
        ];
        assert_eq!(choose_adapter(&adapters, None), AdapterChoice::Ambiguous);
    }

    #[test]
    fn selection_picks_among_several() {
        let adapters = [
            adapter("/dev/ttyACM0", Some("A")),
            adapter("/dev/ttyACM1", Some("B")),
        ];
        assert_eq!(
            choose_adapter(&adapters, Some("B")),
            AdapterChoice::One(&adapters[1])
        );
    }

    #[test]
    fn selected_adapter_gone_does_not_fall_back_to_another_arm() {
        let adapters = [adapter("/dev/ttyACM0", Some("A"))];
        assert_eq!(
            choose_adapter(&adapters, Some("B")),
            AdapterChoice::SelectedMissing
        );
    }
}
