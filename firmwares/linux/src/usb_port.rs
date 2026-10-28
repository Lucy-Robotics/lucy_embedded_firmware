use std::io::{Read, Write};
use std::io;
use std::time::Duration;
use serialport::{SerialPortType, UsbPortInfo};

use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct UsbPortConfig {
    pub target_vid: u16,
    pub target_pid: u16,
    pub baud_rate: u32,
}

/// What the person should do for this open failure. The process has already
/// exited, so every fix ends with a relaunch.
pub fn user_fix(kind: io::ErrorKind, device: &str) -> String {
    match kind {
        io::ErrorKind::NotFound => {
            format!("Disconnect and reconnect {device}. Then relaunch Lucy.")
        }
        io::ErrorKind::PermissionDenied => {
            format!("Lucy is not allowed to use {device}. Grant access. Then relaunch Lucy.")
        }
        io::ErrorKind::ResourceBusy | io::ErrorKind::AddrInUse => {
            format!("{device} is in use by another program. Close that program. Then relaunch Lucy.")
        }
        _ => format!("Lucy could not open {device}. Relaunch Lucy."),
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

    pub fn open(&self) -> Result<UsbPort, io::Error> {
        let ports = serialport::available_ports()?;
        let port_info = self.select(ports)?;
        let port = serialport::new(&port_info.port_name, self.baud_rate)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|err| annotate_open(&port_info.port_name, err))?;
        Ok(UsbPort { port })
    }

    fn select(
        &self,
        ports: Vec<serialport::SerialPortInfo>,
    ) -> Result<serialport::SerialPortInfo, io::Error> {
        ports
            .into_iter()
            .find(|port| match &port.port_type {
                SerialPortType::UsbPort(UsbPortInfo { vid, pid, .. }) => {
                    *vid == self.target_vid && *pid == self.target_pid
                }
                _ => false,
            })
            .ok_or_else(|| {
                io::Error::new(io::ErrorKind::NotFound, "no matching USB")
            })
    }
}

fn annotate_open(port_name: &str, err: serialport::Error) -> io::Error {
    let io_err = io::Error::from(err);
    io::Error::new(io_err.kind(), format!("{port_name}: {io_err}"))
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
    use super::UsbPortConfig;
    use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo};

    fn usb(name: &str, vid: u16, pid: u16) -> SerialPortInfo {
        SerialPortInfo {
            port_name: name.into(),
            port_type: SerialPortType::UsbPort(UsbPortInfo {
                vid,
                pid,
                serial_number: None,
                manufacturer: None,
                product: None,
            }),
        }
    }

    #[test]
    fn missing_adapter_is_not_found() {
        let cfg = UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000);
        let err = cfg
            .select(vec![usb("/dev/ttyUSB0", 0x1234, 0x5678)])
            .expect_err("different vid/pid");
        assert_eq!(err.kind(), std::io::ErrorKind::NotFound);
        assert_eq!(
            super::user_fix(err.kind(), "SO-ARM"),
            "Disconnect and reconnect SO-ARM. Then relaunch Lucy."
        );
    }

    #[test]
    fn permission_and_busy_name_a_different_fix() {
        assert_eq!(
            super::user_fix(std::io::ErrorKind::PermissionDenied, "SO-ARM"),
            "Lucy is not allowed to use SO-ARM. Grant access. Then relaunch Lucy."
        );
        assert_eq!(
            super::user_fix(std::io::ErrorKind::ResourceBusy, "SO-ARM"),
            "SO-ARM is in use by another program. Close that program. Then relaunch Lucy."
        );
    }

    #[test]
    fn selects_matching_vid_pid() {
        let cfg = UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000);
        let found = cfg
            .select(vec![
                usb("/dev/ttyUSB0", 0x1234, 0x5678),
                usb("/dev/ttyACM0", 0x1a86, 0x55d3),
            ])
            .expect("matching adapter");
        assert_eq!(found.port_name, "/dev/ttyACM0");
    }
}
