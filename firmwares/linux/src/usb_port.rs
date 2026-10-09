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

impl UsbPortConfig {
    pub fn new(target_vid: u16, target_pid: u16, baud_rate: u32) -> Self {
        Self {
            target_vid,
            target_pid,
            baud_rate,
        }
    }

    pub fn open(&self) -> Result<UsbPort, io::Error> {
        let ports = serialport::available_ports().unwrap();

        let matching_port = ports.into_iter().find(|p| {
            if let SerialPortType::UsbPort(UsbPortInfo { vid, pid, .. }) = p.port_type {
                vid == self.target_vid && pid == self.target_pid
            } else {
                false
            }
        });

        match matching_port {
            Some(port_info) => {
                let port = serialport::new(&port_info.port_name, self.baud_rate)
                    .timeout(Duration::from_millis(50))
                    .open()?;
                Ok(UsbPort {
                    config: self.clone(),
                    port
                })
            }
            None => {
                Err(io::Error::new(io::ErrorKind::NotFound, "No matching USB"))
            }
        }

    }
}

pub struct UsbPort {
    pub config: UsbPortConfig,
    pub port: Box<dyn serialport::SerialPort>,
}

impl UsbPort {

}

impl SerialChannel for UsbPort {
    type Error = std::io::Error;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.port.write_all(bytes)?;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let n = self.port.read_exact(buffer)?;
        Ok(buffer.len())
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        self.port.clear(serialport::ClearBuffer::Input)?;
        Ok(())
    }
}
