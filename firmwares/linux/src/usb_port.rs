use std::io::{Read, Write};
use std::io;
use std::fs::File;
use std::fmt;
use std::time::Duration;
use core::cell::Cell;
use serialport::{SerialPortType, UsbPortInfo};

use lucy_embedded_firmware_core::joint::{JointCommand, JointState, JointConfig};
use lucy_embedded_firmware_core::actuator::{Actuator};

use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use lucy_embedded_firmware_feetech::link::{*};

use memmap2::MmapMut;
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, TryFromBytes};

use libc;
use std::ffi::CString;
use std::os::fd::FromRawFd;

pub struct UsbPort {
    pub target_pid: u16,
    pub target_vid: u16,
    pub baud_rate: u32,
    pub port: Option<Box<dyn serialport::SerialPort>>,
}

impl UsbPort {
    pub const fn new(target_pid: u16, target_vid: u16, baud_rate: u32) -> Self {
        Self {
            target_pid,
            target_vid,
            baud_rate,
            port: None,
        }
    }
}

impl SerialChannel for UsbPort {
    type Error = std::io::Error;

    fn open(&mut self) -> Result<(), Self::Error> {
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
                self.port = Some(port);
                Ok(())
            }
            None => {
                Err(io::Error::new(io::ErrorKind::NotFound, "No matching USB"))
            }
        }
    }

    fn close(&mut self) -> Result<(), Self::Error> {
        self.port = None;
        Ok(())
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        if let Some(port) = &mut self.port {
            port.write_all(bytes)?;
        }
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        if let Some(port) = &mut self.port {
            let n = port.read_exact(buffer)?;
            Ok(buffer.len())
        } else {
            println!("Error");
            Ok(0)
        }
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        if let Some(port) = &mut self.port {
            port.clear(serialport::ClearBuffer::All)?;
        }
        Ok(())
    }
}
