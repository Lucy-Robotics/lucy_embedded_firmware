use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use serialport::{SerialPort, SerialPortType, UsbPortInfo};

pub use serialport::ClearBuffer;

use lucy_embedded_firmware_serial::SerialChannel;

const RECONNECT_PERIOD: Duration = Duration::from_millis(500);
const IO_TIMEOUT: Duration = Duration::from_millis(50);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsbPortConfig {
    pub target_vid: u16,
    pub target_pid: u16,
    pub baud_rate: u32,
    pub clear_buffer: ClearBuffer,
}

impl UsbPortConfig {
    pub fn new(target_vid: u16, target_pid: u16, baud_rate: u32) -> Self {
        Self {
            target_vid,
            target_pid,
            baud_rate,
            clear_buffer: ClearBuffer::Input,
        }
    }

    pub fn with_clear_buffer(self, clear_buffer: ClearBuffer) -> Self {
        Self { clear_buffer, ..self }
    }

    fn open(&self) -> io::Result<(String, Box<dyn SerialPort>)> {
        let port_info = serialport::available_ports()
            .map_err(io::Error::other)?
            .into_iter()
            .find(|p| matches!(
                p.port_type,
                SerialPortType::UsbPort(UsbPortInfo { vid, pid, .. })
                    if vid == self.target_vid && pid == self.target_pid
            ))
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "no matching device"))?;

        let port = serialport::new(&port_info.port_name, self.baud_rate)
            .timeout(IO_TIMEOUT)
            .open()?;
        port.clear(ClearBuffer::All)?;
        Ok((port_info.port_name, port))
    }
}

pub struct UsbPort {
    config: UsbPortConfig,
    port: Option<Box<dyn SerialPort>>,
    last_attempt: Option<Instant>,
    last_error: Option<String>,
}

impl UsbPort {
    pub fn new(config: UsbPortConfig) -> Self {
        let mut port = Self { config, port: None, last_attempt: None, last_error: None };
        port.reconnect();
        port
    }

    pub fn connected(&self) -> bool {
        self.port.is_some()
    }

    pub fn read_available(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        let result = self.port()?.read(buffer);
        self.check(result)
    }

    fn port(&mut self) -> io::Result<&mut Box<dyn SerialPort>> {
        if self.port.is_none() && self.last_attempt.is_none_or(|t| t.elapsed() >= RECONNECT_PERIOD) {
            self.reconnect();
        }
        self.port.as_mut().ok_or_else(|| io::Error::from(io::ErrorKind::NotConnected))
    }

    fn reconnect(&mut self) {
        self.last_attempt = Some(Instant::now());
        match self.config.open() {
            Ok((name, port)) => {
                eprintln!("USB {} connected on {name}", self.label());
                self.port = Some(port);
                self.last_error = None;
            }
            Err(e) => {
                let error = e.to_string();
                if self.last_error.as_ref() != Some(&error) {
                    eprintln!("USB {} unavailable: {error}", self.label());
                    self.last_error = Some(error);
                }
            }
        }
    }

    fn check<T>(&mut self, result: io::Result<T>) -> io::Result<T> {
        if let Err(e) = &result {
            let transient = matches!(
                e.kind(),
                io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
            );
            if !transient && self.port.take().is_some() {
                eprintln!("USB {} disconnected: {e}", self.label());
            }
        }
        result
    }

    fn label(&self) -> String {
        format!("{:04x}:{:04x}", self.config.target_vid, self.config.target_pid)
    }
}

impl SerialChannel for UsbPort {
    type Error = io::Error;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        let result = self.port()?.write_all(bytes);
        self.check(result)
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let result = self.port()?.read_exact(buffer);
        self.check(result).map(|()| buffer.len())
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        let clear_buffer = self.config.clear_buffer;
        let result = self.port()?.clear(clear_buffer).map_err(io::Error::from);
        self.check(result)
    }
}
