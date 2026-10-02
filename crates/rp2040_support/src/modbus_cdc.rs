//! Modbus RTU-over-USB-CDC poll helper shared by board firmwares.

use lucy_embedded_firmware_core::modbus::{
    inter_frame_delay_us, parse_modbus_frame, route_modbus_request, ModbusError, RegisterTable,
    Slave,
};
use usb_device::class_prelude::UsbBus;
use usb_device::prelude::UsbDevice;
use usbd_serial::SerialPort;

use crate::picotool_reset::PicoToolReset;

/// RX framing state for Modbus RTU over CDC.
pub struct ModbusCdcState {
    pub rx_buf: [u8; 256],
    pub tx_buf: [u8; 256],
    pub rx_len: usize,
    pub rx_active_timer: bool,
    pub last_rx_micros: u64,
    pub frame_gap_us: u64,
}

impl ModbusCdcState {
    pub fn new(baud: u32) -> Self {
        Self {
            rx_buf: [0u8; 256],
            tx_buf: [0u8; 256],
            rx_len: 0,
            rx_active_timer: false,
            last_rx_micros: 0,
            frame_gap_us: inter_frame_delay_us(baud),
        }
    }

    /// Poll USB, accumulate CDC bytes into the RX buffer.
    pub fn poll_usb<B: UsbBus>(
        &mut self,
        usb_dev: &mut UsbDevice<'_, B>,
        serial: &mut SerialPort<'_, B>,
        picotool: &mut PicoToolReset<'_, B>,
        now: u64,
    ) {
        if usb_dev.poll(&mut [serial, picotool]) {
            let mut tmp_buf = [0u8; 64];
            while let Ok(count) = serial.read(&mut tmp_buf) {
                if count == 0 {
                    break;
                }
                if self.rx_len + count <= self.rx_buf.len() {
                    self.rx_buf[self.rx_len..self.rx_len + count]
                        .copy_from_slice(&tmp_buf[..count]);
                    self.rx_len += count;
                    self.last_rx_micros = now;
                    self.rx_active_timer = true;
                } else {
                    self.rx_active_timer = false;
                    self.rx_len = 0;
                    break;
                }
            }
        }
    }

    /// After inter-frame gap, parse and route one Modbus request.
    pub fn try_route_frame<B: UsbBus>(
        &mut self,
        slave: &Slave,
        rt: &RegisterTable,
        serial: &mut SerialPort<'_, B>,
        now: u64,
    ) {
        if !(self.rx_active_timer && (now.wrapping_sub(self.last_rx_micros) >= self.frame_gap_us))
        {
            return;
        }
        self.rx_active_timer = false;
        if self.rx_len >= 4 {
            match parse_modbus_frame(slave, &self.rx_buf[..self.rx_len]) {
                Ok(request) => {
                    if let Ok(n) =
                        route_modbus_request(slave.address, rt, request, &mut self.tx_buf)
                    {
                        let _ = serial.write(&self.tx_buf[..n]);
                    }
                }
                Err(ModbusError::InvalidAddress) => {}
                Err(_) => {}
            }
        }
        self.rx_len = 0;
    }
}
