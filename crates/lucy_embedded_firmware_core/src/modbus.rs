use modbus_core::Request;
use crc::{Crc, CRC_16_MODBUS};

use core::{
    cell::Cell,
    result::Result::{self, Err, Ok},
    option::Option::{self, None, Some},
};

const MODBUS_CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

/// Maximum RTU ADU size (slave + PDU + CRC).
pub const MAX_ADU_LEN: usize = 256;

/// Exception codes (Modbus Application Protocol).
pub mod exception {
    pub const ILLEGAL_FUNCTION: u8 = 0x01;
    pub const ILLEGAL_DATA_ADDRESS: u8 = 0x02;
    pub const ILLEGAL_DATA_VALUE: u8 = 0x03;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModbusError {
    InvalidAddress,
    InvalidFrame,
    CrcError,
    UnknownOpcode,
    IllegalFunction,
    IllegalDataAddress,
    BufferTooSmall,
}

pub trait ModbusAdapter {
    fn tick(&mut self, rv: &RegisterView);
    fn get_nb_register(&self) -> u16;
    fn get_base_register(&self) -> u16;
}

pub struct ModbusRedirect {
    pub source: u16,
    pub destination: u16,
}

impl ModbusRedirect {}

/* WIP descriptor API */

pub enum Access {
    ReadOnly,
    WriteOnly,
    ReadWrite,
}

pub struct RegisterDescriptor<Ctx> {
    pub address: u16,
    pub read: Option<fn(&Ctx) -> Result<u16, ModbusError>>,
    pub write: Option<fn(&mut Ctx, u16) -> Result<(), ModbusError>>,
}

impl<Ctx> RegisterDescriptor<Ctx> {
    pub fn read(&self, ctx: &Ctx) -> Result<u16, ModbusError> {
        match self.read {
            Some(func) => func(ctx),
            None => Err(ModbusError::IllegalFunction),
        }
    }

    pub fn write(&self, ctx: &mut Ctx, value: u16) -> Result<(), ModbusError> {
        match self.write {
            Some(func) => func(ctx, value),
            None => Err(ModbusError::IllegalFunction),
        }
    }
}

pub struct RegisterTable {
    pub registers: [Cell<u16>; 0xFF],
}

impl RegisterTable {
    pub const fn new() -> Self {
        Self {
            registers: [const { Cell::new(0) }; 0xFF],
        }
    }
}

impl Default for RegisterTable {
    fn default() -> Self {
        Self::new()
    }
}

pub struct RegisterView<'a> {
    pub table: &'a RegisterTable,
    pub base_register: u16,
    pub nb_register: u16,
}

impl<'a> RegisterView<'a> {
    pub fn new(table: &'a RegisterTable, base_register: u16, nb_register: u16) -> Self {
        RegisterView {
            table,
            base_register,
            nb_register,
        }
    }

    pub fn read_register(&self, index: u16) -> u16 {
        // index is 0-based within the adapter window; valid range is [0, nb_register)
        if index >= self.nb_register {
            return 0;
        }
        let reg = self.base_register + index;
        if (reg as usize) < self.table.registers.len() {
            self.table.registers[reg as usize].get()
        } else {
            0
        }
    }

    // &self is enough: RegisterTable uses Cell interior mutability.
    pub fn write_register(&self, index: u16, value: u16) {
        if index >= self.nb_register {
            return;
        }
        let reg = self.base_register + index;
        if (reg as usize) < self.table.registers.len() {
            self.table.registers[reg as usize].set(value);
        }
    }
}

pub struct Slave {
    pub address: u8,
}

/// Inter-frame silence gap in microseconds for Modbus RTU (3.5 character times).
///
/// Spec (Modbus over serial line v1.02 §2.5.1.1): character time uses 11 bits.
/// For baud rates > 19200, implementations commonly use a fixed 1750 µs floor.
pub fn inter_frame_delay_us(baud: u32) -> u64 {
    if baud == 0 {
        return 1750;
    }
    if baud > 19_200 {
        return 1750;
    }
    // t3.5 = 3.5 * 11 / baud seconds → microseconds
    // = (35 * 11 * 1_000_000) / (10 * baud)  with ceiling
    let numer = 35u64 * 11 * 1_000_000;
    let denom = 10u64 * u64::from(baud);
    (numer + denom - 1) / denom
}

pub fn check_crc(frame: &[u8]) -> bool {
    if frame.len() < 4 {
        return false;
    }
    let len = frame.len();
    let payload = &frame[..len - 2];
    let crc = MODBUS_CRC.checksum(payload);
    let crc_received = u16::from_le_bytes([frame[len - 2], frame[len - 1]]);
    crc == crc_received
}

fn append_crc(buf: &mut [u8], len: usize) -> Result<usize, ModbusError> {
    if len + 2 > buf.len() {
        return Err(ModbusError::BufferTooSmall);
    }
    let crc = MODBUS_CRC.checksum(&buf[..len]);
    let bytes = crc.to_le_bytes();
    buf[len] = bytes[0];
    buf[len + 1] = bytes[1];
    Ok(len + 2)
}

/// Build a framed exception response into `out`. Returns total ADU length.
pub fn build_exception_response(
    slave_addr: u8,
    function: u8,
    exception_code: u8,
    out: &mut [u8],
) -> Result<usize, ModbusError> {
    if out.len() < 5 {
        return Err(ModbusError::BufferTooSmall);
    }
    out[0] = slave_addr;
    out[1] = function | 0x80;
    out[2] = exception_code;
    append_crc(out, 3)
}

fn function_code(request: &Request<'_>) -> u8 {
    match request {
        Request::ReadCoils(_, _) => 0x01,
        Request::ReadDiscreteInputs(_, _) => 0x02,
        Request::ReadHoldingRegisters(_, _) => 0x03,
        Request::ReadInputRegisters(_, _) => 0x04,
        Request::WriteSingleCoil(_, _) => 0x05,
        Request::WriteSingleRegister(_, _) => 0x06,
        Request::WriteMultipleCoils(_, _) => 0x0F,
        Request::WriteMultipleRegisters(_, _) => 0x10,
        _ => 0x00,
    }
}

pub fn parse_modbus_frame<'a>(
    slave: &'a Slave,
    frame: &'a [u8],
) -> Result<Request<'a>, ModbusError> {
    let len = frame.len();

    if len < 4 {
        return Err(ModbusError::InvalidFrame);
    }
    if frame[0] != slave.address {
        return Err(ModbusError::InvalidAddress);
    }
    if !check_crc(frame) {
        return Err(ModbusError::CrcError);
    }

    let pdu = &frame[1..len - 2];

    match Request::try_from(pdu) {
        Ok(request) => Ok(request),
        Err(_) => Err(ModbusError::UnknownOpcode),
    }
}

/// Route a request, mutate the register table, and write a framed RTU response into `out`.
/// Returns the number of response bytes written.
pub fn route_modbus_request(
    slave_addr: u8,
    register_table: &RegisterTable,
    request: Request<'_>,
    out: &mut [u8],
) -> Result<usize, ModbusError> {
    let fc = function_code(&request);

    match request {
        Request::ReadHoldingRegisters(addr, quantity) => {
            if quantity == 0 || quantity > 125 {
                return build_exception_response(
                    slave_addr,
                    fc,
                    exception::ILLEGAL_DATA_VALUE,
                    out,
                );
            }
            let start = addr as usize;
            let end = start + quantity as usize;
            if end > register_table.registers.len() {
                return build_exception_response(
                    slave_addr,
                    fc,
                    exception::ILLEGAL_DATA_ADDRESS,
                    out,
                );
            }

            let byte_count = (quantity * 2) as u8;
            // addr + fc + byte_count + data + crc
            let needed = 3 + byte_count as usize + 2;
            if out.len() < needed {
                return Err(ModbusError::BufferTooSmall);
            }
            out[0] = slave_addr;
            out[1] = 0x03;
            out[2] = byte_count;
            for i in 0..quantity as usize {
                let value = register_table.registers[start + i].get();
                let be = value.to_be_bytes();
                out[3 + i * 2] = be[0];
                out[3 + i * 2 + 1] = be[1];
            }
            append_crc(out, 3 + byte_count as usize)
        }
        Request::WriteSingleRegister(addr, value) => {
            let index = addr as usize;
            if index >= register_table.registers.len() {
                return build_exception_response(
                    slave_addr,
                    fc,
                    exception::ILLEGAL_DATA_ADDRESS,
                    out,
                );
            }
            register_table.registers[index].set(value);
            // Echo request: addr + fc + reg + value + crc
            if out.len() < 8 {
                return Err(ModbusError::BufferTooSmall);
            }
            out[0] = slave_addr;
            out[1] = 0x06;
            let addr_be = addr.to_be_bytes();
            let val_be = value.to_be_bytes();
            out[2] = addr_be[0];
            out[3] = addr_be[1];
            out[4] = val_be[0];
            out[5] = val_be[1];
            append_crc(out, 6)
        }
        Request::WriteMultipleRegisters(addr, data) => {
            let start = addr as usize;
            let end = start + data.len();
            if data.is_empty() || end > register_table.registers.len() {
                return build_exception_response(
                    slave_addr,
                    fc,
                    exception::ILLEGAL_DATA_ADDRESS,
                    out,
                );
            }
            for i in 0..data.len() {
                if let Some(value) = data.get(i) {
                    register_table.registers[start + i].set(value);
                }
            }
            // addr + fc + start + quantity + crc
            if out.len() < 8 {
                return Err(ModbusError::BufferTooSmall);
            }
            let quantity = data.len() as u16;
            out[0] = slave_addr;
            out[1] = 0x10;
            let addr_be = addr.to_be_bytes();
            let qty_be = quantity.to_be_bytes();
            out[2] = addr_be[0];
            out[3] = addr_be[1];
            out[4] = qty_be[0];
            out[5] = qty_be[1];
            append_crc(out, 6)
        }
        _ => build_exception_response(slave_addr, fc, exception::ILLEGAL_FUNCTION, out),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame_write_single(slave: u8, addr: u16, value: u16) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0] = slave;
        buf[1] = 0x06;
        let a = addr.to_be_bytes();
        let v = value.to_be_bytes();
        buf[2] = a[0];
        buf[3] = a[1];
        buf[4] = v[0];
        buf[5] = v[1];
        let crc = MODBUS_CRC.checksum(&buf[..6]);
        let c = crc.to_le_bytes();
        buf[6] = c[0];
        buf[7] = c[1];
        buf
    }

    #[test]
    fn inter_frame_delay_high_baud_uses_floor() {
        assert_eq!(inter_frame_delay_us(115_200), 1750);
        assert_eq!(inter_frame_delay_us(38_400), 1750);
    }

    #[test]
    fn inter_frame_delay_low_baud_is_3_5_chars() {
        // 9600: 3.5 * 11 / 9600 * 1e6 ≈ 4010.4 → 4011
        assert_eq!(inter_frame_delay_us(9600), 4011);
    }

    #[test]
    fn write_single_echoes_and_stores() {
        let table = RegisterTable::new();
        let req_frame = frame_write_single(1, 2, 0x1234);
        let slave = Slave { address: 1 };
        let request = parse_modbus_frame(&slave, &req_frame).unwrap();
        let mut out = [0u8; 16];
        let n = route_modbus_request(1, &table, request, &mut out).unwrap();
        assert_eq!(n, 8);
        assert_eq!(&out[..8], &req_frame);
        assert_eq!(table.registers[2].get(), 0x1234);
    }

    #[test]
    fn read_holding_registers_returns_values() {
        let table = RegisterTable::new();
        table.registers[0].set(0x0001);
        table.registers[1].set(0xABCD);

        // Build FC03 request: addr=0, qty=2
        let mut req = [0u8; 8];
        req[0] = 1;
        req[1] = 0x03;
        req[2] = 0x00;
        req[3] = 0x00;
        req[4] = 0x00;
        req[5] = 0x02;
        let crc = MODBUS_CRC.checksum(&req[..6]);
        let c = crc.to_le_bytes();
        req[6] = c[0];
        req[7] = c[1];

        let slave = Slave { address: 1 };
        let request = parse_modbus_frame(&slave, &req).unwrap();
        let mut out = [0u8; 32];
        let n = route_modbus_request(1, &table, request, &mut out).unwrap();
        // 1+1+1+4+2 = 9
        assert_eq!(n, 9);
        assert_eq!(out[0], 1);
        assert_eq!(out[1], 0x03);
        assert_eq!(out[2], 4);
        assert_eq!(out[3], 0x00);
        assert_eq!(out[4], 0x01);
        assert_eq!(out[5], 0xAB);
        assert_eq!(out[6], 0xCD);
        assert!(check_crc(&out[..n]));
    }

    #[test]
    fn register_view_rejects_index_equal_to_nb() {
        let table = RegisterTable::new();
        table.registers[0].set(42);
        let view = RegisterView::new(&table, 0, 1);
        assert_eq!(view.read_register(0), 42);
        assert_eq!(view.read_register(1), 0);
    }
}
