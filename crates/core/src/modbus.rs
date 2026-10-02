use modbus_core::{Request, Response};
use crc::{Crc, CRC_16_MODBUS};

use core::{
    cell::Cell,
    error::Error,
    result::Result::{self, Err, Ok},
    option::Option::{self, None, Some},
};

const MODBUS_CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);


/* Trait definition */

pub enum ModbusError {
    InvalidAddress,
    InvalidFrame,
    CrcError,
    UnknownOpcode,
    IllegalFunction
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

impl ModbusRedirect {

}

/* WIP */

pub enum Access {
    ReadOnly,
    WriteOnly,
    ReadWrite
}

pub struct RegisterDescriptor<Ctx> {
    pub address: u16,
    pub read: Option<fn(&Ctx) -> Result<u16, ModbusError>>,
    pub write: Option<fn(&mut Ctx, u16) -> Result<(), ModbusError>>
}

impl<Ctx> RegisterDescriptor<Ctx> {
    pub fn read(&self, ctx: &Ctx) -> Result<u16, ModbusError> {
        match self.read {
            Some(func) => func(ctx),
            None => Err(ModbusError::IllegalFunction)
        }
    }
        
    pub fn write(&self, ctx: &mut Ctx, value: u16) -> Result<(), ModbusError> {
        match self.write {
            Some(func) => func(ctx, value),
            None => Err(ModbusError::IllegalFunction)
        }
    }
}

/* Registers structure definition */

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
        RegisterView { table, base_register, nb_register }
    }

    pub fn read_register(&self, index: u16) -> u16 {
        if index > self.nb_register {
            return 0;
        }
        let reg = self.base_register + index;
        if reg < self.table.registers.len() as u16 {
            self.table.registers[reg as usize].get()
        } else {
            0
        }
    }

    pub fn write_register(&self, index: u16, value: u16) {
        if index > self.nb_register {
            return;
        }
        let reg = self.base_register + index;
        if reg < self.table.registers.len() as u16 {
            self.table.registers[reg as usize].set(value);
        }
    }
}



pub struct Slave {
    pub address: u8,
}

pub fn check_crc(frame: &[u8]) -> bool {
    let len = frame.len();
    let payload = &frame[..len - 2];
    let crc = MODBUS_CRC.checksum(payload);
    let crc_received = u16::from_le_bytes([frame[len - 2], frame[len - 1]]);

    crc == crc_received
}

pub fn parse_modbus_frame<'a>(slave: &'a Slave, frame: &'a[u8]) -> Result<Request<'a>, ModbusError> {
    let len = frame.len();

    if len < 4 {
        return Err(ModbusError::InvalidFrame);
    }
    if frame[0] != slave.address {
        return Err(ModbusError::InvalidAddress);
    }
    if !check_crc(&frame) {
        return Err(ModbusError::CrcError);
    }

    let pdu = &frame[1..len - 2];

    match Request::try_from(pdu) {
        Ok(request) => Ok(request),
        Err(_) => Err(ModbusError::UnknownOpcode)
    }
}

pub fn route_modbus_request(register_table: &RegisterTable, request: Request<'_>) -> Result<usize, ModbusError> {
    match request {
        Request::ReadHoldingRegisters(addr, quantity) => {
            let registers = &register_table.registers[addr as usize..(addr + quantity) as usize];
            Ok(0)
        },
        Request::WriteSingleRegister(addr, value) => {
            register_table.registers[addr as usize].set(value);
            Ok(value as usize)
        },
        Request::WriteMultipleRegisters(addr, data) => {
            let end_addr = addr as usize + data.len();
            if end_addr > register_table.registers.len() {
                return Err(ModbusError::InvalidFrame);
            }
            for i in 0..data.len() {
                if let Some(value) = data.get(i) {
                    register_table.registers[addr as usize + i].set(value);
                }
            }
            Ok(0)
        },
        _ => {
            Err(ModbusError::UnknownOpcode)
        }
    }
}
