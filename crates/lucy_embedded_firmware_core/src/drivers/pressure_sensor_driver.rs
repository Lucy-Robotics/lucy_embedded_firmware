use crate::modbus::{RegisterView, ModbusAdapter};
use core::result::Result;
use core::error::Error;
use embedded_hal::*;

pub enum PressureSensorDriverError {
    Error
}

pub struct PressureSensorModBusAdapter {
    cmd_reg_off: u16,
    value_reg_off: u16,
    base_reg: u16,
    driver: PressureSensorDriver,
}

impl PressureSensorModBusAdapter {
    pub fn new(cmd_reg_off: u16, value_reg_off: u16, base_reg: u16) -> Self {
        let driver: PressureSensorDriver = PressureSensorDriver::new();
        PressureSensorModBusAdapter { cmd_reg_off: cmd_reg_off, value_reg_off: value_reg_off, base_reg: base_reg, driver }
    }
}

impl ModbusAdapter for PressureSensorModBusAdapter {
    fn tick(&mut self, view: &RegisterView) {
        let command: u16 = view.read_register(self.cmd_reg_off);
        if command == 1 {

        }
    }



    fn get_nb_register(&self) -> u16 {
        3
    }

    fn get_base_register(&self) ->  u16 {
        self.base_reg
    }
}

pub struct PressureSensorDriver {
}

impl PressureSensorDriver {

    pub fn new() -> Self {
        PressureSensorDriver {  }
    }
    
    pub fn read() -> u16 {
        0
    }
}
