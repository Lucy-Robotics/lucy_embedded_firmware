
use crate::driver_generic::{ModBusDriver, ModBusDriverError};
use crate::modbus::{*};
use core::result::Result;
use core::error::Error;

#[derive(Debug)]
pub enum SG90DriverError {
    Error
}

pub struct SG90ModBusAdapter {
    pub cmd_reg_off: u16,
    pub angle_reg_off: u16,
    pub base_reg: u16,
    driver: SG90Driver,
}

impl SG90ModBusAdapter {
    pub fn new(cmd_reg_off: u16, angle_reg_off: u16, min_angle: u16, max_angle: u16, default_angle: u16, base_reg: u16, pin: u16) -> Self {
        let driver = SG90Driver::new(min_angle, max_angle, default_angle, pin);


        SG90ModBusAdapter {
            cmd_reg_off,
            angle_reg_off,
            driver,
            base_reg,
        }
    }
}

impl ModBusDriver for SG90ModBusAdapter {
    fn tick(&mut self, view: RegisterView) -> Result<(), ModBusDriverError> {

        let command: u16 = view.read_register(self.cmd_reg_off);
        Ok(())
    }

    fn getNbRegisters() -> u16 {
        2
    }

    fn getBaseRegister(&mut self) ->  u16 {
        self.base_reg
    }
}

pub struct SG90Driver {
    pub min_angle: u16,
    pub max_angle: u16,
    pub default_angle: u16,
    pub pin: u16,
}

impl SG90Driver {

    pub fn new(min_angle: u16, max_angle: u16, default_angle: u16, pin: u16) -> Self {
        SG90Driver {
            min_angle,
            max_angle,
            default_angle,
            pin,
        }
    }

    pub fn move_to(angle: u16) {
    }
    
    pub fn reset() {
    }
    
    pub fn calibrate() {
    }
}
