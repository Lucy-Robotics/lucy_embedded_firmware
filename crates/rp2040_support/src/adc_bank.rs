//! ADC / pressure-sensor Modbus bank (placeholder until RP2040 ADC is wired).
//!
//! Processes `GENERATED_ADC_DEVICES` via [`PressureSensorModbusAdapter`].

use lucy_embedded_firmware_core::drivers::{
    PressureSensorConfig, PressureSensorDriver, PressureSensorModbusAdapter,
};
use lucy_embedded_firmware_core::modbus::{ModbusAdapter, RegisterTable, RegisterView};

pub const MAX_ADC_DEVICES: usize = 8;

#[derive(Clone, Copy)]
pub struct AdcBankDevice {
    pub channel: u8,
    pub base_register: u16,
    pub config: PressureSensorConfig,
}

pub struct AdcBank {
    devices: [AdcBankDevice; MAX_ADC_DEVICES],
    adapters: [PressureSensorModbusAdapter; MAX_ADC_DEVICES],
    count: usize,
}

impl AdcBank {
    pub fn new(devices: &[AdcBankDevice]) -> Self {
        let empty_cfg = PressureSensorConfig {
            min_value: 0,
            max_value: 4095,
        };
        let mut bank = Self {
            devices: [const {
                AdcBankDevice {
                    channel: 0,
                    base_register: 0,
                    config: PressureSensorConfig {
                        min_value: 0,
                        max_value: 4095,
                    },
                }
            }; MAX_ADC_DEVICES],
            adapters: [const {
                PressureSensorModbusAdapter {
                    base_register: 0,
                    cmd_reg_off: 0,
                    value_reg_off: 1,
                    driver: PressureSensorDriver {
                        config: PressureSensorConfig {
                            min_value: 0,
                            max_value: 4095,
                        },
                        last_value: 0,
                    },
                }
            }; MAX_ADC_DEVICES],
            count: 0,
        };
        let _ = empty_cfg;
        for d in devices.iter().take(MAX_ADC_DEVICES) {
            bank.devices[bank.count] = *d;
            bank.adapters[bank.count] = PressureSensorModbusAdapter {
                base_register: d.base_register,
                cmd_reg_off: 0,
                value_reg_off: 1,
                driver: PressureSensorDriver::new(d.config),
            };
            bank.count += 1;
        }
        bank
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn tick(&mut self, rt: &RegisterTable) {
        for i in 0..self.count {
            let base = self.devices[i].base_register;
            let rv = RegisterView {
                table: rt,
                base_register: base,
                nb_register: 2,
            };
            self.adapters[i].tick(&rv);
        }
    }
}
