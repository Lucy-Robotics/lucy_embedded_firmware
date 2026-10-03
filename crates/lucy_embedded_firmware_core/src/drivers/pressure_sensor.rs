//! Pressure sensor Modbus placeholder (2 registers: cmd + value).
//!
//! Real ADC sampling is not wired yet. Config codegen still emits
//! [`PressureSensorConfig`] and reserves register blocks so firmware builds
//! when YAML lists pressure sensors. The adapter clears `cmd` and leaves
//! `value` at the last placeholder reading (0 until a future ADC driver).

use crate::modbus::{ModbusAdapter, RegisterView};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub struct PressureSensorConfig {
    pub min_value: u16,
    pub max_value: u16,
}

/// Placeholder driver — no hardware ADC until sensors are implemented.
pub struct PressureSensorDriver {
    pub config: PressureSensorConfig,
    pub last_value: u16,
}

impl PressureSensorDriver {
    pub fn new(config: PressureSensorConfig) -> Self {
        Self {
            config,
            last_value: 0,
        }
    }

    /// Returns the last placeholder value (clamped to config range).
    pub fn read_placeholder(&mut self) -> u16 {
        let clamped = self
            .last_value
            .clamp(self.config.min_value, self.config.max_value);
        self.last_value = clamped;
        clamped
    }
}

/// Holding registers: `[cmd, value]`.
///
/// - cmd `1` = refresh placeholder into `value` (no ADC yet)
/// - value is written by the adapter for host reads
pub struct PressureSensorModbusAdapter {
    pub base_register: u16,
    pub cmd_reg_off: u16,
    pub value_reg_off: u16,
    pub driver: PressureSensorDriver,
}

impl ModbusAdapter for PressureSensorModbusAdapter {
    fn tick(&mut self, rv: &RegisterView) {
        let cmd = rv.read_register(self.cmd_reg_off);
        rv.write_register(self.cmd_reg_off, 0);
        if cmd == 1 {
            let value = self.driver.read_placeholder();
            rv.write_register(self.value_reg_off, value);
        }
    }

    fn get_nb_register(&self) -> u16 {
        2
    }

    fn get_base_register(&self) -> u16 {
        self.base_register
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::modbus::RegisterTable;

    #[test]
    fn pressure_block_size_is_two() {
        let mut adapter = PressureSensorModbusAdapter {
            base_register: 4,
            cmd_reg_off: 0,
            value_reg_off: 1,
            driver: PressureSensorDriver::new(PressureSensorConfig {
                min_value: 0,
                max_value: 4095,
            }),
        };
        assert_eq!(adapter.get_nb_register(), 2);
        assert_eq!(adapter.get_base_register(), 4);

        let rt = RegisterTable::default();
        rt.registers[4].set(1);
        let rv = RegisterView {
            table: &rt,
            base_register: 4,
            nb_register: 2,
        };
        adapter.tick(&rv);
        assert_eq!(rt.registers[4].get(), 0);
        assert_eq!(rt.registers[5].get(), 0);
    }

    #[test]
    fn placeholder_clamps_stored_value() {
        let mut driver = PressureSensorDriver {
            config: PressureSensorConfig {
                min_value: 100,
                max_value: 200,
            },
            last_value: 999,
        };
        assert_eq!(driver.read_placeholder(), 200);
        driver.last_value = 50;
        assert_eq!(driver.read_placeholder(), 100);
    }
}
