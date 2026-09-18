//! Pressure sensor driver + Modbus adapter (2 registers: cmd + value).

use crate::modbus::{ModbusAdapter, RegisterView};

/// ADC sample source used by [`PressureSensorDriver`].
pub trait AdcChannel {
    type Error;
    fn read(&mut self) -> Result<u16, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub struct PressureSensorConfig {
    pub min_value: u16,
    pub max_value: u16,
}

pub struct PressureSensorDriver<C> {
    pub config: PressureSensorConfig,
    pub channel: C,
    pub last_value: u16,
}

impl<C: AdcChannel> PressureSensorDriver<C> {
    pub fn read(&mut self) -> u16 {
        let raw = self.channel.read().unwrap_or(self.last_value);
        let clamped = raw.clamp(self.config.min_value, self.config.max_value);
        self.last_value = clamped;
        clamped
    }
}

/// Holding registers: `[cmd, value]`.
///
/// - cmd `1` = sample ADC into `value`
/// - value is read-only from the host (written by the adapter)
pub struct PressureSensorModbusAdapter<C> {
    pub base_register: u16,
    pub cmd_reg_off: u16,
    pub value_reg_off: u16,
    pub driver: PressureSensorDriver<C>,
}

impl<C: AdcChannel> ModbusAdapter for PressureSensorModbusAdapter<C> {
    fn tick(&mut self, rv: &RegisterView) {
        let cmd = rv.read_register(self.cmd_reg_off);
        rv.write_register(self.cmd_reg_off, 0);
        match cmd {
            1 => {
                let value = self.driver.read();
                rv.write_register(self.value_reg_off, value);
            }
            _ => {}
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

    struct FakeAdc {
        value: u16,
    }

    impl AdcChannel for FakeAdc {
        type Error = ();
        fn read(&mut self) -> Result<u16, Self::Error> {
            Ok(self.value)
        }
    }

    #[test]
    fn pressure_block_size_is_two() {
        let mut adapter = PressureSensorModbusAdapter {
            base_register: 4,
            cmd_reg_off: 0,
            value_reg_off: 1,
            driver: PressureSensorDriver {
                config: PressureSensorConfig {
                    min_value: 0,
                    max_value: 4095,
                },
                channel: FakeAdc { value: 1234 },
                last_value: 0,
            },
        };
        assert_eq!(adapter.get_nb_register(), 2);
        assert_eq!(adapter.get_base_register(), 4);

        let rt = RegisterTable::default();
        // Seed cmd=1 at absolute register 4
        rt.registers[4].set(1);
        let rv = RegisterView {
            table: &rt,
            base_register: 4,
            nb_register: 2,
        };
        adapter.tick(&rv);
        assert_eq!(rt.registers[4].get(), 0);
        assert_eq!(rt.registers[5].get(), 1234);
    }

    #[test]
    fn clamps_to_config_range() {
        let mut driver = PressureSensorDriver {
            config: PressureSensorConfig {
                min_value: 100,
                max_value: 200,
            },
            channel: FakeAdc { value: 999 },
            last_value: 0,
        };
        assert_eq!(driver.read(), 200);
        driver.channel.value = 50;
        assert_eq!(driver.read(), 100);
    }
}
