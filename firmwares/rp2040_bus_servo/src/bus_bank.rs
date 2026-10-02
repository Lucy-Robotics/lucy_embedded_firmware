//! YAML-driven bus-servo bank: one shared UART, N Feetech device IDs.
//!
//! Modbus block per device (matches host `LucySystemHardware`):
//! `[id, angle_millirad, cmd]` at `base_register`.

use lucy_embedded_firmware_core::drivers::{
    BusServoConfig, BusServoDriver, BusServoModbusAdapter,
};
use lucy_embedded_firmware_core::modbus::{ModbusAdapter, RegisterTable, RegisterView};
use lucy_embedded_firmware_core::uart::UartChannel;

/// Max bus servos on one UART lane (SO-ARM uses 6).
pub const MAX_BUS_DEVICES: usize = 16;

#[derive(Clone, Copy)]
pub struct BusBankDevice {
    pub device_id: u8,
    pub base_register: u16,
    pub config: BusServoConfig,
}

pub struct BusBank<U: UartChannel> {
    driver: BusServoDriver<U>,
    devices: [BusBankDevice; MAX_BUS_DEVICES],
    count: usize,
}

impl<U: UartChannel> BusBank<U> {
    pub fn new(channel: U, devices: &[BusBankDevice]) -> Self {
        let mut bank = Self {
            driver: BusServoDriver {
                config: BusServoConfig {
                    min_pulse: 0,
                    max_pulse: 4095,
                    min_angle: 0,
                    max_angle: 6283,
                    default_angle: 3142,
                },
                channel,
            },
            devices: [const {
                BusBankDevice {
                    device_id: 0,
                    base_register: 0,
                    config: BusServoConfig {
                        min_pulse: 0,
                        max_pulse: 4095,
                        min_angle: 0,
                        max_angle: 6283,
                        default_angle: 3142,
                    },
                }
            }; MAX_BUS_DEVICES],
            count: 0,
        };
        for d in devices.iter().take(MAX_BUS_DEVICES) {
            bank.devices[bank.count] = BusBankDevice {
                device_id: d.device_id,
                base_register: d.base_register,
                config: d.config,
            };
            bank.count += 1;
        }
        bank
    }

    /// Seed Modbus `id` registers from YAML `UART0:N` device ids.
    pub fn seed_id_registers(&self, rt: &RegisterTable) {
        for i in 0..self.count {
            let d = &self.devices[i];
            let rv = RegisterView {
                table: rt,
                base_register: d.base_register,
                nb_register: 3,
            };
            rv.write_register(0, u16::from(d.device_id));
        }
    }

    pub fn tick(&mut self, rt: &RegisterTable) {
        for i in 0..self.count {
            let d = self.devices[i];
            self.driver.config = d.config;
            let rv = RegisterView {
                table: rt,
                base_register: d.base_register,
                nb_register: 3,
            };
            // Prefer host-written id; fall back to YAML device_id.
            let id_reg = rv.read_register(0);
            if id_reg == 0 {
                rv.write_register(0, u16::from(d.device_id));
            }
            let mut adapter = BusServoModbusAdapter {
                base_register: d.base_register,
                id_reg_off: 0,
                angle_reg_off: 1,
                cmd_reg_off: 2,
                driver: &mut self.driver,
            };
            adapter.tick(&rv);
        }
    }
}
