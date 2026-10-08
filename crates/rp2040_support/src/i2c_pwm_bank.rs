//! I2C / PCA9685 PWM Modbus bank (placeholder until I2C HAL is wired).
//!
//! Accepts `GENERATED_I2C_PWM_DEVICES` and processes the same Modbus cmd/pulse
//! protocol as on-board PWM servos. Hardware writes are no-ops until a real
//! PCA9685 driver is connected.

use lucy_embedded_firmware_core::drivers::PwmServoConfig;
use lucy_embedded_firmware_core::modbus::{RegisterTable, RegisterView};
use lucy_embedded_firmware_core::utils::{clamp_pulse, rad_to_pulse};

pub const MAX_I2C_PWM_DEVICES: usize = 16;

#[derive(Clone, Copy)]
pub struct I2cPwmBankDevice {
    pub bus: u8,
    pub device_addr: u8,
    pub channel: u8,
    pub base_register: u16,
    pub config: PwmServoConfig,
}

pub struct I2cPwmBank {
    devices: [I2cPwmBankDevice; MAX_I2C_PWM_DEVICES],
    /// Last commanded pulse per slot (for future I2C flush).
    last_pulse: [u16; MAX_I2C_PWM_DEVICES],
    count: usize,
}

impl I2cPwmBank {
    pub fn new(devices: &[I2cPwmBankDevice]) -> Self {
        let mut bank = Self {
            devices: [const {
                I2cPwmBankDevice {
                    bus: 0,
                    device_addr: 0x40,
                    channel: 0,
                    base_register: 0,
                    config: PwmServoConfig {
                        min_pulse: 1000,
                        max_pulse: 2000,
                        min_angle: 0.0,
                        max_angle: core::f32::consts::PI,
                        default_angle: core::f32::consts::FRAC_PI_2,
                    },
                }
            }; MAX_I2C_PWM_DEVICES],
            last_pulse: [0; MAX_I2C_PWM_DEVICES],
            count: 0,
        };
        for d in devices.iter().take(MAX_I2C_PWM_DEVICES) {
            let pulse = rad_to_pulse(
                d.config.default_angle,
                d.config.min_angle,
                d.config.max_angle,
                d.config.min_pulse,
                d.config.max_pulse,
            );
            bank.devices[bank.count] = *d;
            bank.last_pulse[bank.count] = pulse;
            bank.count += 1;
        }
        bank
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn tick(&mut self, rt: &RegisterTable) {
        for i in 0..self.count {
            let d = self.devices[i];
            let rv = RegisterView {
                table: rt,
                base_register: d.base_register,
                nb_register: 2,
            };
            let cmd = rv.read_register(0);
            rv.write_register(0, 0);
            let pulse = match cmd {
                1 => Some(clamp_pulse(
                    rv.read_register(1),
                    d.config.min_pulse,
                    d.config.max_pulse,
                )),
                2 => Some(rad_to_pulse(
                    d.config.default_angle,
                    d.config.min_angle,
                    d.config.max_angle,
                    d.config.min_pulse,
                    d.config.max_pulse,
                )),
                _ => None,
            };
            if let Some(p) = pulse {
                // Placeholder: retain pulse until PCA9685 I2C write is wired.
                self.last_pulse[i] = p;
            }
        }
    }
}
