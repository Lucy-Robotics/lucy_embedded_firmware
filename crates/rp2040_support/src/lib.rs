//! Shared RP2040 MCU bring-up and peripheral banks for Lucy board firmwares.
//!
//! Board binaries own pinmux and board-only extras (e.g. Servo2040 WS2812,
//! UART-bus pad choice). This crate holds USB CDC + Modbus polling, PWM /
//! UART bus / I2C-PWM / ADC banks, and picotool force-reset — reusable across
//! RP2040 boards. Banks activate from YAML codegen tables (`GENERATED_HAS_*`).

#![no_std]

pub mod adc_bank;
pub mod bus_bank;
pub mod i2c_pwm_bank;
pub mod modbus_cdc;
pub mod picotool_reset;
pub mod pwm_bank;
pub mod uart_channel;
pub mod usb_cdc;

pub use adc_bank::{AdcBank, AdcBankDevice, MAX_ADC_DEVICES};
pub use bus_bank::{BusBank, BusBankDevice, MAX_BUS_DEVICES};
pub use i2c_pwm_bank::{I2cPwmBank, I2cPwmBankDevice, MAX_I2C_PWM_DEVICES};
pub use modbus_cdc::ModbusCdcState;
pub use picotool_reset::PicoToolReset;
pub use pwm_bank::{unreset_pwm, PwmBank};
pub use uart_channel::Rp2040UartChannel;
pub use usb_cdc::{build_usb_device, resolve_usb_serial};
