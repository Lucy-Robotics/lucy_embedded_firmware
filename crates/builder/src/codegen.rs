//! Quote codegen: AssignmentPlan → `$OUT_DIR/config.rs`.

use crate::assign::{config_type_for_driver, plan_config};
use crate::schema::AssignmentPlan;
use lucy_embedded_firmware_core::board_layout::HardwareIdentity;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use heck::ToShoutySnakeCase;
use heck::ToSnakeCase;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use crate::schema::FirmwareConfig;

pub fn build_config(config_path: impl AsRef<Path>) {
    let config_path = config_path.as_ref();
    println!("cargo:rerun-if-changed={}", config_path.display());

    let out_dir = env::var_os("OUT_DIR").expect("OUT_DIR not set");
    let filepath = PathBuf::from(out_dir).join("config.rs");

    let contents = fs::read_to_string(config_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", config_path.display()));

    let parsed: FirmwareConfig = serde_yaml::from_str(&contents)
        .unwrap_or_else(|e| panic!("failed to parse {}: {e}", config_path.display()));

    let plan = plan_config(&parsed).unwrap_or_else(|e| {
        panic!("config assignment failed for {}: {e}", config_path.display())
    });
    let code = generate_config_tokens(&plan);
    fs::write(&filepath, code.to_string())
        .unwrap_or_else(|e| panic!("failed to write {}: {e}", filepath.display()));
}

/// Write a minimal empty generated config (when YAML has no actuators yet).
pub fn write_empty_config() {
    let out_dir = env::var_os("OUT_DIR").expect("OUT_DIR not set");
    let filepath = PathBuf::from(out_dir).join("config.rs");
    let code = quote! {
        /// Auto-generated stub (no actuators: in config.yaml).
        pub const GENERATED_BOARD_ID: &str = "unset";
        pub const GENERATED_SLAVE_ADDRESS: u8 = 1;
        pub const GENERATED_USB_SERIAL_ID: &str = "";
        pub const GENERATED_DEVICE_COUNT: usize = 0;
        #[derive(Clone, Copy)]
        pub struct GeneratedPwmDevice {
            pub gpio: u8,
            pub base_register: u16,
            pub config: lucy_embedded_firmware_core::drivers::PwmServoConfig,
        }
        pub const GENERATED_PWM_DEVICE_COUNT: usize = 0;
        pub const GENERATED_PWM_DEVICES: &[GeneratedPwmDevice] = &[];
        #[derive(Clone, Copy)]
        pub struct GeneratedBusDevice {
            pub uart: u8,
            pub device_id: u8,
            pub base_register: u16,
            pub config: lucy_embedded_firmware_core::drivers::BusServoConfig,
        }
        pub const GENERATED_BUS_DEVICE_COUNT: usize = 0;
        pub const GENERATED_BUS_DEVICES: &[GeneratedBusDevice] = &[];
        pub fn init_generated_configs() {}
    };
    fs::write(&filepath, code.to_string())
        .unwrap_or_else(|e| panic!("failed to write {}: {e}", filepath.display()));
}

fn hw_identity_tokens(hw: &HardwareIdentity) -> TokenStream {
    match hw {
        HardwareIdentity::PwmGpio { servo_index, gpio } => quote! {
            lucy_embedded_firmware_core::board_layout::HardwareIdentity::PwmGpio {
                servo_index: #servo_index,
                gpio: #gpio,
            }
        },
        HardwareIdentity::Adc { channel } => quote! {
            lucy_embedded_firmware_core::board_layout::HardwareIdentity::Adc {
                channel: #channel,
            }
        },
        HardwareIdentity::UartBus { uart, device_id } => {
            let dev = match device_id {
                Some(d) => quote! { Some(#d) },
                None => quote! { None },
            };
            quote! {
                lucy_embedded_firmware_core::board_layout::HardwareIdentity::UartBus {
                    uart: #uart,
                    device_id: #dev,
                }
            }
        }
        HardwareIdentity::I2cPwm {
            bus,
            device,
            channel,
        } => quote! {
            lucy_embedded_firmware_core::board_layout::HardwareIdentity::I2cPwm {
                bus: #bus,
                device: #device,
                channel: #channel,
            }
        },
    }
}

pub fn generate_config_tokens(plan: &AssignmentPlan) -> TokenStream {
    let slave = plan.slave_address;
    let board = plan.board_id.as_str();
    let board_class = plan.board_class.as_str();
    let serial_id = plan.serial_id.as_str();
    let device_count = plan.devices.len();

    let mut const_blocks = Vec::new();
    let mut init_lets = Vec::new();

    for dev in &plan.devices {
        let snake = dev.id.to_snake_case();
        let shouty = dev.id.to_shouty_snake_case();
        let pin_const = format_ident!("{}_VIRTUAL_PIN", shouty);
        let base_const = format_ident!("{}_BASE_REGISTER", shouty);
        let nb_const = format_ident!("{}_NB_REGISTERS", shouty);
        let channel_const = format_ident!("{}_CHANNEL", shouty);
        let config_const = format_ident!("{}_CONFIG", shouty);
        let hw_fn = format_ident!("{}_hardware", snake);
        let config_type = format_ident!("{}", config_type_for_driver(&dev.driver));
        let vp = dev.virtual_pin;
        let base = dev.base_register;
        let nb = dev.nb_registers;
        let channel = dev.channel.as_str();
        let hw_tok = hw_identity_tokens(&dev.hardware);

        let field_tokens: Vec<TokenStream> = dev
            .fields
            .iter()
            .map(|(k, v)| {
                let key = format_ident!("{}", k);
                quote! { #key: #v }
            })
            .collect();

        const_blocks.push(quote! {
            #[allow(dead_code)]
            pub const #pin_const: u16 = #vp;
            #[allow(dead_code)]
            pub const #base_const: u16 = #base;
            #[allow(dead_code)]
            pub const #nb_const: u16 = #nb;
            #[allow(dead_code)]
            pub const #channel_const: &str = #channel;
            #[allow(dead_code)]
            pub const #config_const: lucy_embedded_firmware_core::drivers::#config_type =
                lucy_embedded_firmware_core::drivers::#config_type {
                    #(#field_tokens),*
                };
            #[allow(dead_code)]
            pub const fn #hw_fn() -> lucy_embedded_firmware_core::board_layout::HardwareIdentity {
                #hw_tok
            }
        });

        let config_var = format_ident!("{}_config", snake);
        init_lets.push(quote! {
            let #config_var = #config_const;
            let _ = (#base_const, #pin_const, #config_var);
        });
    }

    let mut pwm_entries = Vec::new();
    let mut bus_entries = Vec::new();
    for dev in &plan.devices {
        match &dev.hardware {
            HardwareIdentity::PwmGpio { gpio, .. } => {
                let shouty = dev.id.to_shouty_snake_case();
                let config_const = format_ident!("{}_CONFIG", shouty);
                let base = dev.base_register;
                pwm_entries.push(quote! {
                    GeneratedPwmDevice {
                        gpio: #gpio,
                        base_register: #base,
                        config: #config_const,
                    }
                });
            }
            HardwareIdentity::UartBus { uart, device_id } => {
                let shouty = dev.id.to_shouty_snake_case();
                let config_const = format_ident!("{}_CONFIG", shouty);
                let base = dev.base_register;
                let device_id = device_id.unwrap_or(0);
                bus_entries.push(quote! {
                    GeneratedBusDevice {
                        uart: #uart,
                        device_id: #device_id,
                        base_register: #base,
                        config: #config_const,
                    }
                });
            }
            _ => {}
        }
    }
    let pwm_count = pwm_entries.len();
    let bus_count = bus_entries.len();

    quote! {
        /// Auto-generated by `builder::build_config`. Do not edit.
        pub const GENERATED_BOARD_ID: &str = #board;
        pub const GENERATED_BOARD_CLASS: &str = #board_class;
        pub const GENERATED_SLAVE_ADDRESS: u8 = #slave;
        /// USB CDC serial; must match host ``serial_id`` (flash unique id hex).
        pub const GENERATED_USB_SERIAL_ID: &str = #serial_id;
        pub const GENERATED_DEVICE_COUNT: usize = #device_count;

        #(#const_blocks)*

        /// One enabled on-board PWM servo (Servo2040 silk = gpio+1).
        #[derive(Clone, Copy)]
        pub struct GeneratedPwmDevice {
            pub gpio: u8,
            pub base_register: u16,
            pub config: lucy_embedded_firmware_core::drivers::PwmServoConfig,
        }

        pub const GENERATED_PWM_DEVICE_COUNT: usize = #pwm_count;
        pub const GENERATED_PWM_DEVICES: &[GeneratedPwmDevice] = &[
            #(#pwm_entries),*
        ];

        /// One enabled UART bus servo (`UART<n>:<feetech_id>`).
        #[derive(Clone, Copy)]
        pub struct GeneratedBusDevice {
            pub uart: u8,
            pub device_id: u8,
            pub base_register: u16,
            pub config: lucy_embedded_firmware_core::drivers::BusServoConfig,
        }

        pub const GENERATED_BUS_DEVICE_COUNT: usize = #bus_count;
        pub const GENERATED_BUS_DEVICES: &[GeneratedBusDevice] = &[
            #(#bus_entries),*
        ];

        #[allow(unused_variables, unused_mut)]
        pub fn init_generated_configs() {
            #(#init_lets)*
        }
    }
}
