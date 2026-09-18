//! Stub board crate for internal PWM + I2C expander (`board_class: internal_servo_i2c_pwm`).
#![no_std]
#![no_main]

mod board_layout;

include!(concat!(env!("OUT_DIR"), "/config.rs"));

use cortex_m_rt::entry;
use panic_halt as _;
// Force-link the PAC interrupt vectors (cortex-m-rt `device`); unused deps are dropped.
use rp2040_hal as _;

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[entry]
fn main() -> ! {
    let _ = GENERATED_SLAVE_ADDRESS;
    init_generated_configs();
    loop {
        cortex_m::asm::wfi();
    }
}
