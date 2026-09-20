//! Hardware-PWM bank for Servo2040 silk 1..18 (GPIO 0..17).
//!
//! Slice = (gpio / 2) % 8, channel A/B = gpio & 1.
//! GPIO0/16 and GPIO1/17 share a slice channel — do not enable both.

use lucy_embedded_firmware_core::drivers::PwmServoConfig;
use lucy_embedded_firmware_core::modbus::{RegisterTable, RegisterView};
use lucy_embedded_firmware_core::utils::millirad_to_pulse;
use rp2040_hal::{
    gpio::{DynFunction, DynPinId, FunctionNull, Pin, PullDown},
    pac,
};

const PWM_TOP: u16 = 20_000 - 1;
const PWM_DIV_INT: u8 = 125;

type PwmPin = Pin<DynPinId, DynFunction, PullDown>;

pub struct PwmBank {
    /// Keep pins alive so function-select is not reset on drop.
    _pins: [Option<PwmPin>; 18],
    gpio: [u8; 18],
    base_register: [u16; 18],
    config: [PwmServoConfig; 18],
    count: usize,
}

impl PwmBank {
    /// Claim and configure only the GPIOs listed in ``devices``.
    ///
    /// ``pin_pool[g]`` must hold GPIO ``g`` (0..17) as a dyn pin, or ``None``
    /// if that pad was already taken. Unlisted pads stay unused.
    pub fn from_null_pool(
        pwm: &pac::PWM,
        pin_pool: &mut [Option<Pin<DynPinId, FunctionNull, PullDown>>; 18],
        devices: &[(u8, u16, PwmServoConfig)],
    ) -> Self {
        let mut bank = Self {
            _pins: [const { None }; 18],
            gpio: [0; 18],
            base_register: [0; 18],
            config: [PwmServoConfig {
                min_pulse: 1000,
                max_pulse: 2000,
                min_angle: 0,
                max_angle: 3142,
                default_angle: 1571,
            }; 18],
            count: 0,
        };

        for &(gpio, base, cfg) in devices.iter().take(18) {
            if gpio > 17 {
                continue;
            }
            let Some(pin) = pin_pool[gpio as usize].take() else {
                continue;
            };
            let mut pwm_pin = pin.into_function::<DynFunction>();
            let _ = pwm_pin.try_set_function(DynFunction::Pwm);
            let slice = ((gpio / 2) % 8) as usize;
            enable_slice(pwm, slice);
            let pulse = millirad_to_pulse(
                cfg.default_angle,
                cfg.min_angle,
                cfg.max_angle,
                cfg.min_pulse,
                cfg.max_pulse,
            );
            set_duty(pwm, gpio, pulse);

            bank._pins[gpio as usize] = Some(pwm_pin);
            bank.gpio[bank.count] = gpio;
            bank.base_register[bank.count] = base;
            bank.config[bank.count] = cfg;
            bank.count += 1;
        }
        bank
    }

    pub fn tick(&self, pwm: &pac::PWM, rt: &RegisterTable) {
        for i in 0..self.count {
            let base = self.base_register[i];
            let cfg = self.config[i];
            let gpio = self.gpio[i];
            let rv = RegisterView {
                table: rt,
                base_register: base,
                nb_register: 2,
            };
            let cmd = rv.read_register(0);
            rv.write_register(0, 0);
            match cmd {
                1 => {
                    let angle = rv.read_register(1);
                    let pulse = millirad_to_pulse(
                        angle,
                        cfg.min_angle,
                        cfg.max_angle,
                        cfg.min_pulse,
                        cfg.max_pulse,
                    );
                    set_duty(pwm, gpio, pulse);
                }
                2 => {
                    let pulse = millirad_to_pulse(
                        cfg.default_angle,
                        cfg.min_angle,
                        cfg.max_angle,
                        cfg.min_pulse,
                        cfg.max_pulse,
                    );
                    set_duty(pwm, gpio, pulse);
                }
                _ => {}
            }
        }
    }
}

fn enable_slice(pwm: &pac::PWM, slice: usize) {
    let ch = pwm.ch(slice);
    ch.div()
        .write(|w| unsafe { w.int().bits(PWM_DIV_INT).frac().bits(0) });
    ch.top().write(|w| unsafe { w.bits(u32::from(PWM_TOP)) });
    ch.csr().modify(|_, w| {
        w.en().set_bit();
        w.divmode().div()
    });
}

fn set_duty(pwm: &pac::PWM, gpio: u8, pulse: u16) {
    let slice = ((gpio / 2) % 8) as usize;
    let ch = pwm.ch(slice);
    let level = u32::from(pulse.min(PWM_TOP));
    if gpio & 1 == 0 {
        ch.cc().modify(|r, w| {
            let b = r.bits() & 0xffff_0000;
            unsafe { w.bits(b | level) }
        });
    } else {
        ch.cc().modify(|r, w| {
            let a = r.bits() & 0x0000_ffff;
            unsafe { w.bits(a | (level << 16)) }
        });
    }
}
