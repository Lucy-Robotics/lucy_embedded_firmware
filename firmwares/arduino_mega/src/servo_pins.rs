use core::cell::RefCell;

use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::{mode::Output, Pin};
use avr_device::interrupt::Mutex;

use lucy_embedded_firmware_pwm::pwm_channel::PwmChannel;

use crate::channel::PwmError;
use crate::config::MEGA_PINS;

const N: usize = MEGA_PINS.len();
const TICKS_PER_US: u16 = 2;
const FRAME_TICKS: u16 = 20_000 * TICKS_PER_US;
const IDLE_SLOT_TICKS: u16 = 500 * TICKS_PER_US;

struct Frame {
    pins: [Pin<Output, Dynamic>; N],
    /// Timer ticks; 0 = limp
    pulses: [u16; N],
    /// N = waiting for the next frame
    current: usize,
    used: u16,
}

static FRAME: Mutex<RefCell<Option<Frame>>> = Mutex::new(RefCell::new(None));

pub fn init(tc5: arduino_hal::pac::TC5, mut pins: [Pin<Output, Dynamic>; N]) {
    for pin in pins.iter_mut() {
        pin.set_low();
    }
    avr_device::interrupt::free(|cs| {
        FRAME.borrow(cs).replace(Some(Frame { pins, pulses: [0; N], current: N, used: 0 }));
    });
    if N == 0 {
        return;
    }
    tc5.tccr5a().write(|w| w.wgm5().set(0b00));
    tc5.tccr5b().write(|w| w.wgm5().set(0b01).cs5().prescale_8());
    tc5.ocr5a().write(|w| w.set(IDLE_SLOT_TICKS));
    tc5.timsk5().write(|w| w.ocie5a().set_bit());
    core::mem::forget(tc5);
    unsafe { avr_device::interrupt::enable() };
}

#[avr_device::interrupt(atmega2560)]
fn TIMER5_COMPA() {
    avr_device::interrupt::free(|cs| {
        let mut frame = FRAME.borrow(cs).borrow_mut();
        let Some(f) = frame.as_mut() else { return };
        if f.current < N {
            f.pins[f.current].set_low();
        }
        f.current = if f.current >= N { 0 } else { f.current + 1 };
        let ticks = if f.current < N {
            let pulse = f.pulses[f.current];
            if pulse > 0 {
                f.pins[f.current].set_high();
                pulse
            } else {
                IDLE_SLOT_TICKS
            }
        } else {
            FRAME_TICKS.saturating_sub(f.used).max(IDLE_SLOT_TICKS)
        };
        f.used = if f.current == 0 { ticks } else { f.used.saturating_add(ticks) };
        // SAFETY: this module owns TC5 after init()
        unsafe { (*arduino_hal::pac::TC5::ptr()).ocr5a().write(|w| w.set(ticks)) };
    });
}

pub struct PinServo {
    pub index: usize,
}

impl PwmChannel for PinServo {
    type Error = PwmError;

    /// `pulse` in us; 0 = limp
    fn set_pwm(&mut self, pulse: u16) -> Result<(), Self::Error> {
        let ticks = pulse.min(2_400).saturating_mul(TICKS_PER_US);
        avr_device::interrupt::free(|cs| {
            if let Some(f) = FRAME.borrow(cs).borrow_mut().as_mut() {
                f.pulses[self.index] = ticks;
            }
        });
        Ok(())
    }
}
