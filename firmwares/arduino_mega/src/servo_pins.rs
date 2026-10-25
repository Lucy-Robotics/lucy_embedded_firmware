//! Servos straight on Mega pins (the Nervo board ribbons), driven like Arduino's Servo
//! library: Timer5 raises one pin, times its pulse, lowers it and moves to the next, then
//! waits out the rest of the 20 ms frame. Works on any pin, unlike the timers' OCnx outputs.

use core::cell::RefCell;

use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::{mode::Output, Pin};
use avr_device::interrupt::Mutex;

use lucy_embedded_firmware_pwm::pwm_channel::PwmChannel;

use crate::channel::PwmError;
use crate::config::MEGA_PINS;

const N: usize = MEGA_PINS.len();
/// Timer5 at 16 MHz / 8: 0.5 us per tick.
const TICKS_PER_US: u16 = 2;
const FRAME_TICKS: u16 = 20_000 * TICKS_PER_US;
/// Slot length while a pin sends no pulse (torque off), so the frame keeps its pace.
const IDLE_SLOT_TICKS: u16 = 500 * TICKS_PER_US;

struct Frame {
    pins: [Pin<Output, Dynamic>; N],
    /// Pulse per pin in timer ticks; 0 = no pulse, the servo goes limp.
    pulses: [u16; N],
    /// Pin whose pulse is running; N = waiting for the next frame.
    current: usize,
    used: u16,
}

static FRAME: Mutex<RefCell<Option<Frame>>> = Mutex::new(RefCell::new(None));

/// Starts the frame on Timer5 (CTC on OCR5A). Pins idle low until a pulse is set.
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
    // The TC5 handle is only needed for setup; the ISR reaches the registers itself.
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
            // Rest of the frame; the 8 pins x 2.4 ms cap keeps this positive.
            FRAME_TICKS.saturating_sub(f.used).max(IDLE_SLOT_TICKS)
        };
        f.used = if f.current == 0 { ticks } else { f.used.saturating_add(ticks) };
        // SAFETY: TC5 belongs to this module once init() ran; only OCR5A is touched here.
        unsafe { (*arduino_hal::pac::TC5::ptr()).ocr5a().write(|w| w.set(ticks)) };
    });
}

/// One Mega pin servo, `index` into MEGA_PINS.
pub struct PinServo {
    pub index: usize,
}

impl PwmChannel for PinServo {
    type Error = PwmError;

    /// `pulse` in microseconds; 0 stops pulses so the servo goes limp.
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
