use lucy_embedded_firmware_core::pwm::{PwmChannel};

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    i2c::I2c,
    pwm::SetDutyCycle,
};

use rp2040_hal::{
    fugit::RateExtU32,
    fugit::MicrosDuration,
    clocks::init_clocks_and_plls,
    gpio::{bank0, Pins, FunctionPio0, FunctionPwm, FunctionI2C, PullUp},
    pac,
    i2c::I2C,
    pwm::{Slices, AnySlice, Slice, SliceId, Channel, ChannelId, FreeRunning, A, B},
    pio::PIOExt,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
    Clock
};

pub enum ChannelError {

}

pub struct Rp2040PwmChannel<C> 
where
    C: SetDutyCycle,
{
    pub channel: C
}

impl<C> PwmChannel for Rp2040PwmChannel<C>
where
    C: SetDutyCycle,
{
    type Error = ChannelError;

    fn set_pwm(&mut self, pulse: u16) -> Result<(), ChannelError> {
        self.channel.set_duty_cycle(pulse);
        Ok(())
    }
}

