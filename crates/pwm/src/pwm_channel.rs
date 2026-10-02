use core::result::Result;

pub trait PwmChannel {
    type Error;
    fn set_pwm(&mut self, pulse: u16) -> Result<(), Self::Error>;
}
