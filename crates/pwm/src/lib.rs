#![cfg_attr(not(test), no_std)]

pub mod pca9685;
pub mod pulse;

pub trait PwmChannel {
    type Error;
    fn set_pwm(&mut self, pulse: u16) -> Result<(), Self::Error>;
}
