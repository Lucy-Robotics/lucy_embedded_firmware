use lucy_embedded_firmware_core::pwm::{PwmChannel};

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    i2c::I2c,
    pwm::SetDutyCycle,
};

pub enum ChannelError {

}

pub struct ArduinoMega2560PwmChannel
{
    // Timer / Counter 1, works only for certain pins
    pub tc1: arduino_hal::pac::TC1
}

impl ArduinoMega2560PwmChannel 
{
    pub fn new(tc1: arduino_hal::pac::TC1) -> Self {
        unsafe {
            tc1.tccr1a()
                .write(|w| w.wgm1().bits(0b10).com1a().match_clear());

            tc1.icr1()
                .write(|w| w.bits(39999));   // TOP first
            
            tc1.ocr1a()
                .write(|w| w.bits(3000));   // 1.5 ms = centre, valid from the start

            tc1.tccr1b()
                .write(|w| w.wgm1().bits(0b11).cs1().prescale_8()); // start the timer last
    }

        Self { tc1 }
    }

    fn enable_output(&mut self) {
        self.tc1.tccr1a().modify(|_, w| w.com1a().match_clear());
    }

    /// Stop sending pulses: the servo goes limp and stops drawing current.
    pub fn release(&mut self) {
        self.tc1.tccr1a().modify(|_, w| w.com1a().disconnected());
    }

    pub fn set_angle(&mut self, angle: u16) {
        let angle = angle.min(180);
        let pulse = 2000 + (angle as u32 * 2000 / 180) as u16;

        self.set_pwm(pulse);
    }

}

impl PwmChannel for ArduinoMega2560PwmChannel
{
    type Error = ChannelError;

    fn set_pwm(&mut self, pulse: u16) -> Result<(), ChannelError> {    
        unsafe {
            self.tc1.ocr1a().write(|w| w.bits(pulse));
        }

        self.enable_output();

        Ok(())
    }
}
