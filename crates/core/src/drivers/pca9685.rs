
struct Pca9685<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C: I2c> Pca9685<I2C> {
    const fn new(i2c: I2C, address: u8) -> Self {
        Pca9685 {
            i2c: i2c,
            address: address,
        }
    }

    fn init(&mut self, frequency: u16) {
        let divisor = 4096 * 50;
        let prescale = ((25_000_000 + divisor / 2) / divisor - 1) as u8;
        self.i2c.write(self.address, &[0x00, 0x10]).unwrap();
        self.i2c.write(self.address, &[0xFE, prescale]).unwrap();
        self.i2c.write(self.address, &[0x00, 0x20]).unwrap();
        //delay.delay_us(500);
        self.i2c.write(self.address, &[0x01, 0x04]).unwrap();
    }

    fn get_channel(&mut self, channel: u16) {

    }
}

/*struct Pca9685Channel {
    channel: u8
}

impl<I2C: I2c> PwmChannel for Pca9685Channel<I2C> {
    fn set_pwm(&mut self, pulse: u16) {
        let on = 0;
        let off = pulse;
        let reg = 0x06 + self.channel * 4;
        let buffer = [
            reg,
            (on & 0xFF) as u8,
            ((on >> 8) & 0x0F) as u8,
            (off & 0xFF) as u8,
            ((off >> 8) & 0x0F) as u8,
        ];
        self.i2c.write(0x40_u8, &buffer).unwrap();
    }
}*/

