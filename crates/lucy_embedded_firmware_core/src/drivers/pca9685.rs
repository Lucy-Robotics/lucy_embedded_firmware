pub const PCA9685_DEFAULT_ADDR: u8 = 0x40;

pub const PCA9685_PWM_REGS: u16 = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pca9685Config {
    pub address: u8,
    pub channel: u8,
}

impl Pca9685Config {
    pub const fn new(channel: u8) -> Self {
        Self {
            address: PCA9685_DEFAULT_ADDR,
            channel,
        }
    }
}
