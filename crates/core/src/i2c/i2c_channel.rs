use core::result::Result;

pub trait I2cChannel {
    type Error;
    fn write(address: u16, write: &[u8]) -> Result<(), Self::Error>;
}
