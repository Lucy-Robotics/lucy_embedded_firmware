use core::result::Result;

pub trait SerialChannel {
    type Error;

    fn open(&mut self) -> Result<(), Self::Error>;
    fn close(&mut self) -> Result<(), Self::Error>;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error>;

    fn clear(&mut self) -> Result<(), Self::Error>;
}
