use core::result::Result;

pub enum TransportError {
    WriteError,
    ReadError,
    FlushError,
}

pub trait Transport {
    type Error;

    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error>;
    fn flush(&mut self) -> Result<(), Self::Error>;
}
