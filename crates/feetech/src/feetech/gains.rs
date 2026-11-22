use super::driver::{FeetechBusDriver, FeetechBusError};
use lucy_embedded_firmware_serial::SerialChannel;

const REG_P_COEFFICIENT: u8 = 0x15;
const REG_LOCK: u8 = 0x37;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PositionGains {
    pub p: u8,
    pub d: u8,
    pub i: u8,
}

impl PositionGains {
    pub const SO101: Self = Self { p: 16, d: 32, i: 0 };
}

impl<S: SerialChannel, const N: usize> FeetechBusDriver<S, N> {
    pub fn set_position_gains(
        &mut self,
        bus: &mut S,
        gains: PositionGains,
        delay: &mut impl FnMut(),
    ) -> Result<(), FeetechBusError> {
        self.sync_write_all(bus, REG_LOCK, &[[0u8]; N])?;
        delay();
        let written =
            self.sync_write_all(bus, REG_P_COEFFICIENT, &[[gains.p, gains.d, gains.i]; N]);
        delay();
        let locked = self.sync_write_all(bus, REG_LOCK, &[[1u8]; N]);
        written.and(locked)
    }
}
