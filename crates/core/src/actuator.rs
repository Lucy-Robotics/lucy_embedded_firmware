use core::result::Result;
use core::error::Error;

// Command

pub enum ActuatorState {
    Disabled,
    Enabling { since_tick: u32 },
    Active,
    Holding,
    Faulted,
}

pub trait Actuator {
    type Error;
    fn enable(&mut self) -> Result<(), Self::Error>;
    fn apply(&mut self) -> Result<(), Self::Error>;
    fn disable(&mut self);
}
