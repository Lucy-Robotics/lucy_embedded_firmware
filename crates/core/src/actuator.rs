use core::result::Result;
use core::error::Error;

use crate::joint::{Command, State};

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
    fn disable(&mut self);

    fn write(&mut self, command: &mut [Command], state: &mut [State]) -> Result<(), Self::Error>;
    fn read(&mut self, command: &mut [Command], state: &mut [State]) -> Result<(), Self::Error>;
    fn apply(&mut self, command: &mut [Command], state: &mut [State]) -> Result<(), Self::Error>;
}

pub trait ActuatorGroup {
    type Bus;
    type Error;

    fn update(&mut self, tick: u64, bus: &mut Self::Bus, command: &mut [Command], state: &mut [State]) -> Result<(), Self::Error>;
}
