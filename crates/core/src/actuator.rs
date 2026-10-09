use core::result::Result;

use crate::joint::{Command, State};

pub enum ActuatorState {
    Disabled,
    Enabling { since_tick: u32 },
    Active,
    Holding,
    Faulted,
}

pub trait ActuatorGroup {
    type Bus;
    type Error;

    fn update(&mut self, tick: u64, bus: &mut Self::Bus, command: &mut [Command], state: &mut [State]) -> Result<(), Self::Error>;
}
