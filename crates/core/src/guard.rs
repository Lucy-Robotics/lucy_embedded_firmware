use crate::joint::{Command, State};

pub trait Guard {
    fn apply(&mut self, state: &[State], command: &mut [Command]);

    fn tripped(&self) -> bool {
        false
    }
}

impl Guard for () {
    fn apply(&mut self, _state: &[State], _command: &mut [Command]) {}
}

impl<A: Guard, B: Guard> Guard for (A, B) {
    fn apply(&mut self, state: &[State], command: &mut [Command]) {
        self.0.apply(state, command);
        self.1.apply(state, command);
    }

    fn tripped(&self) -> bool {
        self.0.tripped() || self.1.tripped()
    }
}
