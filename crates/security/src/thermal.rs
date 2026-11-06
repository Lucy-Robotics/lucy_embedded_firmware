use lucy_embedded_firmware_core::guard::Guard;
use lucy_embedded_firmware_core::joint::{Command, State, TorqueStatus};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalLimit {
    pub max: f32,
    pub release: f32,
}

impl ThermalLimit {
    pub const fn new(max: f32, release: f32) -> Self {
        assert!(release < max, "ThermalLimit: release must be below max");
        Self { max, release }
    }
}

pub struct ThermalGuard<const M: usize> {
    limits: &'static [ThermalLimit; M],
    tripped: [bool; M],
}

impl<const M: usize> ThermalGuard<M> {
    pub const fn new(limits: &'static [ThermalLimit; M]) -> Self {
        Self {
            limits,
            tripped: [false; M],
        }
    }

    pub fn tripped_joints(&self) -> &[bool; M] {
        &self.tripped
    }
}

impl<const M: usize> Guard for ThermalGuard<M> {
    fn apply(&mut self, state: &[State], command: &mut [Command]) {
        let joints = self
            .limits
            .iter()
            .zip(self.tripped.iter_mut())
            .zip(state.iter().zip(command.iter_mut()));

        for ((limit, tripped), (state, command)) in joints {
            if state.temperature >= limit.max {
                *tripped = true;
            } else if state.temperature <= limit.release {
                *tripped = false;
            }

            if *tripped {
                command.torque_enabled = TorqueStatus::Disabled;
            }
        }
    }

    fn tripped(&self) -> bool {
        self.tripped.iter().any(|&t| t)
    }
}
