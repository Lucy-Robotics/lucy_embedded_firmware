use lucy_embedded_firmware_core::guard::Guard;
use lucy_embedded_firmware_core::joint::{Command, State, TorqueStatus};

pub const MEDIAN_READINGS: usize = 30;
pub const APPLIES_PER_READING: u32 = 20;

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
    readings: [[f32; MEDIAN_READINGS]; M],
    next_slot: usize,
    applies: u32,
}

impl<const M: usize> ThermalGuard<M> {
    pub const fn new(limits: &'static [ThermalLimit; M]) -> Self {
        Self {
            limits,
            tripped: [false; M],
            readings: [[f32::NAN; MEDIAN_READINGS]; M],
            next_slot: 0,
            applies: 0,
        }
    }

    pub fn tripped_joints(&self) -> &[bool; M] {
        &self.tripped
    }

    fn record(&mut self, state: &[State]) {
        if self.applies.is_multiple_of(APPLIES_PER_READING) {
            for (window, state) in self.readings.iter_mut().zip(state.iter()) {
                window[self.next_slot] = state.temperature;
            }
            self.next_slot = (self.next_slot + 1) % MEDIAN_READINGS;
        }
        self.applies = self.applies.wrapping_add(1);
    }
}

fn median(window: &[f32; MEDIAN_READINGS]) -> Option<f32> {
    let mut valid = [0f32; MEDIAN_READINGS];
    let mut count = 0;
    for &reading in window.iter().filter(|r| r.is_finite()) {
        valid[count] = reading;
        count += 1;
    }
    if count == 0 {
        return None;
    }
    let valid = &mut valid[..count];
    valid.sort_unstable_by(f32::total_cmp);
    Some(valid[(count - 1) / 2])
}

impl<const M: usize> Guard for ThermalGuard<M> {
    fn apply(&mut self, state: &[State], command: &mut [Command]) {
        self.record(state);

        let joints = self
            .limits
            .iter()
            .zip(self.tripped.iter_mut())
            .zip(self.readings.iter().zip(command.iter_mut()));

        for ((limit, tripped), (window, command)) in joints {
            match median(window) {
                Some(t) if t >= limit.max => *tripped = true,
                Some(t) if t <= limit.release => *tripped = false,
                _ => {}
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
