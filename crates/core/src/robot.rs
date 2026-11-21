use crate::data::*;
use crate::guard::Guard;
use crate::joint::*;

pub struct Robot<const M: usize, G: Guard = ()> {
    pub joints_config: &'static [CommandConfig],
    joints_command: [Command; M],
    joints_state: [State; M],
    guard: G,
}

impl<const M: usize, G: Guard> Robot<M, G> {
    pub fn new(joints_config: &'static [CommandConfig], guard: G) -> Self {
        Self {
            joints_config,
            joints_command: [Command::default(); M],
            joints_state: [State::default(); M],
            guard,
        }
    }

    pub fn tick<R>(&mut self, drive: impl FnOnce(&mut [Command; M], &mut [State; M]) -> R) -> R {
        self.guard
            .apply(&self.joints_state, &mut self.joints_command);
        let result = drive(&mut self.joints_command, &mut self.joints_state);
        self.guard
            .apply(&self.joints_state, &mut self.joints_command);
        result
    }

    pub fn commands(&self) -> &[Command; M] {
        &self.joints_command
    }

    pub fn commands_mut(&mut self) -> &mut [Command; M] {
        &mut self.joints_command
    }

    pub fn states(&self) -> &[State; M] {
        &self.joints_state
    }

    pub fn disable_torque(&mut self) {
        for joint in self.joints_command.iter_mut() {
            joint.torque_enabled = TorqueStatus::Disabled;
        }
    }

    pub fn faulted(&self) -> bool {
        self.guard.tripped()
    }

    pub fn sync_write_shm(&mut self, shm: &SharedTable) {
        let n = M.min(shm.max_actuators());
        for (i, command) in self.joints_command.iter_mut().enumerate().take(n) {
            command.position = shm.hw_commands()[i];
            command.velocity = shm.hw_velocities()[i];
            command.acceleration = shm.hw_accelerations()[i];
            command.torque_enabled = TorqueStatus::from(shm.hw_torque_enabled()[i]);
        }
    }

    pub fn sync_read_shm(&self, shm: &mut SharedTable) {
        let n = M.min(shm.max_actuators());
        for (i, state) in self.joints_state.iter().enumerate().take(n) {
            shm.hw_positions_mut()[i] = state.position;
            shm.hw_temperatures_mut()[i] = state.temperature;
            shm.hw_efforts_mut()[i] = state.effort;
            shm.hw_currents_mut()[i] = state.current;
            shm.hw_loads_mut()[i] = state.load;
            shm.hw_voltages_mut()[i] = state.voltage;
        }
    }
}
