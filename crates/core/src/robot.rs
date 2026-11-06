use crate::joint::{*};
use crate::data::{*};
use crate::guard::Guard;

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
        self.guard.apply(&self.joints_state, &mut self.joints_command);
        let result = drive(&mut self.joints_command, &mut self.joints_state);
        self.guard.apply(&self.joints_state, &mut self.joints_command);
        result
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
        for i in 0..n {
            self.joints_command[i].position = shm.hw_commands()[i];
            self.joints_command[i].velocity = shm.hw_velocities()[i];
            self.joints_command[i].acceleration = shm.hw_accelerations()[i];
            self.joints_command[i].torque_enabled = TorqueStatus::from(shm.hw_torque_enabled()[i]);
        }
    }

    pub fn sync_read_shm(&self, shm: &mut SharedTable) {
        let n = M.min(shm.max_actuators());
        for i in 0..n {
            shm.hw_positions_mut()[i] = self.joints_state[i].position;
            shm.hw_temperatures_mut()[i] = self.joints_state[i].temperature;
            shm.hw_efforts_mut()[i] = self.joints_state[i].effort;
            shm.hw_currents_mut()[i] = self.joints_state[i].current;
            shm.hw_loads_mut()[i] = self.joints_state[i].load;
        }
    }
}
