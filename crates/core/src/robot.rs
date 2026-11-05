use crate::joint::{*};
use crate::data::{*};

pub struct Robot<const M: usize> {
    pub joints_config: &'static [CommandConfig],
    pub joints_command: [Command; M],
    pub joints_state: [State; M],
}

impl<const M: usize> Robot<M> {
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
