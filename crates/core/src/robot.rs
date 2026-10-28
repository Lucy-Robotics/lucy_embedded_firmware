use crate::joint::{*};
use crate::data::{*};

pub struct Robot<const M: usize> {
    pub joints_config: &'static [CommandConfig],
    pub joints_command: [Command; M],
    pub joints_state: [State; M],
}

impl<const M: usize> Robot<M> {
    pub fn sync_write<const N: usize>(&mut self, command_block: &CommandBlock<N>) {
        for i in 0..M {
            self.joints_command[i].position = command_block.hw_commands[i];
            self.joints_command[i].velocity = command_block.hw_velocities[i];
            self.joints_command[i].acceleration = command_block.hw_accelerations[i];
            self.joints_command[i].torque_enabled = TorqueStatus::from(command_block.hw_torque_enabled[i]);
        }
    }

    pub fn sync_read<const N: usize>(&self, state_block: &mut StateBlock<N>) {
        for i in 0..M {
            state_block.hw_positions[i] = self.joints_state[i].position;
        }
    }
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
        }
    }
}
