use crate::joint::{*};
use crate::actuator::{*};
use crate::data::{*};

pub struct Robot<A, const M: usize, const N: usize> {
    pub command: [JointCommand; M],
    pub state: [JointState; M],
    pub joint_config: [JointConfig; M],
    pub actuator_config: [A; N],
}

impl<A, const M: usize, const N: usize> Robot<A, M, N> {
    pub fn sync_write(&mut self, command_block: &CommandBlock<N>) {
        for i in 0..N {
            self.command[i].position = command_block.hw_commands[i];
            self.command[i].velocity = command_block.hw_velocities[i];
            self.command[i].acceleration = command_block.hw_accelerations[i];
            self.command[i].torque_enabled = TorqueStatus::from(command_block.hw_torque_enabled[i]);
        }
    }

    pub fn sync_read(&self, state_block: &mut StateBlock<N>) {
        for i in 0..N {
            state_block.hw_positions[i] = self.state[i].position;
        }
    }

    pub fn tick(&mut self) {
    }
}

