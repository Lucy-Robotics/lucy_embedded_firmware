use crate::joint::{*};
use crate::data::{*};

pub struct Robot<const M: usize> {
    pub joints_config: &'static [CommandConfig],
    pub joints_command: [Command; M],
    pub joints_state: [State; M],
}

impl<const M: usize> Robot<M> {
    pub fn clamp_position(&self, i: usize, position: f64) -> f64 {
        match self.joints_config.get(i) {
            Some(config) => position.clamp(config.limit_min, config.limit_max),
            None => position,
        }
    }

    pub fn sync_write<const N: usize>(&mut self, command_block: &CommandBlock<N>) {
        for i in 0..M {
            self.joints_command[i].position = self.clamp_position(i, command_block.hw_commands[i]);
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
            self.joints_command[i].position = self.clamp_position(i, shm.hw_commands()[i]);
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

#[cfg(test)]
mod tests {
    use super::*;
    use portable_atomic::AtomicU64;

    static CONFIG: [CommandConfig; 1] = [CommandConfig { default: 0.5, limit_min: 0.5, limit_max: 2.0 }];

    #[test]
    fn sync_write_clamps_to_joint_limits() {
        let mut robot = Robot::<1> { joints_config: &CONFIG, joints_command: [Command::default()], joints_state: [State::default()] };
        let mut block = CommandBlock::<1> {
            command_seq: AtomicU64::new(0),
            hw_commands: [0.0],
            hw_velocities: [0.0],
            hw_accelerations: [0.0],
            hw_torque_enabled: [1.0],
        };
        for (sent, expected) in [(0.0, 0.5), (1.0, 1.0), (3.0, 2.0)] {
            block.hw_commands[0] = sent;
            robot.sync_write(&block);
            assert_eq!(robot.joints_command[0].position, expected);
        }
    }
}
