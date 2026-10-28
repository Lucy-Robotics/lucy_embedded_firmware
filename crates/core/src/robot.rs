use crate::joint::{*};
use crate::data::{*};

pub struct Robot<const M: usize> {
    pub joints_config: &'static [CommandConfig],
    pub joints_command: [Command; M],
    pub joints_state: [State; M],
}

impl<const M: usize> Robot<M> {
    pub fn clamp_position(&self, i: usize, position: f32) -> f32 {
        match self.joints_config.get(i) {
            Some(config) => position.clamp(config.limit_min, config.limit_max),
            None => position,
        }
    }

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
            shm.hw_temperatures_mut()[i] = self.joints_state[i].temperature;
            shm.hw_efforts_mut()[i] = self.joints_state[i].effort;
            shm.hw_currents_mut()[i] = self.joints_state[i].current;
            shm.hw_loads_mut()[i] = self.joints_state[i].load;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHM_LEN: usize = ShmHeader::size_for(1);

    static CONFIG: [CommandConfig; 1] = [CommandConfig { default: 0.5, limit_min: 0.5, limit_max: 2.0 }];

    #[repr(C, align(64))]
    struct Segment([u8; SHM_LEN]);

    #[test]
    fn sync_write_shm_clamps_to_joint_limits() {
        let mut robot = Robot::<1> { joints_config: &CONFIG, joints_command: [Command::default()], joints_state: [State::default()] };
        let mut segment = Segment([0; SHM_LEN]);
        let base = segment.0.as_mut_ptr();
        unsafe { (*(base as *mut ShmHeader)).max_actuators = 1 };
        let table = unsafe { SharedTable::from_raw(base, SHM_LEN) }.unwrap();
        let command = unsafe { base.add(SHM_PAYLOAD_OFFSET) } as *mut f32;
        for (sent, expected) in [(0.0, 0.5), (1.0, 1.0), (3.0, 2.0)] {
            unsafe { *command = sent };
            robot.sync_write_shm(&table);
            assert_eq!(robot.joints_command[0].position, expected);
        }
    }
}
