use core::sync::atomic::{AtomicU64};

#[repr(C, align(64))]
pub struct CommandBlock<const N: usize> {
    pub command_seq: AtomicU64,
    pub hw_commands: [f64; N],
    pub hw_velocities: [f64; N],
    pub hw_accelerations: [f64; N],
    pub hw_torque_enabled: [f64; N],
}

#[repr(C, align(64))]
pub struct StateBlock<const N: usize> {
    pub state_seq: AtomicU64,
    pub hw_positions: [f64; N],
}

#[repr(C, align(64))]
pub struct HeartbeatBlock {
    pub heartbeat: AtomicU64,
}

#[repr(C, align(64))]
pub struct JointTable<const N: usize> {
    pub commands: CommandBlock<N>,
    pub states: StateBlock<N>,
    pub heartbeat: HeartbeatBlock,
}
