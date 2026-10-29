use portable_atomic::AtomicU64;

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
pub struct Table<const N: usize> {
    pub commands: CommandBlock<N>,
    pub states: StateBlock<N>,
    pub heartbeat: HeartbeatBlock,
}


pub const SHM_PAYLOAD_OFFSET: usize = 144;

/// Must match `ActuatorSharedState` in `lucy_ros2_control`.
#[repr(C)]
pub struct ShmHeader {
    pub command_seq: AtomicU64,
    _pad0: [u8; 56],
    pub state_seq: AtomicU64,
    _pad1: [u8; 56],
    pub heartbeat: AtomicU64,
    pub max_actuators: u32,
    _pad2: u32,
}

const _: () = assert!(size_of::<ShmHeader>() == SHM_PAYLOAD_OFFSET);

impl ShmHeader {
    pub const fn size_for(max_actuators: usize) -> usize {
        (SHM_PAYLOAD_OFFSET + 63) / 64 * 64 + 5 * max_actuators * size_of::<f64>()
    }
}

pub struct SharedTable {
    base: *mut u8,
    len: usize,
}

impl SharedTable {
    /// # Safety
    /// `base` must point to a mapping of at least `len` bytes that outlives the view.
    pub unsafe fn from_raw(base: *mut u8, len: usize) -> Result<Self, &'static str> {
        if len < size_of::<ShmHeader>() {
            return Err("shm smaller than header");
        }
        let table = Self { base, len };
        if len < ShmHeader::size_for(table.header().max_actuators as usize) {
            return Err("shm smaller than declared max_actuators");
        }
        Ok(table)
    }

    pub fn header(&self) -> &ShmHeader {
        unsafe { &*(self.base as *const ShmHeader) }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn max_actuators(&self) -> usize {
        self.header().max_actuators as usize
    }

    fn array(&self, idx: usize) -> *mut f64 {
        unsafe { (self.base.add(SHM_PAYLOAD_OFFSET) as *mut f64).add(idx * self.max_actuators()) }
    }

    fn slice(&self, idx: usize) -> &[f64] {
        unsafe { core::slice::from_raw_parts(self.array(idx), self.max_actuators()) }
    }

    fn slice_mut(&mut self, idx: usize) -> &mut [f64] {
        unsafe { core::slice::from_raw_parts_mut(self.array(idx), self.max_actuators()) }
    }

    pub fn hw_commands(&self) -> &[f64] { self.slice(0) }
    pub fn hw_velocities(&self) -> &[f64] { self.slice(1) }
    pub fn hw_accelerations(&self) -> &[f64] { self.slice(2) }
    pub fn hw_torque_enabled(&self) -> &[f64] { self.slice(3) }
    pub fn hw_positions(&self) -> &[f64] { self.slice(4) }
    pub fn hw_positions_mut(&mut self) -> &mut [f64] { self.slice_mut(4) }
}
