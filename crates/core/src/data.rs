use portable_atomic::AtomicU64;

pub const SHM_PAYLOAD_OFFSET: usize = 144;
pub const SHM_ARRAYS: usize = 9;

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
        SHM_PAYLOAD_OFFSET.div_ceil(64) * 64 + SHM_ARRAYS * max_actuators * size_of::<f32>()
    }
}

pub struct SharedTable {
    base: *mut u8,
    len: usize,
}

impl SharedTable {
    /// # Safety
    ///
    /// `base` must be valid for reads and writes for `len` bytes, aligned
    /// for `ShmHeader`, and must stay valid and exclusively accessed
    /// through the returned `SharedTable` for as long as it exists.
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

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn max_actuators(&self) -> usize {
        self.header().max_actuators as usize
    }

    fn array(&self, idx: usize) -> *mut f32 {
        unsafe { (self.base.add(SHM_PAYLOAD_OFFSET) as *mut f32).add(idx * self.max_actuators()) }
    }

    fn slice(&self, idx: usize) -> &[f32] {
        unsafe { core::slice::from_raw_parts(self.array(idx), self.max_actuators()) }
    }

    fn slice_mut(&mut self, idx: usize) -> &mut [f32] {
        unsafe { core::slice::from_raw_parts_mut(self.array(idx), self.max_actuators()) }
    }

    pub fn hw_commands(&self) -> &[f32] {
        self.slice(0)
    }
    pub fn hw_velocities(&self) -> &[f32] {
        self.slice(1)
    }
    pub fn hw_accelerations(&self) -> &[f32] {
        self.slice(2)
    }
    pub fn hw_torque_enabled(&self) -> &[f32] {
        self.slice(3)
    }

    pub fn hw_positions(&self) -> &[f32] {
        self.slice(4)
    }
    pub fn hw_positions_mut(&mut self) -> &mut [f32] {
        self.slice_mut(4)
    }
    pub fn hw_temperatures(&self) -> &[f32] {
        self.slice(5)
    }
    pub fn hw_temperatures_mut(&mut self) -> &mut [f32] {
        self.slice_mut(5)
    }
    pub fn hw_efforts(&self) -> &[f32] {
        self.slice(6)
    }
    pub fn hw_efforts_mut(&mut self) -> &mut [f32] {
        self.slice_mut(6)
    }
    pub fn hw_currents(&self) -> &[f32] {
        self.slice(7)
    }
    pub fn hw_currents_mut(&mut self) -> &mut [f32] {
        self.slice_mut(7)
    }
    pub fn hw_loads(&self) -> &[f32] {
        self.slice(8)
    }
    pub fn hw_loads_mut(&mut self) -> &mut [f32] {
        self.slice_mut(8)
    }
}
