use lucy_embedded_firmware_core::data::{SHM_PAYLOAD_OFFSET, SharedTable, ShmHeader};
use lucy_embedded_firmware_core::joint::{CommandConfig, State};
use lucy_embedded_firmware_core::robot::Robot;

static JOINTS_CONFIG: [CommandConfig; 2] = [CommandConfig {
    default: 0.0,
    limit_min: 0.0,
    limit_max: 1.0,
}; 2];

/// Mirrored by `VoltagesSitAtSlot9` in `lucy_ros2_control/test/test_shared_memory_channel.cpp`.
#[test]
fn servo_voltages_reach_shared_memory_slot_9() {
    let mut buffer = vec![0u64; ShmHeader::size_for(2).div_ceil(8)];
    let base = buffer.as_mut_ptr() as *mut u8;
    unsafe { (base.add(136) as *mut u32).write(2) };
    let mut shm = unsafe { SharedTable::from_raw(base, buffer.len() * 8) }.unwrap();
    let mut robot = Robot::<2, _>::new(&JOINTS_CONFIG, ());
    robot.tick(|_, state| {
        state[0] = State {
            voltage: 12.2,
            ..State::default()
        };
        state[1] = State {
            voltage: 0.0,
            ..State::default()
        };
    });

    robot.sync_read_shm(&mut shm);

    assert_eq!(shm.hw_voltages(), [12.2, 0.0]);
    let slot_9 = unsafe { base.add(SHM_PAYLOAD_OFFSET) as *const f32 }.wrapping_add(9 * 2);
    assert_eq!(shm.hw_voltages().as_ptr(), slot_9);
}
