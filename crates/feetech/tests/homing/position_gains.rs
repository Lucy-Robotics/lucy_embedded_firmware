use lucy_embedded_firmware_feetech::feetech::PositionGains;

use crate::fake_bus::FakeBus;
use crate::fixtures::driver;
use crate::fixtures::no_delay;

#[test]
fn position_gains_reach_every_servo_and_leave_the_eeprom_locked() {
    let mut bus = FakeBus::with_positions(&[(1, 2048), (2, 2048)]);

    driver()
        .set_position_gains(&mut bus, PositionGains::SO101, &mut no_delay())
        .unwrap();

    for id in [1, 2] {
        // P 16, D 32, I 0: https://github.com/huggingface/lerobot/blob/7d615acf9aef770e66054ce1d38d256e2922448f/src/lerobot/robots/so_follower/config_so_follower.py#L45-L47
        assert_eq!(bus.servo(id).gains, [16, 32, 0]);
        assert!(bus.servo(id).locked);
    }
}
