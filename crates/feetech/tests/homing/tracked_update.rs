use lucy_embedded_firmware_core::joint::Command;
use lucy_embedded_firmware_core::joint::State;

use crate::fake_bus::FakeBus;
use crate::fixtures::driver;

#[test]
fn tracked_update_reports_which_servos_answered() {
    let mut bus = FakeBus::with_positions(&[(1, 100), (2, 200)]);
    bus.servo(2).answers = false;
    let mut command = [Command::default(); 2];
    let mut state = [State::default(); 2];
    let mut seen = [false; 2];

    // Tick 5 skips the temperature and effort reads.
    let result = driver().update_tracked(5, &mut bus, &mut command, &mut state, &mut seen);

    assert!(result.is_err());
    assert_eq!(seen, [true, false]);
}
