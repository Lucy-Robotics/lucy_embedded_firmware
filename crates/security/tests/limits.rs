use lucy_embedded_firmware_core::guard::Guard;
use lucy_embedded_firmware_core::joint::{Command, CommandConfig, State, TorqueStatus};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_security::limits::PositionLimitGuard;
use lucy_embedded_firmware_security::thermal::{ThermalGuard, ThermalLimit};

static LIMITS: [CommandConfig; 2] = [CommandConfig { default: 1.0, limit_min: 0.5, limit_max: 2.0 }; 2];
static THERMAL: [ThermalLimit; 2] = [ThermalLimit::new(70.0, 60.0); 2];

fn targets(p0: f32, p1: f32) -> [Command; 2] {
    let at = |position| Command { position, torque_enabled: TorqueStatus::Enabled, ..Command::default() };
    [at(p0), at(p1)]
}

fn apply(command: &mut [Command; 2]) {
    PositionLimitGuard::new(&LIMITS).apply(&[State::default(); 2], command);
}

#[test]
fn position_inside_limits_is_untouched() {
    let mut command = targets(0.5, 1.7);
    apply(&mut command);
    assert_eq!(command, targets(0.5, 1.7));
}

#[test]
fn position_outside_limits_is_clamped() {
    let mut command = targets(-3.0, 6.0);
    apply(&mut command);
    assert_eq!(command, targets(0.5, 2.0));
}

#[test]
fn non_finite_position_drops_torque_on_that_joint_only() {
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut command = targets(bad, 1.0);
        apply(&mut command);
        assert_eq!(command[0].torque_enabled, TorqueStatus::Disabled, "{bad}");
        assert_eq!(command[1], targets(0.0, 1.0)[1]);
    }
}

#[test]
fn non_finite_is_not_latched() {
    let mut guard = PositionLimitGuard::new(&LIMITS);
    guard.apply(&[State::default(); 2], &mut targets(f32::NAN, 1.0));

    let mut command = targets(1.0, 1.0);
    guard.apply(&[State::default(); 2], &mut command);
    assert_eq!(command, targets(1.0, 1.0));
    assert!(!guard.tripped());
}

#[test]
fn robot_tick_never_drives_outside_limits() {
    let guards = (ThermalGuard::new(&THERMAL), PositionLimitGuard::new(&LIMITS));
    let mut robot = Robot::<2, _>::new(&LIMITS, guards);
    *robot.commands_mut() = targets(9.0, f32::NAN);

    robot.tick(|command, _| {
        assert_eq!(command[0].position, 2.0);
        assert_eq!(command[0].torque_enabled, TorqueStatus::Enabled);
        assert_eq!(command[1].torque_enabled, TorqueStatus::Disabled);
    });
    assert!(!robot.faulted());
}
