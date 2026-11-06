use lucy_embedded_firmware_core::guard::Guard;
use lucy_embedded_firmware_core::joint::{Command, CommandConfig, State, TorqueStatus};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_security::thermal::{ThermalGuard, ThermalLimit};

static LIMITS: [ThermalLimit; 2] = [ThermalLimit::new(70.0, 60.0), ThermalLimit::new(70.0, 60.0)];
static JOINTS_CONFIG: [CommandConfig; 2] = [CommandConfig { default: 0.0, limit_min: 0.0, limit_max: 1.0 }; 2];

fn enabled() -> [Command; 2] {
    [Command { torque_enabled: TorqueStatus::Enabled, ..Command::default() }; 2]
}

fn at(t0: f32, t1: f32) -> [State; 2] {
    [State { temperature: t0, ..State::default() }, State { temperature: t1, ..State::default() }]
}

fn torque(command: &[Command; 2]) -> [TorqueStatus; 2] {
    command.map(|c| c.torque_enabled)
}

#[test]
fn below_max_keeps_commanded_torque() {
    let mut guard = ThermalGuard::new(&LIMITS);
    let mut command = enabled();
    guard.apply(&at(69.0, 25.0), &mut command);
    assert_eq!(torque(&command), [TorqueStatus::Enabled; 2]);
    assert!(!guard.tripped());
}

#[test]
fn at_max_cuts_only_the_hot_joint() {
    let mut guard = ThermalGuard::new(&LIMITS);
    let mut command = enabled();
    guard.apply(&at(70.0, 25.0), &mut command);
    assert_eq!(torque(&command), [TorqueStatus::Disabled, TorqueStatus::Enabled]);
    assert_eq!(guard.tripped_joints(), &[true, false]);
    assert!(guard.tripped());
}

#[test]
fn stays_latched_against_new_commands_until_release() {
    let mut guard = ThermalGuard::new(&LIMITS);
    guard.apply(&at(75.0, 25.0), &mut enabled());

    let mut command = enabled();
    guard.apply(&at(65.0, 25.0), &mut command);
    assert_eq!(command[0].torque_enabled, TorqueStatus::Disabled);

    let mut command = enabled();
    guard.apply(&at(60.0, 25.0), &mut command);
    assert_eq!(command[0].torque_enabled, TorqueStatus::Enabled);
    assert!(!guard.tripped());
}

#[test]
fn nan_reading_keeps_the_current_decision() {
    let mut guard = ThermalGuard::new(&LIMITS);
    guard.apply(&at(75.0, 25.0), &mut enabled());

    let mut command = enabled();
    guard.apply(&at(f32::NAN, f32::NAN), &mut command);
    assert_eq!(torque(&command), [TorqueStatus::Disabled, TorqueStatus::Enabled]);
}

#[test]
fn robot_tick_applies_the_guard_before_driving() {
    let mut robot = Robot::<2, _>::new(&JOINTS_CONFIG, ThermalGuard::new(&LIMITS));
    *robot.commands_mut() = enabled();

    robot.tick(|command, state| {
        assert_eq!(torque(command), [TorqueStatus::Enabled; 2]);
        *state = at(80.0, 25.0);
    });
    assert!(robot.faulted());

    *robot.commands_mut() = enabled();
    robot.tick(|command, _| {
        assert_eq!(torque(command), [TorqueStatus::Disabled, TorqueStatus::Enabled]);
    });
}
