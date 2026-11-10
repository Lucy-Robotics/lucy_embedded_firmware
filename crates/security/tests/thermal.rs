use lucy_embedded_firmware_core::guard::Guard;
use lucy_embedded_firmware_core::joint::{Command, CommandConfig, State, TorqueStatus};
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_security::thermal::{
    APPLIES_PER_READING, MEDIAN_READINGS, ThermalGuard, ThermalLimit,
};

static LIMITS: [ThermalLimit; 2] = [ThermalLimit::new(70.0, 60.0), ThermalLimit::new(70.0, 60.0)];
static JOINTS_CONFIG: [CommandConfig; 2] = [CommandConfig {
    default: 0.0,
    limit_min: 0.0,
    limit_max: 1.0,
}; 2];

fn enabled() -> [Command; 2] {
    [Command {
        torque_enabled: TorqueStatus::Enabled,
        ..Command::default()
    }; 2]
}

fn at(t0: f32, t1: f32) -> [State; 2] {
    [
        State {
            temperature: t0,
            ..State::default()
        },
        State {
            temperature: t1,
            ..State::default()
        },
    ]
}

fn torque(command: &[Command; 2]) -> [TorqueStatus; 2] {
    command.map(|c| c.torque_enabled)
}

fn read(guard: &mut ThermalGuard<2>, t0: f32, t1: f32) -> [Command; 2] {
    let mut command = enabled();
    for _ in 0..APPLIES_PER_READING {
        command = enabled();
        guard.apply(&at(t0, t1), &mut command);
    }
    command
}

#[test]
fn below_max_keeps_commanded_torque() {
    let mut guard = ThermalGuard::new(&LIMITS);
    let command = read(&mut guard, 69.0, 25.0);
    assert_eq!(torque(&command), [TorqueStatus::Enabled; 2]);
    assert!(!guard.tripped());
}

#[test]
fn one_hot_reading_among_normal_ones_keeps_torque() {
    let mut guard = ThermalGuard::new(&LIMITS);
    for _ in 0..10 {
        read(&mut guard, 30.0, 25.0);
    }
    let command = read(&mut guard, 77.0, 25.0);
    assert_eq!(torque(&command), [TorqueStatus::Enabled; 2]);
    assert!(!guard.tripped());
}

#[test]
fn a_median_at_max_cuts_only_the_hot_joint() {
    let mut guard = ThermalGuard::new(&LIMITS);
    for _ in 0..MEDIAN_READINGS {
        read(&mut guard, 30.0, 25.0);
    }
    for _ in 0..MEDIAN_READINGS / 2 {
        assert_eq!(
            read(&mut guard, 70.0, 25.0)[0].torque_enabled,
            TorqueStatus::Enabled
        );
    }
    let command = read(&mut guard, 70.0, 25.0);
    assert_eq!(
        torque(&command),
        [TorqueStatus::Disabled, TorqueStatus::Enabled]
    );
    assert_eq!(guard.tripped_joints(), &[true, false]);
    assert!(guard.tripped());
}

#[test]
fn stays_latched_until_the_median_cools_to_release() {
    let mut guard = ThermalGuard::new(&LIMITS);
    for _ in 0..MEDIAN_READINGS {
        read(&mut guard, 75.0, 25.0);
    }
    for _ in 0..MEDIAN_READINGS {
        assert_eq!(
            read(&mut guard, 65.0, 25.0)[0].torque_enabled,
            TorqueStatus::Disabled
        );
    }
    for _ in 0..MEDIAN_READINGS / 2 {
        read(&mut guard, 60.0, 25.0);
    }
    assert_eq!(
        read(&mut guard, 60.0, 25.0)[0].torque_enabled,
        TorqueStatus::Enabled
    );
    assert!(!guard.tripped());
}

#[test]
fn nan_readings_keep_the_current_decision() {
    let mut guard = ThermalGuard::new(&LIMITS);
    read(&mut guard, 75.0, 25.0);

    let command = read(&mut guard, f32::NAN, f32::NAN);
    assert_eq!(
        torque(&command),
        [TorqueStatus::Disabled, TorqueStatus::Enabled]
    );
}

#[test]
fn a_servo_that_never_reports_a_temperature_keeps_torque() {
    let mut guard = ThermalGuard::new(&LIMITS);
    let command = read(&mut guard, f32::NAN, 25.0);
    assert_eq!(torque(&command), [TorqueStatus::Enabled; 2]);
    assert!(!guard.tripped());
}

#[test]
fn robot_tick_applies_the_guard_before_driving() {
    let mut robot = Robot::<2, _>::new(&JOINTS_CONFIG, ThermalGuard::new(&LIMITS));
    *robot.commands_mut() = enabled();

    robot.tick(|command, state| {
        assert_eq!(torque(command), [TorqueStatus::Enabled; 2]);
        *state = at(80.0, 25.0);
    });
    for _ in 0..APPLIES_PER_READING {
        robot.tick(|_, _| {});
    }
    assert!(robot.faulted());

    *robot.commands_mut() = enabled();
    robot.tick(|command, _| {
        assert_eq!(
            torque(command),
            [TorqueStatus::Disabled, TorqueStatus::Enabled]
        );
    });
}
