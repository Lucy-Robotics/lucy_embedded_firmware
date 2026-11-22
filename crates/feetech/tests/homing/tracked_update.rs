use lucy_embedded_firmware_core::joint::Command;
use lucy_embedded_firmware_core::joint::State;

use crate::fake_bus::{FakeBus, REG_CURRENT, REG_LOAD, REG_PRESENT, REG_TEMPERATURE, REG_VOLTAGE};
use crate::fixtures::driver;

#[test]
fn tracked_update_reports_which_servos_answered() {
    let mut bus = FakeBus::with_positions(&[(1, 100), (2, 200)]);
    bus.servo(2).answers = false;
    let mut command = [Command::default(); 2];
    let mut state = [State::default(); 2];
    let mut seen = [false; 2];

    // Tick 4 skips the temperature, voltage and effort reads.
    let result = driver().update_tracked(4, &mut bus, &mut command, &mut state, &mut seen);

    assert!(result.is_err());
    assert_eq!(seen, [true, false]);
}

#[test]
fn update_reads_effort_every_other_tick_temperature_and_voltage_every_tenth() {
    let mut bus = FakeBus::with_positions(&[(1, 100), (2, 200)]);
    let mut command = [Command::default(); 2];
    let mut state = [State::default(); 2];
    let mut seen = [false; 2];

    let extra_reads: Vec<Vec<u8>> = (0..10)
        .map(|tick| {
            bus.read_registers.clear();
            let _ = driver().update_tracked(tick, &mut bus, &mut command, &mut state, &mut seen);
            bus.read_registers
                .iter()
                .copied()
                .filter(|&register| register != REG_PRESENT)
                .collect()
        })
        .collect();

    let effort = vec![REG_CURRENT, REG_LOAD];
    let expected: Vec<Vec<u8>> = (0..10)
        .map(|tick| match tick {
            0 => vec![REG_TEMPERATURE],
            2 => vec![REG_VOLTAGE],
            t if t % 2 == 1 => effort.clone(),
            _ => vec![],
        })
        .collect();
    assert_eq!(extra_reads, expected);
}
