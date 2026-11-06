use lucy_embedded_firmware_core::joint::*;
use lucy_embedded_firmware_core::robot::Robot;

use lucy_embedded_firmware_feetech::feetech::{
    FeetechBusConfig, FeetechBusDriver, FeetechBusError, FeetechServoConfig, HomingRecord,
};

use crate::maintenance::{Request, RestoreEntry, json_error, json_string};
use crate::resources::Resources;
use crate::usb_port::UsbPort;

use core::f32::consts::{PI, TAU};
use std::io::Write;
use std::time::{Duration, Instant};

const EEPROM_DELAY: Duration = Duration::from_millis(20);

pub const JOINTS_CONFIG: [CommandConfig; 6] = [
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
    CommandConfig {
        default: PI,
        limit_min: 0.0,
        limit_max: TAU,
    },
];

pub const FEETECH_0: FeetechBusConfig<6> = FeetechBusConfig {
    servos: [
        FeetechServoConfig {
            id: 1,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 2,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 3,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 4,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 5,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
        FeetechServoConfig {
            id: 6,
            amplitude: TAU,
            min_pulse: 0,
            max_pulse: 4096,
            torque: 2.94,
            gear_ratio: 1.0,
            stall_current: 2.7,
            no_load_current: 0.18,
            current_lsb: 0.0065,
        },
    ],
};

pub fn get_robot() -> Robot<6> {
    Robot {
        joints_config: &JOINTS_CONFIG,
        joints_command: [Command::default(); 6],
        joints_state: [State::default(); 6],
    }
}

/// When each servo last answered a position read. The bus result alone can't say which one failed,
/// and the SHM state counter advances regardless.
#[derive(Default)]
pub struct Telemetry {
    pub last_seen: [Option<Instant>; 6],
}

pub fn robot_tick(
    tick: u64,
    robot: &mut Robot<6>,
    resources: &mut Resources,
    telemetry: &mut Telemetry,
) {
    let Some(usb) = resources.usb0.as_mut() else {
        return;
    };
    let mut seen = [false; 6];
    let _ = driver().update_tracked(
        tick,
        usb,
        &mut robot.joints_command[0..6],
        &mut robot.joints_state[0..6],
        &mut seen,
    );
    let now = Instant::now();
    for (last, seen) in telemetry.last_seen.iter_mut().zip(seen) {
        if seen {
            *last = Some(now);
        }
    }
}

pub fn robot_release(resources: &mut Resources) {
    let Some(usb) = resources.usb0.as_mut() else {
        return;
    };
    let mut driver = driver();
    let mut ok = false;
    for _ in 0..2 {
        ok |= driver.set_torque(usb, &[Command::default(); 6]).is_ok();
    }
    if !ok {
        let _ = writeln!(
            std::io::stderr(),
            "Warning: torque off failed, the servos may still hold"
        );
    }
}

fn driver() -> FeetechBusDriver<UsbPort, 6> {
    FeetechBusDriver::new(&FEETECH_0)
}

fn torque_on(robot: &Robot<6>) -> bool {
    robot
        .joints_command
        .iter()
        .any(|c| c.torque_enabled == TorqueStatus::Enabled)
}

/// Answer a maintenance socket request. Runs between control ticks, so the bus is free.
pub fn handle_maintenance(
    request: Result<Request, String>,
    robot: &Robot<6>,
    resources: &mut Resources,
    telemetry: &Telemetry,
) -> String {
    let Ok(request) = request else {
        return json_error("bad_request");
    };

    match request {
        Request::Status => status_json(robot, resources, telemetry),
        Request::Select(serial) => {
            if torque_on(robot) {
                return json_error("torque_on");
            }
            resources.select(serial);
            status_json(robot, resources, telemetry)
        }
        Request::ReadHoming => {
            let Some(usb) = resources.usb0.as_mut() else {
                return json_error("no_adapter");
            };
            match driver().read_homing(usb) {
                Ok(records) => format!("{{\"ok\":true,\"homing\":{}}}", records_json(&records)),
                Err(e) => bus_error_json(e),
            }
        }
        Request::SetHalfTurn { serial } => {
            if torque_on(robot) {
                return json_error("torque_on");
            }
            if let Err(refusal) = check_open_serial(resources, &serial) {
                return refusal;
            }
            let Some(usb) = resources.usb0.as_mut() else {
                return json_error("no_adapter");
            };
            match driver().set_half_turn(usb, &mut eeprom_delay) {
                Ok(done) => format!(
                    "{{\"ok\":true,\"old\":{},\"new\":{}}}",
                    records_json(&done.old),
                    records_json(&done.new),
                ),
                Err(fault) => format!(
                    "{{\"ok\":false,{},\"restored\":{}}}",
                    bus_error_fields(fault.error),
                    fault.restored,
                ),
            }
        }
        Request::RestoreHoming { serial, entries } => {
            if torque_on(robot) {
                return json_error("torque_on");
            }
            let Some(records) = records_from_entries(&entries) else {
                return json_error("bad_restore_values");
            };
            if let Err(refusal) = check_open_serial(resources, &serial) {
                return refusal;
            }
            let Some(usb) = resources.usb0.as_mut() else {
                return json_error("no_adapter");
            };
            match driver().write_homing(usb, &records, &mut eeprom_delay) {
                Ok(()) => "{\"ok\":true}".to_string(),
                Err(e) => bus_error_json(e),
            }
        }
    }
}

/// An EEPROM write must name the adapter that is actually open.
fn check_open_serial(resources: &Resources, serial: &str) -> Result<(), String> {
    match &resources.open_adapter {
        None => Err(json_error("no_adapter")),
        Some(open) if open.serial.as_deref() == Some(serial) => Ok(()),
        Some(_) => Err(json_error("serial_mismatch")),
    }
}

fn eeprom_delay() {
    std::thread::sleep(EEPROM_DELAY);
}

/// Exactly one valid entry per configured servo, in configuration order.
fn records_from_entries(entries: &[RestoreEntry]) -> Option<[HomingRecord; 6]> {
    if entries.len() != FEETECH_0.servos.len() {
        return None;
    }
    let mut records = [HomingRecord::default(); 6];
    for (record, servo) in records.iter_mut().zip(FEETECH_0.servos.iter()) {
        let entry = entries.iter().find(|e| e.id == servo.id)?;
        *record = homing_record_from_entry(entry);
        if !record.is_valid() {
            return None;
        }
    }
    Some(records)
}

fn homing_record_from_entry(entry: &RestoreEntry) -> HomingRecord {
    HomingRecord {
        homing_offset: entry.homing_offset,
        min_limit: entry.min_limit,
        max_limit: entry.max_limit,
    }
}

fn bus_error_fields(error: FeetechBusError) -> String {
    match error {
        FeetechBusError::OutOfLimits => "\"error\":\"out_of_limits\"".to_string(),
        FeetechBusError::CommunicationError => "\"error\":\"bus_error\"".to_string(),
        FeetechBusError::ReadbackMismatch => "\"error\":\"readback_mismatch\"".to_string(),
        FeetechBusError::OffsetOutOfRange(id) => {
            format!("\"error\":\"offset_out_of_range\",\"servo\":{id}")
        }
    }
}

fn bus_error_json(error: FeetechBusError) -> String {
    format!("{{\"ok\":false,{}}}", bus_error_fields(error))
}

fn records_json(records: &[HomingRecord; 6]) -> String {
    let items: Vec<String> = records
        .iter()
        .zip(FEETECH_0.servos.iter())
        .map(|(record, servo)| {
            format!(
                "{{\"id\":{},\"homing_offset\":{},\"min_limit\":{},\"max_limit\":{}}}",
                servo.id, record.homing_offset, record.min_limit, record.max_limit,
            )
        })
        .collect();
    format!("[{}]", items.join(","))
}

fn optional_string(value: Option<&str>) -> String {
    value.map_or_else(|| "null".to_string(), json_string)
}

fn status_json(robot: &Robot<6>, resources: &Resources, telemetry: &Telemetry) -> String {
    let now = Instant::now();

    let adapters: Vec<String> = resources
        .adapters
        .iter()
        .map(|a| {
            format!(
                "{{\"port\":{},\"serial\":{}}}",
                json_string(&a.port_name),
                optional_string(a.serial.as_deref()),
            )
        })
        .collect();

    let servos: Vec<String> = FEETECH_0
        .servos
        .iter()
        .enumerate()
        .map(|(i, cfg)| {
            let state = &robot.joints_state[i];
            let ticks = (state.position * cfg.max_pulse as f32 / cfg.amplitude).round() as i32;
            let age = telemetry.last_seen[i].map_or_else(
                || "null".to_string(),
                |seen| now.duration_since(seen).as_millis().to_string(),
            );
            format!(
                "{{\"id\":{},\"ticks\":{},\"temperature\":{},\"load\":{},\"age_ms\":{},\"torque\":{}}}",
                cfg.id,
                ticks,
                state.temperature,
                state.load,
                age,
                robot.joints_command[i].torque_enabled == TorqueStatus::Enabled,
            )
        })
        .collect();

    format!(
        "{{\"ok\":true,\"link\":{},\"adapters\":[{}],\"selected\":{},\"open_serial\":{},\"open_error\":{},\"servos\":[{}]}}",
        json_string(resources.link.as_str()),
        adapters.join(","),
        optional_string(resources.selected.as_deref()),
        optional_string(
            resources
                .open_adapter
                .as_ref()
                .and_then(|a| a.serial.as_deref())
        ),
        optional_string(resources.open_error.as_deref()),
        servos.join(","),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb_port::{Adapter, UsbPortConfig};

    fn offline() -> Resources {
        Resources::new(UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000))
    }

    fn torque_enabled_robot() -> Robot<6> {
        let mut robot = get_robot();
        robot.joints_command[2].torque_enabled = TorqueStatus::Enabled;
        robot
    }

    fn entries() -> Vec<RestoreEntry> {
        (1..=6)
            .map(|id| RestoreEntry {
                id,
                homing_offset: -5,
                min_limit: 0,
                max_limit: 4095,
            })
            .collect()
    }

    #[test]
    fn eeprom_changes_are_refused_while_any_joint_has_torque() {
        let robot = torque_enabled_robot();
        for request in [
            Request::SetHalfTurn {
                serial: "A".to_string(),
            },
            Request::RestoreHoming {
                serial: "A".to_string(),
                entries: entries(),
            },
            Request::Select("A".to_string()),
        ] {
            let reply =
                handle_maintenance(Ok(request), &robot, &mut offline(), &Telemetry::default());
            assert_eq!(reply, json_error("torque_on"));
        }
    }

    #[test]
    fn bus_requests_report_a_missing_adapter() {
        let robot = get_robot();
        for request in [
            Request::ReadHoming,
            Request::SetHalfTurn {
                serial: "A".to_string(),
            },
            Request::RestoreHoming {
                serial: "A".to_string(),
                entries: entries(),
            },
        ] {
            let reply =
                handle_maintenance(Ok(request), &robot, &mut offline(), &Telemetry::default());
            assert_eq!(reply, json_error("no_adapter"));
        }
    }

    fn open_on(serial: Option<&str>) -> Resources {
        let mut resources = offline();
        resources.open_adapter = Some(Adapter {
            port_name: "/dev/ttyACM0".to_string(),
            serial: serial.map(str::to_string),
        });
        resources
    }

    #[test]
    fn eeprom_writes_naming_another_adapter_are_refused() {
        let robot = get_robot();
        for open in [Some("REAL"), None] {
            for request in [
                Request::SetHalfTurn {
                    serial: "OTHER".to_string(),
                },
                Request::RestoreHoming {
                    serial: "OTHER".to_string(),
                    entries: entries(),
                },
            ] {
                let reply = handle_maintenance(
                    Ok(request),
                    &robot,
                    &mut open_on(open),
                    &Telemetry::default(),
                );
                assert_eq!(reply, json_error("serial_mismatch"));
            }
        }
    }

    #[test]
    fn naming_the_open_adapter_passes_the_serial_check() {
        assert!(check_open_serial(&open_on(Some("REAL")), "REAL").is_ok());
    }

    #[test]
    fn bad_request_is_answered_not_fatal() {
        let reply = handle_maintenance(
            Err("nope".to_string()),
            &get_robot(),
            &mut offline(),
            &Telemetry::default(),
        );
        assert_eq!(reply, json_error("bad_request"));
    }

    #[test]
    fn restore_needs_every_servo_exactly_once_with_valid_values() {
        assert!(records_from_entries(&entries()).is_some());

        let mut missing = entries();
        missing.pop();
        assert!(records_from_entries(&missing).is_none());

        let mut wrong_id = entries();
        wrong_id[5].id = 9;
        assert!(records_from_entries(&wrong_id).is_none());

        let mut too_large = entries();
        too_large[0].max_limit = 5000;
        assert!(records_from_entries(&too_large).is_none());

        let mut too_far = entries();
        too_far[0].homing_offset = 2048;
        assert!(records_from_entries(&too_far).is_none());
    }

    #[test]
    fn status_without_a_bus_never_claims_a_fresh_read() {
        let reply = status_json(&get_robot(), &offline(), &Telemetry::default());
        assert!(reply.contains("\"link\":\"no_adapter\""), "{reply}");
        assert_eq!(reply.matches("\"age_ms\":null").count(), 6, "{reply}");
        assert!(reply.contains("\"open_serial\":null"), "{reply}");
    }

    #[test]
    fn status_converts_position_back_to_ticks() {
        let mut robot = get_robot();
        robot.joints_state[0].position = 2047.0 * TAU / 4096.0;
        let reply = status_json(&robot, &offline(), &Telemetry::default());
        assert!(reply.contains("\"id\":1,\"ticks\":2047,"), "{reply}");
    }

    #[test]
    fn status_reports_the_age_of_the_last_good_read() {
        let mut telemetry = Telemetry::default();
        telemetry.last_seen[1] = Some(Instant::now() - Duration::from_secs(5));
        let reply = status_json(&get_robot(), &offline(), &telemetry);
        let age: u64 = reply
            .split("\"id\":2,")
            .nth(1)
            .and_then(|s| s.split("\"age_ms\":").nth(1))
            .and_then(|s| s.split(',').next())
            .and_then(|s| s.parse().ok())
            .unwrap();
        assert!(age >= 5000, "{reply}");
    }
}
