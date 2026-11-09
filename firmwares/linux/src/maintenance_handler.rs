//! Turn a maintenance-socket [`Request`] into a reply, against the live robot and bus.
//!
//! Runs between control ticks, so the bus is free. EEPROM writes are refused outright while any
//! joint has torque on, and must name the adapter that is actually open.

use lucy_embedded_firmware_core::joint::TorqueStatus;
use lucy_embedded_firmware_core::robot::Robot;
use lucy_embedded_firmware_feetech::feetech::HomingRecord;

use std::time::Duration;

use crate::config::{FEETECH_0, Telemetry, driver};
use crate::maintenance::{Request, RestoreEntry, json_error};
use crate::resources::Resources;
use crate::status_report::{bus_error_fields, bus_error_json, records_json, status_json};

const EEPROM_DELAY: Duration = Duration::from_millis(20);

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

fn torque_on(robot: &Robot<6>) -> bool {
    robot
        .joints_command
        .iter()
        .any(|c| c.torque_enabled == TorqueStatus::Enabled)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::get_robot;
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
    fn status_requests_do_not_need_an_open_bus() {
        let reply = handle_maintenance(
            Ok(Request::Status),
            &get_robot(),
            &mut offline(),
            &Telemetry::default(),
        );
        assert!(reply.contains("\"ok\":true"), "{reply}");
    }

    #[test]
    fn select_without_torque_updates_the_chosen_adapter() {
        let reply = handle_maintenance(
            Ok(Request::Select("A".to_string())),
            &get_robot(),
            &mut offline(),
            &Telemetry::default(),
        );
        assert!(reply.contains("\"selected\":\"A\""), "{reply}");
    }

    #[test]
    fn homing_record_from_entry_copies_every_field() {
        let entry = RestoreEntry {
            id: 1,
            homing_offset: -5,
            min_limit: 10,
            max_limit: 4000,
        };
        let record = homing_record_from_entry(&entry);
        assert_eq!(record.homing_offset, -5);
        assert_eq!(record.min_limit, 10);
        assert_eq!(record.max_limit, 4000);
    }

    #[test]
    fn torque_on_is_true_only_when_a_joint_is_enabled() {
        assert!(!torque_on(&get_robot()));
        assert!(torque_on(&torque_enabled_robot()));
    }

    #[test]
    fn restore_homing_rejects_an_unknown_serial_even_with_valid_entries() {
        let robot = get_robot();
        let reply = handle_maintenance(
            Ok(Request::RestoreHoming {
                serial: "A".to_string(),
                entries: entries(),
            }),
            &robot,
            &mut offline(),
            &Telemetry::default(),
        );
        assert_eq!(reply, json_error("no_adapter"));
    }

    #[test]
    fn restore_homing_rejects_bad_entries_before_checking_the_adapter() {
        let robot = get_robot();
        let mut bad = entries();
        bad.pop();
        let reply = handle_maintenance(
            Ok(Request::RestoreHoming {
                serial: "A".to_string(),
                entries: bad,
            }),
            &robot,
            &mut offline(),
            &Telemetry::default(),
        );
        assert_eq!(reply, json_error("bad_restore_values"));
    }
}
