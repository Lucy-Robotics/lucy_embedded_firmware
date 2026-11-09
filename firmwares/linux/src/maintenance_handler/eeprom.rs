//! EEPROM-writing requests: reading, half-turn homing, and restoring a saved calibration.
//!
//! Every write first checks torque is off and that the request names the adapter that is
//! actually open, matching the invariant the maintenance socket as a whole relies on.

mod restore_entries;

use lucy_embedded_firmware_core::robot::Robot;

use std::time::Duration;

use super::torque_on;
use crate::config::driver;
use crate::maintenance::{RestoreEntry, json_error};
use crate::resources::Resources;
use crate::status_report::{bus_error_fields, bus_error_json, records_json};
use restore_entries::records_from_entries;

const EEPROM_DELAY: Duration = Duration::from_millis(20);

pub(super) fn handle_read_homing(resources: &mut Resources) -> String {
    let Some(usb) = resources.usb0.as_mut() else {
        return json_error("no_adapter");
    };
    match driver().read_homing(usb) {
        Ok(records) => format!("{{\"ok\":true,\"homing\":{}}}", records_json(&records)),
        Err(e) => bus_error_json(e),
    }
}

pub(super) fn handle_set_half_turn(
    serial: String,
    robot: &Robot<6>,
    resources: &mut Resources,
) -> String {
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

pub(super) fn handle_restore_homing(
    serial: String,
    entries: Vec<RestoreEntry>,
    robot: &Robot<6>,
    resources: &mut Resources,
) -> String {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Telemetry, get_robot};
    use crate::maintenance::Request;
    use crate::maintenance_handler::handle_maintenance;
    use crate::test_support::{entries, offline, open_on, torque_enabled_robot};

    #[test]
    fn naming_the_open_adapter_passes_the_serial_check() {
        assert!(check_open_serial(&open_on(Some("REAL")), "REAL").is_ok());
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
