//! Turn a maintenance-socket [`Request`] into a reply, against the live robot and bus.
//!
//! Runs between control ticks, so the bus is free. EEPROM writes are refused outright while any
//! joint has torque on, and must name the adapter that is actually open.

mod eeprom;

use lucy_embedded_firmware_core::joint::TorqueStatus;

use crate::config::{LucyRobot, Telemetry};
use crate::maintenance::{Request, json_error};
use crate::resources::Resources;
use crate::status_report::status_json;
use eeprom::{handle_read_homing, handle_restore_homing, handle_set_half_turn};

pub fn handle_maintenance(
    request: Result<Request, String>,
    robot: &LucyRobot,
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
        Request::ReadHoming => handle_read_homing(resources),
        Request::SetHalfTurn { serial } => handle_set_half_turn(serial, robot, resources),
        Request::RestoreHoming { serial, entries } => {
            handle_restore_homing(serial, entries, robot, resources)
        }
    }
}

fn torque_on(robot: &LucyRobot) -> bool {
    robot
        .commands()
        .iter()
        .any(|c| c.torque_enabled == TorqueStatus::Enabled)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::get_robot;
    use crate::test_support::{offline, torque_enabled_robot};

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
    fn select_is_refused_while_torque_is_on() {
        let reply = handle_maintenance(
            Ok(Request::Select("A".to_string())),
            &torque_enabled_robot(),
            &mut offline(),
            &Telemetry::default(),
        );
        assert_eq!(reply, json_error("torque_on"));
    }

    #[test]
    fn torque_on_is_true_only_when_a_joint_is_enabled() {
        assert!(!torque_on(&get_robot()));
        assert!(torque_on(&torque_enabled_robot()));
    }
}
