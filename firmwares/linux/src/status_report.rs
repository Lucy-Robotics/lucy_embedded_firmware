//! JSON formatting for bus status and bus errors. Pure formatting, no control flow.

use lucy_embedded_firmware_core::joint::TorqueStatus;
use lucy_embedded_firmware_feetech::feetech::{FeetechBusError, HomingRecord};
use lucy_embedded_firmware_telemetry::loop_timer::LoopTimer;

use std::time::Instant;

use crate::config::{FEETECH_0, LucyRobot, Telemetry};
use crate::maintenance::json_string;
use crate::resources::Resources;

pub(crate) fn bus_error_fields(error: FeetechBusError) -> String {
    match error {
        FeetechBusError::OutOfLimits => "\"error\":\"out_of_limits\"".to_string(),
        FeetechBusError::CommunicationError => "\"error\":\"bus_error\"".to_string(),
        FeetechBusError::ReadbackMismatch => "\"error\":\"readback_mismatch\"".to_string(),
        FeetechBusError::OffsetOutOfRange(id) => {
            format!("\"error\":\"offset_out_of_range\",\"servo\":{id}")
        }
    }
}

pub(crate) fn bus_error_json(error: FeetechBusError) -> String {
    format!("{{\"ok\":false,{}}}", bus_error_fields(error))
}

pub(crate) fn records_json(records: &[HomingRecord; 6]) -> String {
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

/// Last complete timing window, `null` until the first one closes.
fn loop_json(timer: &LoopTimer) -> String {
    timer.report().map_or_else(
        || "null".to_string(),
        |r| {
            format!(
                "{{\"period_us\":{},\"samples\":{},\"period_min_us\":{},\"period_mean_us\":{},\"period_max_us\":{},\"jitter_mean_us\":{},\"jitter_max_us\":{},\"busy_max_us\":{},\"overruns\":{}}}",
                r.period_us,
                r.samples,
                r.period_min_us,
                r.period_mean_us,
                r.period_max_us,
                r.jitter_mean_us,
                r.jitter_max_us,
                r.busy_max_us,
                r.overruns,
            )
        },
    )
}

fn optional_string(value: Option<&str>) -> String {
    value.map_or_else(|| "null".to_string(), json_string)
}

pub(crate) fn status_json(
    robot: &LucyRobot,
    resources: &Resources,
    telemetry: &Telemetry,
) -> String {
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
            let state = &robot.states()[i];
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
                robot.commands()[i].torque_enabled == TorqueStatus::Enabled,
            )
        })
        .collect();

    format!(
        "{{\"ok\":true,\"link\":{},\"adapters\":[{}],\"selected\":{},\"open_serial\":{},\"open_error\":{},\"servos\":[{}],\"loop\":{}}}",
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
        loop_json(&telemetry.loop_timer),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{TICK_PERIOD_US, TIMING_WINDOW_TICKS, get_robot};
    use crate::test_support::offline;
    use core::f32::consts::TAU;
    use std::time::Duration;

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
        robot.tick(|_, state| state[0].position = 2047.0 * TAU / 4096.0);
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

    #[test]
    fn status_has_no_loop_timing_before_a_full_window() {
        let reply = status_json(&get_robot(), &offline(), &Telemetry::default());
        assert!(reply.ends_with(",\"loop\":null}"), "{reply}");
    }

    #[test]
    fn status_reports_the_last_loop_timing_window() {
        let mut telemetry = Telemetry::default();
        let timer = &mut telemetry.loop_timer;
        for tick in 0..=TIMING_WINDOW_TICKS {
            let start = tick * TICK_PERIOD_US + if tick == 50 { 300 } else { 0 };
            timer.begin(start);
            timer.end(start + 2_000);
        }
        let reply = status_json(&get_robot(), &offline(), &telemetry);
        assert!(
            reply.ends_with(
                ",\"loop\":{\"period_us\":10000,\"samples\":1000,\"period_min_us\":9700,\"period_mean_us\":10000,\"period_max_us\":10300,\"jitter_mean_us\":0,\"jitter_max_us\":300,\"busy_max_us\":2000,\"overruns\":0}}"
            ),
            "{reply}"
        );
    }

    #[test]
    fn bus_error_fields_name_the_offending_servo() {
        assert_eq!(
            bus_error_fields(FeetechBusError::OffsetOutOfRange(3)),
            "\"error\":\"offset_out_of_range\",\"servo\":3"
        );
        assert_eq!(
            bus_error_json(FeetechBusError::ReadbackMismatch),
            "{\"ok\":false,\"error\":\"readback_mismatch\"}"
        );
    }

    #[test]
    fn records_json_lists_one_entry_per_configured_servo() {
        let records = [HomingRecord::default(); 6];
        let json = records_json(&records);
        assert_eq!(json.matches("\"id\":").count(), 6, "{json}");
    }
}
