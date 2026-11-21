use lucy_embedded_firmware_telemetry::loop_timer::{LoopReport, LoopTimer, Pace};

const PERIOD: u32 = 1_000;

/// Ticks starting at each timestamp, each doing `busy` µs of work.
fn run(timer: &mut LoopTimer, starts: &[u32], busy: u32) {
    for &start in starts {
        timer.begin(start);
        timer.end(start.wrapping_add(busy));
    }
}

#[test]
fn no_report_before_a_full_window() {
    let mut timer = LoopTimer::new(PERIOD, 4);
    run(&mut timer, &[0, 1_000, 2_000, 3_000], 100);
    assert_eq!(timer.report(), None);
}

#[test]
fn steady_loop_has_no_jitter() {
    let mut timer = LoopTimer::new(PERIOD, 4);
    run(&mut timer, &[0, 1_000, 2_000, 3_000, 4_000], 100);
    assert_eq!(
        timer.report(),
        Some(LoopReport {
            period_us: PERIOD,
            samples: 4,
            period_min_us: 1_000,
            period_mean_us: 1_000,
            period_max_us: 1_000,
            jitter_mean_us: 0,
            jitter_max_us: 0,
            busy_max_us: 100,
            overruns: 0,
        })
    );
}

#[test]
fn late_tick_shows_as_jitter_on_both_sides() {
    let mut timer = LoopTimer::new(PERIOD, 4);
    // Third tick starts 500 µs late; the fourth is back on schedule.
    run(&mut timer, &[0, 1_000, 2_000, 3_500, 4_000], 100);
    let report = timer.report().unwrap();
    assert_eq!(report.period_min_us, 500);
    assert_eq!(report.period_max_us, 1_500);
    assert_eq!(report.period_mean_us, 1_000);
    assert_eq!(report.jitter_max_us, 500);
    assert_eq!(report.jitter_mean_us, 250);
}

#[test]
fn constant_drift_shows_in_the_mean_period() {
    let mut timer = LoopTimer::new(PERIOD, 2);
    run(&mut timer, &[0, 1_010, 2_020], 100);
    let report = timer.report().unwrap();
    assert_eq!(report.period_mean_us, 1_010);
    assert_eq!(report.jitter_mean_us, 10);
}

#[test]
fn each_window_reports_on_its_own() {
    let mut timer = LoopTimer::new(PERIOD, 2);
    run(&mut timer, &[0, 1_000, 3_000], 100);
    assert_eq!(timer.report().unwrap().jitter_max_us, 1_000);
    run(&mut timer, &[4_000, 5_000], 100);
    assert_eq!(timer.report().unwrap().jitter_max_us, 0);
}

#[test]
fn timestamps_may_wrap() {
    let start = u32::MAX - 1_500;
    let starts: Vec<u32> = (0..5).map(|i| start.wrapping_add(i * PERIOD)).collect();
    let mut timer = LoopTimer::new(PERIOD, 4);
    run(&mut timer, &starts, 100);
    let report = timer.report().unwrap();
    assert_eq!(report.period_mean_us, 1_000);
    assert_eq!(report.jitter_max_us, 0);
}

#[test]
fn begin_hands_back_the_report_of_the_window_it_closes() {
    let mut timer = LoopTimer::new(PERIOD, 2);
    assert_eq!(timer.begin(0), None);
    assert_eq!(timer.begin(1_000), None);
    let closed = timer.begin(2_100);
    assert_eq!(closed, timer.report());
    assert_eq!(closed.unwrap().jitter_max_us, 100);
    assert_eq!(timer.begin(3_000), None);
}

#[test]
fn report_reads_as_one_line() {
    let report = LoopReport {
        period_us: 10_000,
        samples: 100,
        period_min_us: 9_700,
        period_mean_us: 10_000,
        period_max_us: 10_300,
        jitter_mean_us: 6,
        jitter_max_us: 300,
        busy_max_us: 2_000,
        overruns: 1,
    };
    assert_eq!(
        report.to_string(),
        "period 10000 us over 100 ticks: min 9700 / mean 10000 / max 10300 us, \
         jitter mean 6 / max 300 us, busy max 2000 us, 1 overruns"
    );
}

#[test]
fn end_waits_until_the_deadline() {
    let mut timer = LoopTimer::new(PERIOD, 4);
    timer.begin(0);
    assert_eq!(timer.end(300), Pace::Wait(700));
    timer.begin(1_000);
    assert_eq!(timer.end(1_250), Pace::Wait(750));
}

#[test]
fn deadlines_do_not_drift_with_a_late_wake_up() {
    let mut timer = LoopTimer::new(PERIOD, 4);
    timer.begin(0);
    timer.end(100);
    // Woke up 50 µs late: the next deadline stays at 2 000.
    timer.begin(1_050);
    assert_eq!(timer.end(1_100), Pace::Wait(900));
}

#[test]
fn overrun_restarts_the_schedule_from_now() {
    let mut timer = LoopTimer::new(PERIOD, 2);
    timer.begin(0);
    assert_eq!(timer.end(1_300), Pace::Late(300));
    timer.begin(1_300);
    assert_eq!(timer.end(1_400), Pace::Wait(900));
    timer.begin(2_300);
    let report = timer.report().unwrap();
    assert_eq!(report.overruns, 1);
    assert_eq!(report.busy_max_us, 1_300);
}

#[test]
fn ending_exactly_on_the_deadline_is_an_overrun() {
    let mut timer = LoopTimer::new(PERIOD, 1);
    timer.begin(0);
    assert_eq!(timer.end(1_000), Pace::Late(0));
}

#[test]
fn zero_window_reports_every_period() {
    let mut timer = LoopTimer::new(PERIOD, 0);
    run(&mut timer, &[0, 1_200], 100);
    assert_eq!(timer.report().unwrap().samples, 1);
}

#[test]
fn end_before_any_begin_does_not_panic() {
    let mut timer = LoopTimer::new(PERIOD, 1);
    assert!(matches!(timer.end(500), Pace::Late(_) | Pace::Wait(_)));
    assert_eq!(timer.period_us(), PERIOD);
}
