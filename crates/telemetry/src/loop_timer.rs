//! Fixed-rate control loop: pacing on an absolute schedule, and its timing over windows of ticks.
//!
//! Call [`LoopTimer::begin`] at the top of each tick and [`LoopTimer::end`] once the work is done,
//! then wait what [`Pace::Wait`] says. Deadlines advance by one period from the previous one, so
//! the loop does not drift by the time spent between ticks; after an overrun the schedule restarts
//! from the late instant instead of rushing to catch up.
//!
//! Jitter is how far each period (time between two consecutive `begin`) lands from the nominal
//! one. A constant offset shows in `period_mean_us`.
//!
//! [`LoopTimer::begin`] hands back each window's [`LoopReport`] as it closes, for a caller that logs
//! it; [`LoopTimer::report`] keeps the last one for a caller that polls.
//!
//! Timestamps come from any free-running microsecond clock and may wrap: every delta uses
//! `wrapping_sub`, so a single period or window must stay under ~71 min.

/// Timing of the last complete window.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LoopReport {
    /// Nominal period the loop is paced at.
    pub period_us: u32,
    /// Periods measured in the window.
    pub samples: u32,
    pub period_min_us: u32,
    pub period_mean_us: u32,
    pub period_max_us: u32,
    /// Mean of |period - nominal|.
    pub jitter_mean_us: u32,
    /// Largest |period - nominal|.
    pub jitter_max_us: u32,
    /// Longest time between `begin` and `end`.
    pub busy_max_us: u32,
    /// Ticks that ended past their deadline.
    pub overruns: u32,
}

impl core::fmt::Display for LoopReport {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "period {} us over {} ticks: min {} / mean {} / max {} us, jitter mean {} / max {} us, busy max {} us, {} overruns",
            self.period_us,
            self.samples,
            self.period_min_us,
            self.period_mean_us,
            self.period_max_us,
            self.jitter_mean_us,
            self.jitter_max_us,
            self.busy_max_us,
            self.overruns,
        )
    }
}

/// What to do after a tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pace {
    /// On time: wait this long before the next tick.
    Wait(u32),
    /// Past the deadline by this much: start the next tick now.
    Late(u32),
}

#[derive(Clone, Copy, Debug)]
struct Window {
    start: u32,
    samples: u32,
    period_min: u32,
    period_max: u32,
    deviation_sum: u32,
    deviation_max: u32,
    busy_max: u32,
    overruns: u32,
}

impl Window {
    const fn new(start: u32) -> Self {
        Self {
            start,
            samples: 0,
            period_min: u32::MAX,
            period_max: 0,
            deviation_sum: 0,
            deviation_max: 0,
            busy_max: 0,
            overruns: 0,
        }
    }

    /// Only called with at least one sample.
    fn report(&self, period_us: u32, now: u32) -> LoopReport {
        LoopReport {
            period_us,
            samples: self.samples,
            period_min_us: self.period_min,
            period_mean_us: now.wrapping_sub(self.start) / self.samples,
            period_max_us: self.period_max,
            jitter_mean_us: self.deviation_sum / self.samples,
            jitter_max_us: self.deviation_max,
            busy_max_us: self.busy_max,
            overruns: self.overruns,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LoopTimer {
    period_us: u32,
    window_len: u32,
    started: Option<u32>,
    deadline: u32,
    window: Window,
    report: Option<LoopReport>,
}

impl LoopTimer {
    /// `window_len` periods per [`LoopReport`]; 0 counts as 1.
    pub const fn new(period_us: u32, window_len: u32) -> Self {
        Self {
            period_us,
            window_len: if window_len == 0 { 1 } else { window_len },
            started: None,
            deadline: 0,
            window: Window::new(0),
            report: None,
        }
    }

    pub const fn period_us(&self) -> u32 {
        self.period_us
    }

    /// The last complete window, `None` until the first one closes.
    pub const fn report(&self) -> Option<LoopReport> {
        self.report
    }

    /// Start of a tick. Returns the report of the window this tick closes, if any.
    pub fn begin(&mut self, now_us: u32) -> Option<LoopReport> {
        let closed = match self.started.replace(now_us) {
            None => {
                self.window = Window::new(now_us);
                self.deadline = now_us;
                None
            }
            Some(previous) => self.record_period(now_us.wrapping_sub(previous), now_us),
        };
        self.deadline = self.deadline.wrapping_add(self.period_us);
        closed
    }

    /// End of a tick's work.
    pub fn end(&mut self, now_us: u32) -> Pace {
        let busy = self.started.map_or(0, |start| now_us.wrapping_sub(start));
        self.window.busy_max = self.window.busy_max.max(busy);

        let remaining = self.deadline.wrapping_sub(now_us);
        if (remaining as i32) > 0 {
            return Pace::Wait(remaining);
        }
        self.window.overruns += 1;
        self.deadline = now_us;
        Pace::Late(remaining.wrapping_neg())
    }

    fn record_period(&mut self, period: u32, now: u32) -> Option<LoopReport> {
        let deviation = period.abs_diff(self.period_us);
        let window = &mut self.window;
        window.samples += 1;
        window.period_min = window.period_min.min(period);
        window.period_max = window.period_max.max(period);
        window.deviation_sum = window.deviation_sum.saturating_add(deviation);
        window.deviation_max = window.deviation_max.max(deviation);

        if window.samples < self.window_len {
            return None;
        }
        self.report = Some(window.report(self.period_us, now));
        self.window = Window::new(now);
        self.report
    }
}
