//! A monotonic time base the core can use without `std`.
//!
//! The core never reads a clock: platforms pass `now` into
//! `Autopilot::step`, which keeps the loop deterministic under replay.

use core::ops::{Add, Sub};

/// A span of time in microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Hash)]
pub struct Duration(pub u64);

impl Duration {
    /// From whole milliseconds.
    pub const fn from_millis(ms: u64) -> Self {
        Self(ms * 1000)
    }
    /// From microseconds.
    pub const fn from_micros(us: u64) -> Self {
        Self(us)
    }
    /// From seconds as a float, clamping negatives to zero.
    pub fn from_secs_f32(s: f32) -> Self {
        Self(if s > 0.0 { (s * 1.0e6) as u64 } else { 0 })
    }
    /// Seconds as `f32`.
    pub fn as_secs_f32(self) -> f32 {
        self.0 as f32 * 1.0e-6
    }
    /// Microseconds.
    pub const fn as_micros(self) -> u64 {
        self.0
    }
}

/// A point on a monotonic clock, in microseconds since an arbitrary epoch
/// (boot, or the start of a replayed log).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Hash)]
pub struct Instant(pub u64);

impl Instant {
    /// From microseconds since the epoch.
    pub const fn from_micros(us: u64) -> Self {
        Self(us)
    }
    /// From seconds as a float, for replaying pypilot logs.
    pub fn from_secs_f64(s: f64) -> Self {
        Self(if s > 0.0 { (s * 1.0e6) as u64 } else { 0 })
    }
    /// Time since `earlier`, or zero if `earlier` is later.
    pub fn saturating_since(self, earlier: Instant) -> Duration {
        Duration(self.0.saturating_sub(earlier.0))
    }
}

impl Add<Duration> for Instant {
    type Output = Instant;
    fn add(self, d: Duration) -> Instant {
        Instant(self.0.saturating_add(d.0))
    }
}

impl Sub for Instant {
    type Output = Duration;
    fn sub(self, earlier: Instant) -> Duration {
        self.saturating_since(earlier)
    }
}
