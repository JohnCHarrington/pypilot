//! Tacking (`tacking.py`): a state machine that takes over from the pilot,
//! turns the boat at a set rate, and hands back on the new heading.

use rustpilot_hal::{Mode, TackDirection};
use rustpilot_types::{Degrees, Duration, Instant, resolv};

/// `ap.tack.state`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TackState {
    /// Normal steering.
    None,
    /// A control link asked for a tack.
    Begin,
    /// Counting down `delay`.
    Waiting,
    /// Turning.
    Tacking,
}

impl TackState {
    /// pypilot's name for it.
    pub const fn name(self) -> &'static str {
        match self {
            TackState::None => "none",
            TackState::Begin => "begin",
            TackState::Waiting => "waiting",
            TackState::Tacking => "tacking",
        }
    }
}

/// A rolling 5-second log that detects which side the wind or heel is on
/// (`TackSensorLog`).
#[derive(Debug, Clone, PartialEq)]
struct SensorLog {
    log: heapless::Deque<f32, 20>,
    time: Instant,
    threshold: f32,
}

impl SensorLog {
    fn new(threshold: f32, now: Instant) -> Self {
        Self {
            log: heapless::Deque::new(),
            time: now,
            threshold,
        }
    }

    fn update(&mut self, value: f32, now: Instant) -> Option<TackDirection> {
        let dt = now.saturating_since(self.time);
        if dt < Duration::from_millis(250) {
            return None;
        }
        self.time = now;
        if dt > Duration::from_millis(1000) {
            self.log.clear();
            return None;
        }
        if !self.log.is_full() {
            let _ = self.log.push_back(value);
            return None;
        }
        self.log.pop_front();
        let _ = self.log.push_back(value);
        let (mut port, mut starboard) = (true, true);
        for &d in self.log.iter() {
            if d <= self.threshold {
                starboard = false;
            }
            if d >= -self.threshold {
                port = false;
            }
        }
        if starboard {
            Some(TackDirection::Starboard)
        } else if port {
            Some(TackDirection::Port)
        } else {
            None
        }
    }
}

/// What tacking reads each step.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TackInputs {
    /// `ap.enabled`.
    pub enabled: bool,
    /// `ap.heading_command`.
    pub heading_command: Degrees,
    /// `imu.heading_lowpass`.
    pub heading: Degrees,
    /// `imu.headingrate_lowpass`.
    pub headingrate: f32,
    /// `imu.headingraterate_lowpass`.
    pub headingraterate: f32,
    /// Apparent wind direction, when a wind source is present.
    pub wind_direction: Option<Degrees>,
}

/// What a tacking step decides.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TackAction {
    /// Not tacking, waiting, or just finished: run the pilot. `Some` sets a
    /// new heading command first (the tack completed).
    Pilot(Option<Degrees>),
    /// Tacking: command the servo directly, overriding the pilot.
    Servo(f32),
}

/// The tack controller (`ap.tack.*`).
#[derive(Debug, Clone, PartialEq)]
pub struct Tack {
    /// `ap.tack.state`.
    pub state: TackState,
    /// Seconds left before the tack starts (`ap.tack.timeout`).
    pub timeout: f32,
    /// Delay before tacking, seconds (0..60).
    pub delay: f32,
    /// Tack angle, degrees (10..180).
    pub angle: f32,
    /// Turn rate, degrees per second (1..100).
    pub rate: f32,
    /// Percent of the turn after which the pilot takes back over (10..100).
    pub threshold: f32,
    /// Tacks completed (`ap.tack.count`).
    pub count: u32,
    /// Next tack direction, from the wind or heel log, or set by a client.
    pub direction: Option<TackDirection>,
    current_direction: TackDirection,
    time: Instant,
    wind_log: SensorLog,
    heel_log: SensorLog,
    tack_angle: f32,
}

impl Tack {
    /// A tack controller with pypilot's defaults.
    pub fn new(now: Instant) -> Self {
        Self {
            state: TackState::None,
            timeout: 0.0,
            delay: 0.0,
            angle: 100.0,
            rate: 15.0,
            threshold: 50.0,
            count: 0,
            direction: None,
            current_direction: TackDirection::Port,
            time: now,
            wind_log: SensorLog::new(20.0, now),
            heel_log: SensorLog::new(7.0, now),
            tack_angle: 100.0,
        }
    }

    /// Request a tack (`ap.tack.state = begin`), optionally setting the
    /// direction first.
    pub fn begin(&mut self, direction: Option<TackDirection>) {
        if direction.is_some() {
            self.direction = direction;
        }
        self.state = TackState::Begin;
    }

    /// Cancel any tack (`ap.tack.state = none`).
    pub fn cancel(&mut self) {
        self.state = TackState::None;
    }

    /// Detect the tack direction from wind or heel (`Tack.poll`).
    ///
    /// pypilot tests its `use_wind_direction` and `use_heel` settings as
    /// objects, which are always true, so in practice the wind log is used
    /// whenever there is wind and the heel log otherwise. RustPilot keeps
    /// that behaviour.
    pub fn poll(&mut self, wind_direction: Option<Degrees>, heel: f32, now: Instant) {
        let r = if let Some(d) = wind_direction {
            self.wind_log.update(resolv(d, 0.0), now)
        } else if now.saturating_since(self.time) > Duration::from_millis(30_000) {
            self.heel_log.update(heel, now)
        } else {
            None
        };
        if r.is_some() {
            self.direction = r;
        }
    }

    /// One step (`Tack.process`).
    pub fn process(&mut self, i: &TackInputs, mode: Mode, now: Instant) -> TackAction {
        if !i.enabled {
            self.state = TackState::None;
            self.direction = None;
        }
        if self.state == TackState::None {
            return TackAction::Pilot(None);
        }

        if self.state == TackState::Begin {
            self.time = now;
            match self.direction {
                None => self.state = TackState::None,
                Some(d) => {
                    self.current_direction = d;
                    self.state = TackState::Waiting;
                }
            }
        }

        if self.state == TackState::Waiting {
            let elapsed = now.saturating_since(self.time).as_secs_f32();
            let remaining = libm::roundf((self.delay - elapsed) * 10.0) / 10.0;
            if remaining > 0.0 {
                self.timeout = remaining;
                return TackAction::Pilot(None);
            }
            self.timeout = 0.0;
            self.state = TackState::Tacking;
            self.tack_angle = self.angle;
        }

        if self.state != TackState::Tacking {
            return TackAction::Pilot(None);
        }

        let command = i.heading_command;
        let (direction, tack_heading, d);
        if mode.is_wind() {
            // steer through the wind angle, which prevents a gybe
            let winddir = resolv(i.wind_direction.unwrap_or(0.0), 0.0);
            tack_heading = -command;
            if command.abs() < 90.0 {
                // tacking
                direction = if command < 0.0 { 1.0 } else { -1.0 };
                d = (1.0 - winddir / command) / 2.0;
            } else {
                // gybing
                direction = if command > 0.0 { 1.0 } else { -1.0 };
                let pcommand = resolv(command, 180.0);
                d = (resolv(winddir, 180.0) - pcommand) / (180.0 - pcommand) / 2.0;
            }
        } else {
            direction = if self.current_direction == TackDirection::Port {
                1.0
            } else {
                -1.0
            };
            tack_heading = command - direction * self.tack_angle;
            d = direction * (command - resolv(i.heading, command)) / self.tack_angle;
        }

        if 100.0 * d > self.threshold {
            self.state = TackState::None;
            self.direction = None;
            self.count += 1;
            return TackAction::Pilot(Some(tack_heading));
        }

        let servo = (i.headingrate + i.headingraterate / 2.0) / self.rate + direction;
        TackAction::Servo(servo.clamp(-1.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn needs_a_direction() {
        let t = Instant::from_micros(0);
        let mut tack = Tack::new(t);
        tack.begin(None);
        let i = TackInputs {
            enabled: true,
            ..Default::default()
        };
        assert_eq!(tack.process(&i, Mode::Compass, t), TackAction::Pilot(None));
        assert_eq!(tack.state, TackState::None);
    }

    #[test]
    fn compass_tack_to_port_completes() {
        let t = Instant::from_micros(0);
        let mut tack = Tack::new(t);
        tack.begin(Some(TackDirection::Port));
        let mut i = TackInputs {
            enabled: true,
            heading_command: 100.0,
            heading: 100.0,
            ..Default::default()
        };
        match tack.process(&i, Mode::Compass, t) {
            TackAction::Servo(c) => assert_eq!(c, 1.0),
            other => panic!("{other:?}"),
        }
        // more than half way round to port: hand back at 100 - 100 = 0
        i.heading = 45.0;
        assert_eq!(
            tack.process(&i, Mode::Compass, t),
            TackAction::Pilot(Some(0.0))
        );
        assert_eq!(tack.count, 1);
        assert_eq!(tack.state, TackState::None);
    }
}
