//! Sensor arbitration and the per-sensor filters, ported from `sensors.py`.
//!
//! Each sensor keeps the source and device it accepted data from. Data from a
//! lower-priority source is ignored, and so is data from a second device at
//! the same priority, so readings never flip between two instruments.
//! A sensor that hears nothing for 8 seconds is lost and resets.

use rustpilot_hal::DeviceId;
use rustpilot_types::{Degrees, Duration, Instant, SensorSource, resolv};

/// A sensor with no data for this long is lost (`Sensors.poll`).
pub const SENSOR_TIMEOUT: Duration = Duration::from_millis(8000);

/// Which source and device a sensor is following (`Sensor.write`).
#[derive(Debug, Clone, PartialEq)]
pub struct SensorSlot {
    /// Current source; `None` when lost.
    pub source: SensorSource,
    /// Device within that source.
    pub device: Option<DeviceId>,
    /// Last accepted update.
    pub lastupdate: Instant,
}

impl Default for SensorSlot {
    fn default() -> Self {
        Self {
            source: SensorSource::None,
            device: None,
            lastupdate: Instant::default(),
        }
    }
}

impl SensorSlot {
    /// Whether data from `source`/`device` may update this sensor.
    pub fn accepts(&self, source: SensorSource, device: &DeviceId) -> bool {
        let (have, new) = (self.source.priority(), source.priority());
        if have < new {
            return false;
        }
        if have == new && self.device.as_ref() != Some(device) {
            return false;
        }
        true
    }

    /// Record an accepted update.
    pub fn commit(&mut self, source: SensorSource, device: &DeviceId, now: Instant) {
        if self.source != source {
            self.source = source;
            self.device = Some(device.clone());
        }
        self.lastupdate = now;
    }

    /// True when a source is set.
    pub fn present(&self) -> bool {
        self.source != SensorSource::None
    }

    /// True when the source has gone quiet for [`SENSOR_TIMEOUT`].
    pub fn timed_out(&self, now: Instant) -> bool {
        self.present() && now.saturating_since(self.lastupdate) > SENSOR_TIMEOUT
    }

    /// Forget the source (`Sensors.lostsensor`).
    pub fn lose(&mut self) {
        self.source = SensorSource::None;
        self.device = None;
    }
}

/// Apparent or true wind with pypilot's direction filter (`BaseWind`).
#[derive(Debug, Clone, PartialEq)]
pub struct Wind {
    /// Arbitration state.
    pub slot: SensorSlot,
    /// Direction off the bow, ±180 (`wind.direction`).
    pub direction: Option<Degrees>,
    /// Knots (`wind.speed`).
    pub speed: Option<f32>,
    /// Sensor mounting offset, degrees (`wind.offset`).
    pub offset: Degrees,
    /// Mast height for motion compensation, metres (`wind.sensors_height`).
    pub compensation_height: f32,
    /// Low-passed speed (`wind.filtered_speed`).
    pub filtered_speed: f32,
    /// Low-passed direction (`wind.filtered_direction`).
    pub filtered_direction: Degrees,
    /// Filter constant (`wind.filter_constant`).
    pub filter_constant: f32,
    /// Last filter weight (`wind.filter_factor`).
    pub filter_factor: f32,
}

impl Default for Wind {
    fn default() -> Self {
        Self {
            slot: SensorSlot::default(),
            direction: None,
            speed: None,
            offset: 0.0,
            compensation_height: 0.0,
            filtered_speed: 0.0,
            // pypilot starts this at False, which arithmetic treats as 0.
            filtered_direction: 0.0,
            filter_constant: 0.1,
            filter_factor: 0.0,
        }
    }
}

impl Wind {
    /// Apply a reading (`BaseWind.update`). `rates` are the IMU's
    /// `(rollrate, pitchrate)` in degrees per second, used for masthead
    /// motion compensation.
    pub fn update(&mut self, direction: Option<Degrees>, speed: Option<f32>, rates: (f32, f32)) {
        let mut direction = direction.map(|d| d + self.offset);
        let mut speed = speed;
        if self.compensation_height != 0.0
            && let (Some(d), Some(s)) = (direction, speed)
        {
            let r = d.to_radians();
            let mut dx = s * libm::sinf(r);
            let mut dy = s * libm::cosf(r);
            // degrees/s to rad/s, then m/s to knots, times mast height
            let m = 1.0f32.to_radians() * 1.94 * self.compensation_height;
            dx -= m * rates.0;
            dy -= m * rates.1;
            speed = Some(libm::hypotf(dx, dy));
            direction = Some(libm::atan2f(dx, dy).to_degrees());
        }
        if let Some(d) = direction {
            self.direction = Some(resolv(d, 0.0));
        }
        if let Some(s) = speed {
            self.speed = Some(s);
        }
        self.weight();
    }

    /// Update the low-pass filters (`BaseWind.weight`).
    pub fn weight(&mut self) {
        let speed = self.speed.unwrap_or(0.0);
        let d = 0.01;
        self.filtered_speed = (1.0 - d) * self.filtered_speed + d * speed;
        // weight wind direction more with higher wind speed
        let d = self.filter_constant * libm::logf(self.filtered_speed / 5.0 + 1.1);
        let direction = resolv(self.direction.unwrap_or(0.0), self.filtered_direction);
        let filtered = (1.0 - d) * self.filtered_direction + d * direction;
        self.filtered_direction = resolv(filtered, 0.0);
        self.filter_factor = d;
    }

    /// Forget readings when the source is lost.
    pub fn reset(&mut self) {
        self.direction = None;
        self.speed = None;
    }

    /// Synthesise true wind from apparent wind and boat speed
    /// (`TrueWind.update_from_apparent`). Only applies while true wind has
    /// no explicit source, so an instrument's own true wind always wins.
    pub fn update_from_apparent(
        &mut self,
        boat_speed: f32,
        wind_speed: f32,
        wind_direction: Degrees,
        now: Instant,
    ) {
        if !matches!(
            self.slot.source,
            SensorSource::WaterWind | SensorSource::GpsWind | SensorSource::None
        ) {
            return;
        }
        let (direction, speed) = true_wind(boat_speed, wind_speed, wind_direction);
        self.direction = Some(direction);
        self.speed = Some(speed);
        self.weight();
        self.slot.lastupdate = now;
    }
}

/// True wind `(direction, speed)` from boat speed and apparent wind.
pub fn true_wind(boat_speed: f32, wind_speed: f32, wind_direction: Degrees) -> (Degrees, f32) {
    let rd = wind_direction.to_radians();
    let x = wind_speed * libm::sinf(rd);
    let y = wind_speed * libm::cosf(rd) - boat_speed;
    (libm::atan2f(x, y).to_degrees(), libm::hypotf(x, y))
}

/// GPS (`gps.*`). The GPS Kalman filter and compass alignment arrive with
/// the IMU work in phase 2; the control loop only reads speed and track.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Gps {
    /// Arbitration state.
    pub slot: SensorSlot,
    /// Course over ground, degrees true.
    pub track: Option<Degrees>,
    /// Knots.
    pub speed: Option<f32>,
    /// Magnetic declination, degrees.
    pub declination: Option<Degrees>,
}

impl Gps {
    /// Apply a fix (`gps.update`).
    pub fn update(&mut self, fix: &rustpilot_types::GpsFix) {
        self.speed = Some(fix.speed);
        if let Some(t) = fix.track {
            self.track = Some(t);
        }
        if let Some(d) = fix.declination {
            self.declination = Some(d);
        }
    }

    /// Forget readings when the source is lost.
    pub fn reset(&mut self) {
        self.track = None;
        self.speed = None;
    }
}

/// Speed through water (`water.*`). Leeway and current estimation are
/// disabled in pypilot, so they are not ported.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Water {
    /// Arbitration state.
    pub slot: SensorSlot,
    /// Knots.
    pub speed: Option<f32>,
    /// Measured leeway, degrees.
    pub leeway: Option<Degrees>,
}

impl Water {
    /// Apply a reading.
    pub fn update(&mut self, w: &rustpilot_types::WaterSpeed) {
        self.speed = Some(w.speed);
        if let Some(l) = w.leeway {
            self.leeway = Some(l);
        }
    }

    /// Forget readings when the source is lost.
    pub fn reset(&mut self) {
        self.speed = None;
        self.leeway = None;
    }
}

/// Route steering from a plotter (`apb.*`).
#[derive(Debug, Clone, PartialEq)]
pub struct Apb {
    /// Arbitration state.
    pub slot: SensorSlot,
    /// Bearing to steer, degrees.
    pub track: Option<Degrees>,
    /// Cross-track error, nautical miles.
    pub xte: f32,
    /// Degrees of correction per nautical mile of XTE (`apb.xte.gain`);
    /// 300 is 30° for a tenth of a mile.
    pub gain: f32,
    last_time: Instant,
}

impl Apb {
    /// A new APB sensor at time `now`.
    pub fn new(now: Instant) -> Self {
        Self {
            slot: SensorSlot::default(),
            track: None,
            xte: 0.0,
            gain: 300.0,
            last_time: now,
        }
    }

    /// Apply a route reading (`APB.update`). Returns `None` when the update
    /// was rate-limited (pypilot accepts 2 Hz) and so not accepted at all;
    /// otherwise `Some(command)`, where `command` is the heading command to
    /// set if the autopilot is engaged in nav mode.
    pub fn update(
        &mut self,
        route: &rustpilot_types::RouteSteer,
        now: Instant,
    ) -> Option<Option<Degrees>> {
        if now.saturating_since(self.last_time) < Duration::from_millis(500) {
            return None;
        }
        self.last_time = now;
        self.track = Some(route.track);
        self.xte = route.xte;
        // pypilot only steers to APB in "gps" (true) mode messages.
        if route.magnetic {
            return None;
        }
        Some(Some(route.track + self.gain * route.xte))
    }

    /// Forget readings when the source is lost.
    pub fn reset(&mut self) {
        self.xte = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dev(s: &str) -> DeviceId {
        DeviceId::try_from(s).unwrap()
    }

    #[test]
    fn priority_and_device_lock() {
        let mut s = SensorSlot::default();
        let t = Instant::from_micros(0);
        assert!(s.accepts(SensorSource::Signalk, &dev("sk")));
        s.commit(SensorSource::Signalk, &dev("sk"), t);
        // a better source takes over
        assert!(s.accepts(SensorSource::Serial, &dev("/dev/ttyUSB0")));
        s.commit(SensorSource::Serial, &dev("/dev/ttyUSB0"), t);
        // a worse one, or a second device at the same priority, does not
        assert!(!s.accepts(SensorSource::Tcp, &dev("tcp")));
        assert!(!s.accepts(SensorSource::Serial, &dev("/dev/ttyUSB1")));
        assert!(s.accepts(SensorSource::Serial, &dev("/dev/ttyUSB0")));
        assert!(!s.timed_out(t + Duration::from_millis(8000)));
        assert!(s.timed_out(t + Duration::from_millis(8001)));
    }

    #[test]
    fn true_wind_head_on() {
        let (d, s) = true_wind(5.0, 15.0, 0.0);
        assert_eq!(d, 0.0);
        assert!((s - 10.0).abs() < 1e-5);
    }
}
