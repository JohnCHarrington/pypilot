//! Rudder angle from a raw sensor reading, with pypilot's offset, scale and
//! nonlinearity calibration (`rudder.py`).
//!
//! Every rudder source goes through this calibration in pypilot, including
//! NMEA RSA, so RustPilot does the same.

use rustpilot_hal::DeviceId;
use rustpilot_types::{Degrees, Instant};

use crate::sensors::SensorSlot;

/// Calibration steps (`rudder.calibration_state`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationStep {
    /// Reset to scale 100, offset 0, no nonlinearity.
    Reset,
    /// The rudder is centred now.
    Centered,
    /// The rudder is at the starboard stop now.
    StarboardRange,
    /// The rudder is at the port stop now.
    PortRange,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CalPoint {
    raw: f32,
    rudder: f32,
}

/// The rudder sensor.
#[derive(Debug, Clone, PartialEq)]
pub struct Rudder {
    /// Arbitration state.
    pub slot: SensorSlot,
    /// Calibrated angle, degrees; `None` when invalid (`rudder.angle` False).
    pub angle: Option<Degrees>,
    /// Low-passed rudder speed, degrees per second.
    pub speed: f32,
    /// Calibration offset, degrees.
    pub offset: f32,
    /// Calibration scale, degrees per unit raw.
    pub scale: f32,
    /// Calibration nonlinearity.
    pub nonlinearity: f32,
    /// Rudder range either side of centre, degrees (`rudder.range`).
    pub range: f32,
    /// Raw readings at the calibrated range ends, −0.5..0.5.
    pub minmax: (f32, f32),
    /// Last raw reading.
    pub raw: f32,
    last: f32,
    last_time: Instant,
    lastrange: Option<f32>,
    lastdevice: Option<DeviceId>,
    // starboard range, centered, port range
    points: [Option<CalPoint>; 3],
}

impl Rudder {
    /// A rudder with pypilot's default calibration.
    pub fn new(now: Instant) -> Self {
        Self {
            slot: SensorSlot::default(),
            angle: None,
            speed: 0.0,
            offset: 0.0,
            scale: 100.0,
            nonlinearity: 0.0,
            range: 45.0,
            minmax: (-0.5, 0.5),
            raw: 0.0,
            last: 0.0,
            last_time: now,
            lastrange: None,
            lastdevice: None,
            points: [None; 3],
        }
    }

    /// True when there is no valid angle (`Rudder.invalid`).
    pub fn invalid(&self) -> bool {
        self.angle.is_none()
    }

    /// Recompute the raw range ends, rescaling the calibration when the
    /// range setting changed (`update_minmax`).
    pub fn update_minmax(&mut self) {
        let (scale, offset, range) = (self.scale, self.offset, self.range);
        let old = self.minmax;
        self.minmax = ((-range - offset) / scale, (range - offset) / scale);
        if self.lastrange.is_some_and(|r| r != range) {
            let n = self.nonlinearity;
            let b = scale - n * (old.0 + old.1);
            let c = offset + n * old.0 * old.1;
            let (min, max) = self.minmax;
            self.scale = b + n * (min + max);
            self.offset = c - n * min * max;
        }
        self.lastrange = Some(range);
    }

    /// Per-loop housekeeping (`Rudder.poll`, without the disabled auto-gain).
    pub fn poll(&mut self) {
        if self.lastrange.is_some_and(|r| r != self.range) {
            self.update_minmax();
        }
    }

    /// Apply one calibration step at the current raw reading
    /// (`Rudder.calibration`).
    pub fn calibrate(&mut self, step: CalibrationStep) {
        let (index, true_angle) = match step {
            CalibrationStep::Reset => {
                self.nonlinearity = 0.0;
                self.scale = 100.0;
                self.offset = 0.0;
                self.update_minmax();
                self.points = [None; 3];
                return;
            }
            CalibrationStep::StarboardRange => (0, -self.range),
            CalibrationStep::Centered => (1, 0.0),
            CalibrationStep::PortRange => (2, self.range),
        };
        self.points[index] = Some(CalPoint {
            raw: self.raw,
            rudder: true_angle,
        });

        let (mut scale, mut offset, mut nonlinearity) =
            (self.scale, self.offset, self.nonlinearity);
        let mut p: heapless::Vec<CalPoint, 3> = heapless::Vec::new();
        for pt in self.points.iter().flatten() {
            let _ = p.push(*pt);
        }
        match p.len() {
            1 => {
                let (rudder, raw) = (p[0].rudder, p[0].raw);
                offset = rudder
                    - scale * raw
                    - nonlinearity * (self.minmax.0 - raw) * (self.minmax.1 - raw);
            }
            2 => {
                if (p[1].raw - p[0].raw).abs() > 0.001 {
                    scale = (p[1].rudder - p[0].rudder) / (p[1].raw - p[0].raw);
                }
                offset = p[1].rudder - scale * p[1].raw;
                nonlinearity = 0.0;
            }
            3 => {
                let (r0, r1, r2) = (p[0].raw, p[1].raw, p[2].raw);
                let min_gap = (r1 - r0).abs().min((r2 - r0).abs()).min((r2 - r1).abs());
                if min_gap > 0.001 {
                    scale = (p[2].rudder - p[0].rudder) / (r2 - r0);
                    offset = p[0].rudder - scale * r0;
                    nonlinearity = (p[1].rudder - scale * r1 - offset) / (r0 - r1) / (r2 - r1);
                }
            }
            _ => {}
        }

        if scale.abs() <= 0.01 {
            // bad update: keep only this step's point
            for (i, pt) in self.points.iter_mut().enumerate() {
                if i != index {
                    *pt = None;
                }
            }
            return;
        }
        self.offset = offset;
        self.scale = scale;
        self.nonlinearity = nonlinearity;
        self.update_minmax();
    }

    /// Apply a raw reading from `device` (`Rudder.update`). Returns whether
    /// it was accepted. The first reading from a new device is dropped, as
    /// pypilot does, to avoid echoing a rudder angle back from where it was
    /// sent.
    pub fn update(&mut self, raw: Option<f32>, device: &DeviceId, now: Instant) -> bool {
        let Some(raw) = raw.filter(|r| !r.is_nan()) else {
            self.angle = None;
            return false;
        };
        if self.lastdevice.as_ref() != Some(device) {
            self.lastdevice = Some(device.clone());
            self.angle = None;
            return false;
        }
        self.raw = raw;
        let (min, max) = self.minmax;
        let angle = self.scale * raw + self.offset + self.nonlinearity * (min - raw) * (max - raw);
        let angle = libm::roundf(angle * 100.0) / 100.0;
        self.angle = Some(angle);

        let dt = now.saturating_since(self.last_time).as_secs_f32().min(1.0);
        if dt > 0.0 {
            let speed = (angle - self.last) / dt;
            self.last_time = now;
            self.last = angle;
            self.speed = 0.9 * self.speed + 0.1 * speed;
        }
        true
    }

    /// Forget the angle when the source is lost.
    pub fn reset(&mut self) {
        self.angle = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_scale_and_first_reading_dropped() {
        let t = Instant::from_micros(0);
        let d = DeviceId::try_from("servo").unwrap();
        let mut r = Rudder::new(t);
        r.update_minmax();
        assert!(!r.update(Some(0.1), &d, t));
        assert!(r.update(Some(0.1), &d, t));
        assert_eq!(r.angle, Some(10.0));
    }

    #[test]
    fn three_point_calibration() {
        let t = Instant::from_micros(0);
        let mut r = Rudder::new(t);
        r.update_minmax();
        // a sensor reading -0.3 at starboard, 0.05 centred, 0.4 at port
        for (raw, step) in [
            (-0.3, CalibrationStep::StarboardRange),
            (0.05, CalibrationStep::Centered),
            (0.4, CalibrationStep::PortRange),
        ] {
            r.raw = raw;
            r.calibrate(step);
        }
        let d = DeviceId::try_from("servo").unwrap();
        r.update(Some(0.05), &d, t);
        for (raw, want) in [(-0.3f32, -45.0f32), (0.05, 0.0), (0.4, 45.0)] {
            r.update(Some(raw), &d, t);
            assert!(
                (r.angle.unwrap() - want).abs() < 0.05,
                "raw {raw}: {:?}",
                r.angle
            );
        }
    }
}
