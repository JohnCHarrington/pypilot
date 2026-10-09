//! Heading command and heading error, ported from `autopilot.py`
//! (`HeadingProperty.set` and `compute_heading_error`).

use rustpilot_hal::Mode;
use rustpilot_types::{Degrees, resolv};

/// pypilot clamps heading error to ±30° before the pilots see it.
pub const MAX_HEADING_ERROR: Degrees = 30.0;

/// Normalise a heading command the way `HeadingProperty.set` does: ±180
/// for wind modes, 0..360 for the others, rounded to a tenth of a degree.
pub fn clamp_heading_command(value: Degrees, mode: Mode) -> Degrees {
    let v = resolv(value, if mode.is_wind() { 0.0 } else { 180.0 });
    // Python's round() is round-half-even; at tenths of a degree the
    // difference from round-half-away is below f32 noise.
    libm::roundf(v * 10.0) / 10.0
}

/// The heading error and its integral (`ap.heading_error`,
/// `ap.heading_error_int`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct HeadingError {
    /// Last error, degrees, clamped to ±30 and sign-reversed in wind modes.
    pub error: Degrees,
    /// Integral, clamped to ±10.
    pub integral: f32,
}

impl HeadingError {
    /// Compute the error for this step and advance the integral by `dt`
    /// seconds (capped at 1 s, as pypilot does). Returns the new error.
    pub fn update(&mut self, heading: Degrees, command: Degrees, mode: Mode, dt: f32) -> Degrees {
        let mut err = resolv(heading - command, 0.0).clamp(-MAX_HEADING_ERROR, MAX_HEADING_ERROR);
        // Wind direction is where the wind comes from, so the sign flips.
        if mode.is_wind() {
            err = -err;
        }
        self.error = err;

        let dt = dt.min(1.0);
        self.integral = (self.integral + (err / 50.0) * dt).clamp(-10.0, 10.0);
        err
    }

    /// Zero the integral, as pypilot does on a mode change.
    pub fn reset_integral(&mut self) {
        self.integral = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_ranges() {
        assert_eq!(clamp_heading_command(-10.0, Mode::Compass), 350.0);
        assert_eq!(clamp_heading_command(370.04, Mode::Gps), 10.0);
        assert_eq!(clamp_heading_command(200.0, Mode::Wind), -160.0);
        assert_eq!(clamp_heading_command(180.0, Mode::TrueWind), -180.0);
    }

    #[test]
    fn error_clamps_and_wraps() {
        let mut e = HeadingError::default();
        assert_eq!(e.update(350.0, 10.0, Mode::Compass, 0.1), -20.0);
        assert_eq!(e.update(100.0, 10.0, Mode::Compass, 0.1), 30.0);
        assert_eq!(e.update(100.0, 10.0, Mode::Wind, 0.1), -30.0);
    }

    #[test]
    fn integral_caps_dt_and_value() {
        let mut e = HeadingError::default();
        e.update(30.0, 0.0, Mode::Compass, 5.0);
        assert!((e.integral - 0.6).abs() < 1e-6); // 30/50 * min(5, 1)
        for _ in 0..100 {
            e.update(30.0, 0.0, Mode::Compass, 1.0);
        }
        assert_eq!(e.integral, 10.0);
    }
}
