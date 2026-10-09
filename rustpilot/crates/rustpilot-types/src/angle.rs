//! Angles in degrees with pypilot's wrap-around rules.

/// An angle in degrees. A plain alias: wrapping is explicit via [`resolv`]
/// and [`wrap_360`], as in pypilot, so callers keep control of the range.
pub type Degrees = f32;

/// Port of pypilot's `resolv.resolv`: shift `angle` by whole turns so it
/// lies in `[offset - 180, offset + 180)`.
///
/// ```
/// use rustpilot_types::resolv;
/// assert_eq!(resolv(350.0, 0.0), -10.0);
/// assert_eq!(resolv(180.0, 0.0), -180.0);
/// assert_eq!(resolv(10.0, 360.0), 370.0);
/// ```
pub fn resolv(angle: Degrees, offset: Degrees) -> Degrees {
    let mut a = angle;
    if !a.is_finite() || !offset.is_finite() {
        return a;
    }
    // The Python loops by 360; do the bulk shift in one step so a huge
    // input can't spin, then let the loops settle the boundary exactly.
    let turns = libm::roundf((offset - a) / 360.0);
    a += turns * 360.0;
    while offset - a > 180.0 {
        a += 360.0;
    }
    while offset - a <= -180.0 {
        a -= 360.0;
    }
    a
}

/// Wrap an angle into `[0, 360)`, the range pypilot reports headings in.
pub fn wrap_360(angle: Degrees) -> Degrees {
    resolv(angle, 180.0)
}

/// Degrees to radians.
pub fn to_radians(d: Degrees) -> f32 {
    d * (core::f32::consts::PI / 180.0)
}

/// Radians to degrees.
pub fn to_degrees(r: f32) -> Degrees {
    r * (180.0 / core::f32::consts::PI)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reference implementation, line for line from pypilot/resolv.py.
    fn resolv_py(mut angle: f32, offset: f32) -> f32 {
        while offset - angle > 180.0 {
            angle += 360.0;
        }
        while offset - angle <= -180.0 {
            angle -= 360.0;
        }
        angle
    }

    #[test]
    fn matches_python() {
        let mut a = -1000.0f32;
        while a < 1000.0 {
            for off in [-360.0, -180.0, -90.0, 0.0, 45.5, 180.0, 359.0] {
                assert_eq!(resolv(a, off), resolv_py(a, off), "a={a} off={off}");
            }
            a += 0.5;
        }
    }

    #[test]
    fn boundaries() {
        assert_eq!(resolv(180.0, 0.0), -180.0);
        assert_eq!(resolv(-180.0, 0.0), -180.0);
        assert_eq!(wrap_360(-10.0), 350.0);
        assert_eq!(wrap_360(360.0), 0.0);
        assert_eq!(wrap_360(725.0), 5.0);
    }

    #[test]
    fn huge_input_terminates() {
        let r = resolv(1.0e7, 0.0);
        assert!((-180.0..180.0).contains(&r));
    }
}
