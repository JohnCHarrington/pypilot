//! Pilots: how the heading error and turn rates become a rudder command
//! (`pilots/pilot.py`, `pilots/basic.py`).
//!
//! Phase 1 ports the basic pilot, pypilot's default. The other pilots (gps,
//! wind, vmg, rate, fuzzy, simple, absolute) follow in phase 5; the
//! learning pilots are skipped for now.

/// One tunable gain (`AutopilotGain`, a persistent, per-profile value).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gain {
    /// Short name (`P`, `D`...), so the full value name is
    /// `ap.pilot.<pilot>.<name>`.
    pub name: &'static str,
    /// Current setting.
    pub value: f32,
    /// Lower limit.
    pub min: f32,
    /// Upper limit.
    pub max: f32,
    /// This gain's contribution to the last command
    /// (`ap.pilot.<pilot>.<name>gain`).
    pub contribution: f32,
}

impl Gain {
    const fn pos(name: &'static str, value: f32, max: f32) -> Self {
        Self {
            name,
            value,
            min: 0.0,
            max,
            contribution: 0.0,
        }
    }

    /// Set the gain, ignoring values outside its range as pypilot's
    /// `RangeProperty` does.
    pub fn set(&mut self, value: f32) -> bool {
        if value >= self.min && value <= self.max {
            self.value = value;
            true
        } else {
            false
        }
    }
}

/// What a pilot reads each step.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PilotInputs {
    /// `ap.heading_error`, degrees.
    pub heading_error: f32,
    /// `imu.headingrate_lowpass`, degrees per second.
    pub headingrate: f32,
    /// `imu.headingraterate_lowpass`, degrees per second squared.
    pub headingraterate: f32,
    /// `ap.heading_command_rate`, the feed-forward term.
    pub heading_command_rate: f32,
}

/// The available pilots (`ap.pilot`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PilotKind {
    /// The extended PID pilot.
    Basic,
}

impl PilotKind {
    /// pypilot's name for it.
    pub const fn name(self) -> &'static str {
        match self {
            PilotKind::Basic => "basic",
        }
    }
}

/// pypilot's basic pilot: an extended PID on heading error.
#[derive(Debug, Clone, PartialEq)]
pub struct BasicPilot {
    /// P, D, DD, PR and FF, in pypilot's order.
    pub gains: [Gain; 5],
}

impl Default for BasicPilot {
    fn default() -> Self {
        Self {
            gains: [
                Gain::pos("P", 0.003, 0.03),  // position (heading error)
                Gain::pos("D", 0.09, 0.24),   // derivative (gyro)
                Gain::pos("DD", 0.075, 0.24), // rate of derivative
                Gain::pos("PR", 0.005, 0.02), // position root
                Gain::pos("FF", 0.6, 2.4),    // feed forward
            ],
        }
    }
}

impl BasicPilot {
    /// Compute the rudder command (`BasicPilot.process` and
    /// `AutopilotPilot.Compute`), updating each gain's contribution.
    pub fn compute(&mut self, i: &PilotInputs) -> f32 {
        let p = i.heading_error;
        let pr = libm::sqrtf(p.abs()).copysign(p);
        let inputs = [
            p,
            i.headingrate,
            i.headingraterate,
            pr,
            i.heading_command_rate,
        ];
        let mut command = 0.0;
        for (gain, value) in self.gains.iter_mut().zip(inputs) {
            gain.contribution = value * gain.value;
            command += gain.contribution;
        }
        command
    }

    /// Look up a gain by short name.
    pub fn gain_mut(&mut self, name: &str) -> Option<&mut Gain> {
        self.gains.iter_mut().find(|g| g.name == name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_sums_terms() {
        let mut p = BasicPilot::default();
        let c = p.compute(&PilotInputs {
            heading_error: -4.0,
            headingrate: 1.0,
            headingraterate: 0.5,
            heading_command_rate: 0.1,
        });
        let want = -4.0 * 0.003 + 0.09 + 0.5 * 0.075 - 2.0 * 0.005 + 0.1 * 0.6;
        assert!((c - want).abs() < 1e-6);
        assert!((p.gains[3].contribution - -0.01).abs() < 1e-7);
    }
}
