//! Connector family 2: control links (pypilot JSON over WiFi/TCP, NMEA 2000
//! 127237, switch bank and `PP:` text commands, later BLE or a remote).
//!
//! Every link produces the same [`ControlCommand`]s and is gated by the same
//! [`ControlLevel`]s, taken from `docs/n2k-control.md` §6 on the n2k branch.

use rustpilot_types::{Degrees, Instant};

use crate::{ConnError, DeviceId};

/// Maximum length of a pypilot value name (`ap.pilot.basic.P`).
pub const VALUE_NAME_LEN: usize = 48;
/// Maximum length of a value's JSON text in a `SetValue` command.
pub const VALUE_TEXT_LEN: usize = 96;

/// pypilot's steering modes (`ap.mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Magnetic heading from the IMU.
    Compass,
    /// Course over ground.
    Gps,
    /// Follow a route from APB / 129284.
    Nav,
    /// Apparent wind angle.
    Wind,
    /// True wind angle.
    TrueWind,
}

impl Mode {
    /// All modes in pypilot's order.
    pub const ALL: [Mode; 5] = [
        Mode::Compass,
        Mode::Gps,
        Mode::Nav,
        Mode::Wind,
        Mode::TrueWind,
    ];

    /// The string pypilot uses for `ap.mode`.
    pub const fn name(self) -> &'static str {
        match self {
            Mode::Compass => "compass",
            Mode::Gps => "gps",
            Mode::Nav => "nav",
            Mode::Wind => "wind",
            Mode::TrueWind => "true wind",
        }
    }

    /// Parse pypilot's `ap.mode` string.
    pub fn from_name(s: &str) -> Option<Mode> {
        Mode::ALL.into_iter().find(|m| m.name() == s)
    }

    /// Wind modes steer an angle in ±180; the others a heading in 0..360.
    pub const fn is_wind(self) -> bool {
        matches!(self, Mode::Wind | Mode::TrueWind)
    }
}

/// A set of modes (`ap.modes`, the modes the current sensors allow).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct ModeSet(u8);

impl ModeSet {
    /// The empty set.
    pub const EMPTY: ModeSet = ModeSet(0);

    const fn bit(m: Mode) -> u8 {
        1 << (m as u8)
    }

    /// Whether `m` is in the set.
    pub const fn contains(self, m: Mode) -> bool {
        self.0 & Self::bit(m) != 0
    }

    /// Add `m`.
    pub fn insert(&mut self, m: Mode) {
        self.0 |= Self::bit(m);
    }

    /// The modes in pypilot's order.
    pub fn iter(self) -> impl Iterator<Item = Mode> {
        Mode::ALL.into_iter().filter(move |m| self.contains(*m))
    }
}

/// How much a control link may do. Ordered: each level includes the ones
/// below it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ControlLevel {
    /// Nothing is accepted, not even standby.
    Off,
    /// Status and reads only, but standby is always honoured, like a STBY
    /// key on any head.
    Monitor,
    /// Adds engage, mode, heading, tack and manual steering.
    Steer,
    /// Adds writes to any pypilot value.
    Full,
}

/// Tack direction (`ap.tack.direction`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TackDirection {
    /// Tack to port.
    Port,
    /// Tack to starboard.
    Starboard,
}

/// A command from any control link.
#[derive(Debug, Clone, PartialEq)]
pub enum ControlCommand {
    /// Disengage (`ap.enabled = false`).
    Standby,
    /// Engage, optionally switching mode first. `None` keeps the current
    /// mode.
    Engage(Option<Mode>),
    /// Change mode without changing engaged state.
    SetMode(Mode),
    /// Set the heading command (`ap.heading_command`), in the current
    /// mode's frame.
    SetHeading(Degrees),
    /// Nudge the heading command by a relative amount (±1°, ±10°).
    AdjustHeading(Degrees),
    /// Start a tack.
    Tack(TackDirection),
    /// Manual non-follow-up steering: run the motor at `speed` (−1..1) for
    /// one pulse, ended by the core's own timer. Disengages first.
    Jog {
        /// Signed motor speed, positive to port as in pypilot's
        /// `servo.command`.
        speed: f32,
    },
    /// Manual follow-up steering: drive the rudder to an angle.
    /// Disengages first.
    RudderAngle(Degrees),
    /// Write any pypilot value by name, with its value as JSON text, as the
    /// pypilot protocol and `PP:` commands do.
    SetValue {
        /// Value name, such as `ap.pilot.basic.P`.
        name: heapless::String<VALUE_NAME_LEN>,
        /// JSON text of the new value.
        value: heapless::String<VALUE_TEXT_LEN>,
    },
}

impl ControlCommand {
    /// The lowest [`ControlLevel`] at which a link may issue this command.
    pub const fn required_level(&self) -> ControlLevel {
        match self {
            ControlCommand::Standby => ControlLevel::Monitor,
            ControlCommand::SetValue { .. } => ControlLevel::Full,
            _ => ControlLevel::Steer,
        }
    }

    /// Whether a link at `level` may issue this command.
    pub fn permitted_at(&self, level: ControlLevel) -> bool {
        level >= self.required_level()
    }
}

/// A command with the link and device it came from, so the core can apply
/// allow-lists and report who changed what.
#[derive(Debug, Clone, PartialEq)]
pub struct ControlRequest {
    /// The device that sent it (an N2K NAME as text, a TCP peer address).
    pub device: DeviceId,
    /// When it arrived.
    pub time: Instant,
    /// The command.
    pub command: ControlCommand,
}

/// The autopilot state every control link publishes (127237, pypilot
/// watches, switch-bank status).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PilotState {
    /// Engaged (`ap.enabled`).
    pub enabled: bool,
    /// Current mode (`ap.mode`); `None` before the first step.
    pub mode: Option<Mode>,
    /// Heading command in the mode's frame (`ap.heading_command`).
    pub heading_command: Degrees,
    /// Current heading in the mode's frame (`ap.heading`).
    pub heading: Degrees,
    /// Heading error (`ap.heading_error`).
    pub heading_error: Degrees,
    /// Low-passed compass heading (`imu.heading_lowpass`).
    pub compass_heading: Degrees,
    /// Calibrated rudder angle, if a rudder sensor is present.
    pub rudder_angle: Option<Degrees>,
    /// Modes the current sensors allow (`ap.modes`).
    pub modes: ModeSet,
    /// Servo flags (`servo.flags`), see `rustpilot_core::servo::flags`.
    pub servo_flags: u32,
}

/// A link that accepts commands and publishes state.
pub trait ControlLink {
    /// This link's configured control level. The core enforces it; links
    /// may also refuse early (an N2K NAK) using
    /// [`ControlCommand::permitted_at`].
    fn level(&self) -> ControlLevel;

    /// Wait for the next command.
    async fn recv(&mut self) -> Result<ControlRequest, ConnError>;

    /// Publish the current state. Links rate-limit as their protocol needs.
    async fn publish(&mut self, state: &PilotState) -> Result<(), ConnError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels() {
        assert!(!ControlCommand::Standby.permitted_at(ControlLevel::Off));
        assert!(ControlCommand::Standby.permitted_at(ControlLevel::Monitor));
        assert!(!ControlCommand::Engage(None).permitted_at(ControlLevel::Monitor));
        assert!(ControlCommand::Engage(None).permitted_at(ControlLevel::Steer));
        let set = ControlCommand::SetValue {
            name: "ap.pilot.basic.P".try_into().unwrap(),
            value: "0.003".try_into().unwrap(),
        };
        assert!(!set.permitted_at(ControlLevel::Steer));
        assert!(set.permitted_at(ControlLevel::Full));
    }

    #[test]
    fn mode_names() {
        for m in Mode::ALL {
            assert_eq!(Mode::from_name(m.name()), Some(m));
        }
        assert_eq!(Mode::from_name("true wind"), Some(Mode::TrueWind));
    }
}
