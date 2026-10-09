//! RustPilot's autopilot core.
//!
//! A pure state machine: platforms read their connectors, pass the latest
//! readings in [`Inputs`], call [`Autopilot::step`] once per IMU sample, and
//! write the returned [`Outputs`] to the motor controller and control links.
//! Nothing here performs I/O or reads a clock, so the same code runs on
//! Linux, ESP32, in the simulator and under log replay.
//!
//! Phase 0 holds the loop's shape and pypilot's heading-error rules; the
//! pilots, tacking, sensor arbitration and servo logic port in phase 1.
#![no_std]

pub mod heading;

use rustpilot_hal::{ControlCommand, Mode, MotorCommand, PilotState};
use rustpilot_types::{Degrees, Instant};

pub use heading::{HeadingError, clamp_heading_command};

/// Everything the loop reads in one step.
#[derive(Debug, Clone, Default)]
pub struct Inputs<'a> {
    /// Low-passed compass heading from the IMU fusion, degrees.
    pub compass_heading: Option<Degrees>,
    /// Commands that arrived since the last step, already checked against
    /// their link's control level.
    pub commands: &'a [ControlCommand],
}

/// Everything the loop writes in one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outputs {
    /// What to send the motor controller.
    pub motor: MotorCommand,
    /// State for the control links to publish.
    pub state: PilotState,
}

/// The autopilot.
#[derive(Debug, Clone)]
pub struct Autopilot {
    state: PilotState,
    error: HeadingError,
    last_step: Option<Instant>,
}

impl Default for Autopilot {
    fn default() -> Self {
        Self::new()
    }
}

impl Autopilot {
    /// A disengaged autopilot in compass mode, pypilot's start-up state.
    pub fn new() -> Self {
        Self {
            state: PilotState {
                mode: Some(Mode::Compass),
                ..PilotState::default()
            },
            error: HeadingError::default(),
            last_step: None,
        }
    }

    /// The current state.
    pub fn state(&self) -> &PilotState {
        &self.state
    }

    /// Run one loop iteration at time `now`.
    pub fn step(&mut self, now: Instant, inputs: &Inputs<'_>) -> Outputs {
        for cmd in inputs.commands {
            self.apply(cmd);
        }

        let mode = self.state.mode.unwrap_or(Mode::Compass);
        if let Some(heading) = inputs.compass_heading {
            self.state.compass_heading = heading;
            if mode == Mode::Compass {
                self.state.heading = heading;
            }
        }
        let dt = self
            .last_step
            .map(|t| (now - t).as_secs_f32())
            .unwrap_or(0.0);
        self.last_step = Some(now);
        self.state.heading_error =
            self.error
                .update(self.state.heading, self.state.heading_command, mode, dt);

        let motor = if self.state.enabled {
            // Phase 1 replaces this with the selected pilot's output.
            MotorCommand::Speed(0.0)
        } else {
            MotorCommand::Disengage
        };
        Outputs {
            motor,
            state: self.state,
        }
    }

    fn apply(&mut self, cmd: &ControlCommand) {
        let mode = self.state.mode.unwrap_or(Mode::Compass);
        match *cmd {
            ControlCommand::Standby => self.state.enabled = false,
            ControlCommand::Engage(m) => {
                if let Some(m) = m {
                    self.state.mode = Some(m);
                }
                if !self.state.enabled {
                    // pypilot engages on the current heading.
                    self.state.heading_command =
                        clamp_heading_command(self.state.heading, self.state.mode.unwrap_or(mode));
                }
                self.state.enabled = true;
            }
            ControlCommand::SetMode(m) => self.state.mode = Some(m),
            ControlCommand::SetHeading(h) => {
                self.state.heading_command = clamp_heading_command(h, mode);
            }
            ControlCommand::AdjustHeading(d) => {
                let h = self.state.heading_command + d;
                self.state.heading_command = clamp_heading_command(h, mode);
            }
            // Tacking, manual steering and value writes arrive in phase 1.
            ControlCommand::Tack(_)
            | ControlCommand::Jog { .. }
            | ControlCommand::RudderAngle(_)
            | ControlCommand::SetValue { .. } => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engage_holds_current_heading() {
        let mut ap = Autopilot::new();
        let t = Instant::from_micros(0);
        let out = ap.step(
            t,
            &Inputs {
                compass_heading: Some(123.4),
                commands: &[],
            },
        );
        assert_eq!(out.motor, MotorCommand::Disengage);

        let cmds = [ControlCommand::Engage(None)];
        let out = ap.step(
            t,
            &Inputs {
                compass_heading: Some(123.4),
                commands: &cmds,
            },
        );
        assert!(out.state.enabled);
        assert_eq!(out.state.heading_command, 123.4);
        assert_eq!(out.motor, MotorCommand::Speed(0.0));

        let cmds = [ControlCommand::AdjustHeading(10.0)];
        let out = ap.step(
            t,
            &Inputs {
                compass_heading: Some(123.4),
                commands: &cmds,
            },
        );
        assert!((out.state.heading_error - -10.0).abs() < 1e-3);

        let cmds = [ControlCommand::Standby];
        let out = ap.step(
            t,
            &Inputs {
                compass_heading: Some(123.4),
                commands: &cmds,
            },
        );
        assert_eq!(out.motor, MotorCommand::Disengage);
    }
}
