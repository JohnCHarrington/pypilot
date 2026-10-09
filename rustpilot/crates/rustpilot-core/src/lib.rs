//! RustPilot's autopilot core.
//!
//! A pure state machine: platforms read their connectors, pass the latest
//! readings in [`Inputs`], call [`Autopilot::step`] once per IMU sample, and
//! write the returned [`Outputs`] to the motor controller and control links.
//! Nothing here performs I/O or reads a clock, so the same code runs on
//! Linux, ESP32, in the simulator and under log replay.
//!
//! The modules port pypilot's `autopilot.py`, `servo.py`, `sensors.py`,
//! `rudder.py`, the basic pilot and tacking. `tests/golden.rs` replays
//! scenarios recorded from pypilot's own loop and checks the outputs match.
#![no_std]

pub mod autopilot;
pub mod heading;
pub mod pilots;
pub mod rudder;
pub mod sensors;
pub mod servo;
pub mod tacking;
pub mod values;

pub use autopilot::{Autopilot, ImuData, Inputs, MotorInputs, Outputs};
pub use heading::{HeadingError, clamp_heading_command};
