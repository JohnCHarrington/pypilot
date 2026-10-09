//! Connector family 1: navigation data in and out (NMEA 0183, NMEA 2000,
//! SignalK, gpsd).

use rustpilot_types::{Degrees, GpsFix, Instant, RouteSteer, SensorSource, WaterSpeed, Wind};

use crate::{ConnError, DeviceId};

/// One piece of navigation data, already in pypilot's units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavMessage {
    /// GPS position, course and speed.
    Gps(GpsFix),
    /// Apparent wind.
    Wind(Wind),
    /// True wind, when the source computes it.
    TrueWind(Wind),
    /// Speed through water.
    Water(WaterSpeed),
    /// Route steering from a plotter.
    Route(RouteSteer),
    /// Rudder reading from an instrument (NMEA RSA, N2K 127245). The core
    /// applies the same rudder calibration to it as to the motor
    /// controller's raw reading, as pypilot does.
    Rudder(f32),
}

/// A [`NavMessage`] with where and when it came from.
#[derive(Debug, Clone, PartialEq)]
pub struct NavReading {
    /// Arbitration class for pypilot's source priority.
    pub source: SensorSource,
    /// The device within that class.
    pub device: DeviceId,
    /// When the connector received it.
    pub time: Instant,
    /// The data.
    pub message: NavMessage,
}

/// Data the autopilot publishes for other instruments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavOutput {
    /// Vessel heading (N2K 127250, NMEA HDM/HDT).
    Heading {
        /// Degrees.
        heading: Degrees,
        /// Magnetic when true, else true heading.
        magnetic: bool,
    },
    /// Rate of turn, degrees per second (127251, ROT).
    RateOfTurn(f32),
    /// Pitch and roll, degrees (127257, XDR).
    Attitude {
        /// Degrees, bow up positive.
        pitch: Degrees,
        /// Degrees, starboard down positive.
        roll: Degrees,
    },
    /// Rudder angle, degrees (127245, RSA).
    Rudder(Degrees),
}

/// A source of navigation data.
pub trait NavSource {
    /// Wait for the next reading.
    async fn recv(&mut self) -> Result<NavReading, ConnError>;
}

/// A sink the autopilot writes its own measurements to.
pub trait NavSink {
    /// Send one output. Connectors may rate-limit or drop as their wire
    /// format requires.
    async fn send(&mut self, out: &NavOutput) -> Result<(), ConnError>;
}
