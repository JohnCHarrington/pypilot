//! The connector traits: everything RustPilot's core needs from the outside
//! world, as `no_std` async traits that each platform implements (Tokio on
//! Linux, Embassy on ESP32, plain structs in the simulator).
//!
//! The core never names a transport. One physical link (an N2K bus, a TCP
//! socket) can implement several of these traits, split into halves at
//! startup so the halves share one driver.
//!
//! Futures returned by these traits are not required to be `Send`: Embassy
//! runs single-threaded, and the Linux binary drives connectors on a local
//! task set. That is why `async_fn_in_trait` is allowed here.
#![no_std]
#![allow(async_fn_in_trait)]

pub mod can;
pub mod control;
pub mod imu;
pub mod motor;
pub mod nav;
pub mod rudder;

pub use can::{CanBus, CanFrame};
pub use control::{
    ControlCommand, ControlLevel, ControlLink, ControlRequest, Mode, PilotState, TackDirection,
};
pub use imu::{ImuReading, ImuSource};
pub use motor::{MotorCommand, MotorController, MotorLimits};
pub use nav::{NavMessage, NavOutput, NavReading, NavSink, NavSource};
pub use rudder::RudderSensor;
pub use rustpilot_types as types;

/// Byte-stream traits for UART and TCP links (NMEA 0183, the Arduino motor
/// controller, pypilot's JSON protocol).
pub use embedded_io_async as io;

use rustpilot_types::Instant;

/// Why a connector call failed. Deliberately coarse: the core only needs to
/// know whether to retry, fall back to another source, or raise a fault.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnError {
    /// The link is down or not yet open; the caller may retry later.
    Disconnected,
    /// No data within the connector's own timeout.
    Timeout,
    /// Data arrived but could not be parsed.
    Malformed,
    /// The peer rejected or does not support the request.
    Unsupported,
    /// The link's queue is full; the message was dropped.
    Overflow,
}

/// A monotonic clock. Platforms read it and pass `now` into the core, which
/// never reads time itself.
pub trait Clock {
    /// The current time.
    fn now(&self) -> Instant;
}

/// A short identifier for the device behind a reading (`/dev/ttyUSB0`,
/// `N2K35`, `signalk`), used like pypilot's `<sensor>.device` value to tell
/// two devices at the same priority apart.
pub type DeviceId = heapless::String<24>;
