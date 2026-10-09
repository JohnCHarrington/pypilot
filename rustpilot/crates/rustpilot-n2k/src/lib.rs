//! NMEA 2000 for RustPilot, built on the [`canboat`] crate (the CANboat PGN
//! database, decode and encode).
//!
//! canboat needs `std`, so this crate does too: it serves the Linux and Pi
//! builds, and the ESP32 build under esp-idf's std runtime. See
//! `docs/phase0-findings.md` for what that decided.
//!
//! canboat decodes and encodes PGN 126208 group functions, resolving each
//! parameter against the target PGN's field layout, but its node layer only
//! answers discovery (address claim, product info, PGN lists, heartbeat). So
//! dispatching 126208 commands and ISO requests to the autopilot is done
//! here, in [`group_function`].

pub mod group_function;

pub use canboat;

use canboat::{Database, Units};

/// The PGN database RustPilot uses. Metric units give degrees, knots and
/// °C, matching pypilot's values.
pub fn database() -> &'static Database {
    Database::embedded(Units::Metric)
}

/// PGN 126208, NMEA Request/Command/Acknowledge group function.
pub const PGN_GROUP_FUNCTION: u32 = 126208;
/// PGN 127237, Heading/Track Control.
pub const PGN_HEADING_TRACK_CONTROL: u32 = 127237;
/// PGN 126998, Configuration Information (carries `PP:` text commands).
pub const PGN_CONFIGURATION_INFORMATION: u32 = 126998;
