//! Plain data types and maths shared by every RustPilot crate.
//!
//! Everything here is `no_std`, allocation-free and uses `f32`, because the
//! ESP32-S3 has a single-precision FPU and software `f64` would cost the
//! control loop its budget. pypilot's Python uses doubles; golden tests
//! compare within tolerances rather than bit for bit.
#![no_std]

pub mod angle;
pub mod quaternion;
pub mod sensors;
pub mod time;
pub mod vector;

pub use angle::{Degrees, resolv};
pub use quaternion::Quaternion;
pub use sensors::*;
pub use time::{Duration, Instant};
pub use vector::Vec3;
