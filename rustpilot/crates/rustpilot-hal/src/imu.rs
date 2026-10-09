//! Connector family 3: inertial measurement units.

use rustpilot_types::{ImuSample, Instant};

use crate::ConnError;

/// One IMU sample with its time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    /// When the sample was taken.
    pub time: Instant,
    /// The sample.
    pub sample: ImuSample,
}

/// An IMU: a raw 9-axis chip, a self-fusing chip, or a degraded heading
/// source.
pub trait ImuSource {
    /// Wait for the next sample.
    async fn read(&mut self) -> Result<ImuReading, ConnError>;

    /// The configured sample rate, which sets the control loop rate
    /// (pypilot: 10 or 20 Hz).
    fn rate_hz(&self) -> u16;
}
