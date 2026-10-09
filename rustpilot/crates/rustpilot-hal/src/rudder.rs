//! Connector family 5: rudder angle sensors.
//!
//! The core applies pypilot's offset, scale and nonlinearity calibration
//! (`rudder.py`), so sensors report raw values. A motor controller adapter
//! that reads the rudder itself implements this trait too. Rudder angles
//! that arrive already calibrated (NMEA RSA, N2K 127245) come in as
//! [`crate::NavMessage::Rudder`] instead.

use crate::ConnError;

/// A raw rudder sensor.
pub trait RudderSensor {
    /// Wait for the next raw reading, 0..1 of the sensor's range. `None`
    /// means the sensor reports no rudder (unplugged).
    async fn read(&mut self) -> Result<Option<f32>, ConnError>;
}
