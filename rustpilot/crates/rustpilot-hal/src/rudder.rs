//! Connector family 5: rudder angle sensors.
//!
//! The core applies pypilot's offset, scale and nonlinearity calibration
//! (`rudder.py`), so sensors report raw values. A motor controller adapter
//! that reads the rudder itself implements this trait too. Rudder readings
//! from instruments (NMEA RSA, N2K 127245) come in as
//! [`crate::NavMessage::Rudder`] and are calibrated the same way.

use crate::ConnError;

/// A raw rudder sensor.
pub trait RudderSensor {
    /// Wait for the next raw reading, nominally −0.5..0.5. `None`
    /// means the sensor reports no rudder (unplugged).
    async fn read(&mut self) -> Result<Option<f32>, ConnError>;
}
