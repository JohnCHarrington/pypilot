//! Sensor readings as plain data, in pypilot's units: degrees, knots,
//! amps, volts, degrees Celsius.

use crate::angle::Degrees;
use crate::quaternion::Quaternion;
use crate::vector::Vec3;

/// Where a reading came from. [`SensorSource::priority`] follows pypilot's
/// `source_priority` (`sensors.py`): a lower value wins when two sources
/// report the same sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorSource {
    /// gpsd daemon (priority 1).
    Gpsd,
    /// The motor controller's own rudder feedback (priority 1).
    Servo,
    /// A serial NMEA 0183 port (priority 2).
    Serial,
    /// NMEA 0183 over TCP (priority 3).
    Tcp,
    /// SignalK (priority 4).
    Signalk,
    /// Synthesised from water speed and wind (priority 5).
    WaterWind,
    /// Synthesised from GPS and wind (priority 6).
    GpsWind,
    /// No source (priority 7).
    None,
    /// NMEA 2000 (priority 1, as `can` on pypilot's n2k branch).
    Can,
}

impl SensorSource {
    /// pypilot's numeric priority; lower is preferred.
    pub const fn priority(self) -> u8 {
        match self {
            SensorSource::Gpsd | SensorSource::Servo | SensorSource::Can => 1,
            SensorSource::Serial => 2,
            SensorSource::Tcp => 3,
            SensorSource::Signalk => 4,
            SensorSource::WaterWind => 5,
            SensorSource::GpsWind => 6,
            SensorSource::None => 7,
        }
    }

    /// The name pypilot uses in `<sensor>.source` values.
    pub const fn name(self) -> &'static str {
        match self {
            SensorSource::Gpsd => "gpsd",
            SensorSource::Servo => "servo",
            SensorSource::Serial => "serial",
            SensorSource::Tcp => "tcp",
            SensorSource::Signalk => "signalk",
            SensorSource::WaterWind => "water+wind",
            SensorSource::GpsWind => "gps+wind",
            SensorSource::None => "none",
            SensorSource::Can => "can",
        }
    }
}

/// A GPS fix (`gps.*` values).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GpsFix {
    /// Course over ground, degrees true. `None` when stationary.
    pub track: Option<Degrees>,
    /// Speed over ground, knots.
    pub speed: f32,
    /// Latitude, degrees.
    pub lat: Option<f64>,
    /// Longitude, degrees.
    pub lon: Option<f64>,
    /// Magnetic declination, degrees, when the source reports it.
    pub declination: Option<Degrees>,
}

/// Wind relative to the boat (`wind.*`, `truewind.*`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Wind {
    /// Angle off the bow, degrees, positive to starboard.
    pub direction: Degrees,
    /// Speed, knots.
    pub speed: f32,
}

/// Speed through water (`water.*`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WaterSpeed {
    /// Knots.
    pub speed: f32,
    /// Leeway, degrees, when measured.
    pub leeway: Option<Degrees>,
}

/// Route steering from a plotter (NMEA APB or N2K 129284; `apb.*`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RouteSteer {
    /// Bearing to steer, degrees.
    pub track: Degrees,
    /// Cross-track error, nautical miles, positive to the right of track.
    pub xte: f32,
    /// True when `track` is magnetic; pypilot then steers in compass mode.
    pub magnetic: bool,
}

/// One IMU reading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImuSample {
    /// Raw 9-axis sample for RustPilot's own fusion.
    Raw {
        /// Acceleration, g.
        accel: Vec3,
        /// Angular rate, degrees per second.
        gyro: Vec3,
        /// Magnetic field, sensor units (calibrated later).
        compass: Vec3,
    },
    /// Attitude from a chip that fuses on board (BNO08x and similar).
    Fused {
        /// Orientation, sensor frame to earth.
        pose: Quaternion,
        /// Angular rate, degrees per second.
        gyro: Vec3,
        /// Acceleration, g.
        accel: Vec3,
    },
}

/// Motor controller telemetry, mirroring the Arduino controller's telemetry
/// flags (`arduino_servo.h`). Fields the controller did not send this cycle
/// are `None`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MotorTelemetry {
    /// Fault and state flags, the controller's raw bitfield.
    pub flags: Option<u16>,
    /// Motor current, amps.
    pub current: Option<f32>,
    /// Supply voltage, volts.
    pub voltage: Option<f32>,
    /// Controller temperature, °C.
    pub controller_temp: Option<f32>,
    /// Motor temperature, °C.
    pub motor_temp: Option<f32>,
    /// Raw rudder sensor reading, 0..1 of the ADC range.
    pub rudder_raw: Option<f32>,
}
