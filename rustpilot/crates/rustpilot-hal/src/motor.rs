//! Connector family 4: motor controllers (the pypilot Arduino controller
//! over UART, a direct PWM H-bridge, later N2K or vendor drives).

use rustpilot_types::MotorTelemetry;

use crate::ConnError;

/// A command to the motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorCommand {
    /// Drive the motor with a signed raw command, −1..1 after the servo's
    /// calibration, positive to port as in pypilot. 0 stops with the clutch
    /// still engaged.
    Speed(f32),
    /// Release the clutch and stop driving.
    Disengage,
    /// Clear latched faults.
    Reset,
}

/// Settings sent to the controller with every command, so it can enforce
/// limits on its own if the loop stalls. These mirror the Arduino
/// controller's parameter set (`ArduinoServo::params`); units follow
/// pypilot's `servo.*` and `rudder.*` values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorLimits {
    /// Current limit before calibration is applied, amps.
    pub raw_max_current: f32,
    /// Raw rudder reading at the starboard end of the range, −0.5..0.5.
    pub rudder_min: f32,
    /// Raw rudder reading at the port end of the range, −0.5..0.5.
    pub rudder_max: f32,
    /// Amps.
    pub max_current: f32,
    /// °C.
    pub max_controller_temp: f32,
    /// °C.
    pub max_motor_temp: f32,
    /// Rudder range either side of centre, degrees.
    pub rudder_range: f32,
    /// Rudder calibration offset.
    pub rudder_offset: f32,
    /// Rudder calibration scale.
    pub rudder_scale: f32,
    /// Rudder calibration nonlinearity.
    pub rudder_nonlinearity: f32,
    /// Fastest slew, percent per second.
    pub max_slew_speed: f32,
    /// Slowest slew, percent per second.
    pub max_slew_slow: f32,
    /// Current sensor calibration factor.
    pub current_factor: f32,
    /// Current sensor calibration offset.
    pub current_offset: f32,
    /// Voltage sensor calibration factor.
    pub voltage_factor: f32,
    /// Voltage sensor calibration offset.
    pub voltage_offset: f32,
    /// Minimum motor speed, percent.
    pub min_speed: f32,
    /// Maximum motor speed, percent.
    pub max_speed: f32,
    /// Command gain; negative reverses the motor.
    pub gain: f32,
    /// Clutch PWM duty, percent.
    pub clutch_pwm: f32,
    /// Brake when stopped.
    pub brake: bool,
}

/// A motor controller.
pub trait MotorController {
    /// Send a command. Called every loop iteration; the controller times
    /// out and stops if commands stop arriving.
    async fn command(&mut self, cmd: MotorCommand) -> Result<(), ConnError>;

    /// Wait for the next telemetry update.
    async fn telemetry(&mut self) -> Result<MotorTelemetry, ConnError>;

    /// Push limits to the controller.
    async fn configure(&mut self, limits: &MotorLimits) -> Result<(), ConnError>;
}
