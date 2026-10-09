//! Connector family 4: motor controllers (the pypilot Arduino controller
//! over UART, a direct PWM H-bridge, later N2K or vendor drives).

use rustpilot_types::MotorTelemetry;

use crate::ConnError;

/// A command to the motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorCommand {
    /// Run at a signed speed, −1 (full port) to 1 (full starboard). 0 stops
    /// with the motor still engaged.
    Speed(f32),
    /// Release the clutch and stop driving.
    Disengage,
    /// Clear latched faults.
    Reset,
}

/// Limits the controller enforces on its own, so a stalled loop can't
/// drive past them. Units follow pypilot's `servo.*` settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorLimits {
    /// Amps.
    pub max_current: f32,
    /// °C.
    pub max_controller_temp: f32,
    /// °C.
    pub max_motor_temp: f32,
    /// Raw rudder range the controller stops at, 0..1, when it reads the
    /// rudder itself.
    pub rudder_min: f32,
    /// See `rudder_min`.
    pub rudder_max: f32,
    /// Fastest slew, percent per second.
    pub max_slew_speed: f32,
    /// Slowest slew, percent per second.
    pub max_slew_slow: f32,
    /// Clutch PWM duty, percent.
    pub clutch_pwm: f32,
    /// Brake when stopped.
    pub use_brake: bool,
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
