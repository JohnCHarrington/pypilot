//! Servo logic (`servo.py`): turns pilot and manual commands into motor
//! commands, with pypilot's period-based windup, minimum speed, direction
//! lockouts after overcurrent or rudder-limit faults, and telemetry
//! corrections.
//!
//! The motor controller itself sits behind
//! [`rustpilot_hal::MotorController`]; this module only decides what to send
//! it. Every driver call pypilot makes in one poll becomes a field of
//! [`ServoOutput`].

use rustpilot_hal::{MotorCommand, MotorLimits};
use rustpilot_types::{Instant, MotorTelemetry};

/// `servo.flags` bits (`ServoFlags`). The low 16 bits come from the
/// controller; the high bits are pypilot's own.
pub mod flags {
    #![allow(missing_docs)]
    pub const SYNC: u32 = 1;
    pub const OVERTEMP_FAULT: u32 = 2;
    pub const OVERCURRENT_FAULT: u32 = 4;
    pub const ENGAGED: u32 = 8;
    pub const INVALID: u32 = 16;
    pub const PORT_PIN_FAULT: u32 = 32;
    pub const STARBOARD_PIN_FAULT: u32 = 64;
    pub const BADVOLTAGE_FAULT: u32 = 128;
    pub const MIN_RUDDER_FAULT: u32 = 256;
    pub const MAX_RUDDER_FAULT: u32 = 512;
    pub const CURRENT_RANGE: u32 = 1024;
    pub const BAD_FUSES: u32 = 2048;
    pub const REBOOTED: u32 = 32768;
    pub const DRIVER_MASK: u32 = 0xffff;
    pub const PORT_OVERCURRENT_FAULT: u32 = 0x1_0000;
    pub const STARBOARD_OVERCURRENT_FAULT: u32 = 0x2_0000;
    pub const DRIVER_TIMEOUT: u32 = 0x4_0000;
    pub const SATURATED: u32 = 0x8_0000;

    /// Space-separated flag names, as pypilot reports `servo.flags`.
    pub fn names(value: u32) -> impl Iterator<Item = &'static str> {
        const ALL: [(u32, &str); 16] = [
            (SYNC, "SYNC"),
            (OVERTEMP_FAULT, "OVERTEMP_FAULT"),
            (OVERCURRENT_FAULT, "OVERCURRENT_FAULT"),
            (ENGAGED, "ENGAGED"),
            (INVALID, "INVALID"),
            (PORT_PIN_FAULT, "PORT_PIN_FAULT"),
            (STARBOARD_PIN_FAULT, "STARBOARD_PIN_FAULT"),
            (BADVOLTAGE_FAULT, "BADVOLTAGE_FAULT"),
            (MIN_RUDDER_FAULT, "MIN_RUDDER_FAULT"),
            (MAX_RUDDER_FAULT, "MAX_RUDDER_FAULT"),
            (BAD_FUSES, "BAD_FUSES"),
            (PORT_OVERCURRENT_FAULT, "PORT_OVERCURRENT_FAULT"),
            (STARBOARD_OVERCURRENT_FAULT, "STARBOARD_OVERCURRENT_FAULT"),
            (DRIVER_TIMEOUT, "DRIVER_TIMEOUT"),
            (SATURATED, "SATURATED"),
            (REBOOTED, "REBOOTED"),
        ];
        ALL.into_iter()
            .filter(move |(b, _)| value & b != 0)
            .map(|(_, n)| n)
    }
}

use flags::*;

/// `servo.state`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoState {
    /// Nothing sent yet.
    None,
    /// Stopped after a fault.
    Stop,
    /// Driving to port.
    Port,
    /// Driving to starboard.
    Starboard,
    /// Stopped, brake on.
    Brake,
    /// Stopped, brake off.
    Idle,
}

impl ServoState {
    /// pypilot's name for it.
    pub const fn name(self) -> &'static str {
        match self {
            ServoState::None => "none",
            ServoState::Stop => "stop",
            ServoState::Port => "port",
            ServoState::Starboard => "starboard",
            ServoState::Brake => "brake",
            ServoState::Idle => "idle",
        }
    }
}

/// A command value with the time it was last written, by a person
/// (`set`) or by the pilot (`command`) (`TimedProperty`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimedCommand {
    /// The value.
    pub value: f32,
    /// Last write of either kind.
    pub time: Instant,
    /// Last manual write.
    pub set_time: Instant,
    /// False after a manual write: manual control skips the period logic.
    pub use_period: bool,
}

impl TimedCommand {
    fn new(now: Instant) -> Self {
        // pypilot's constructor goes through set(), so it starts out as a
        // fresh manual write.
        Self {
            value: 0.0,
            time: now,
            set_time: now,
            use_period: false,
        }
    }

    /// A manual write (`TimedProperty.set`).
    pub fn set(&mut self, value: f32, now: Instant) {
        self.time = now;
        self.set_time = now;
        self.use_period = false;
        self.value = value;
    }

    /// A pilot write (`TimedProperty.command`), ignored for 0.8 s after a
    /// manual one.
    pub fn command(&mut self, value: f32, now: Instant) {
        if secs(now, self.set_time) < 0.8 {
            return;
        }
        self.time = now;
        self.use_period = true;
        self.value = value;
    }
}

fn secs(now: Instant, then: Instant) -> f32 {
    // signed, as pypilot subtracts monotonic times
    (now.0 as f64 - then.0 as f64) as f32 * 1.0e-6
}

fn sign(x: f32) -> f32 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Servo calibration: raw command = `offset + |speed| * slope` per
/// direction (`servo.calibration`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoCalibration {
    /// `[offset, slope]` when driving to port.
    pub port: [f32; 2],
    /// `[offset, slope]` when driving to starboard.
    pub starboard: [f32; 2],
}

impl Default for ServoCalibration {
    /// pypilot's fallback when no calibration file exists.
    fn default() -> Self {
        Self {
            port: [0.2, 0.8],
            starboard: [0.2, 0.8],
        }
    }
}

/// What the servo reads each poll.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ServoInputs {
    /// Telemetry that arrived since the last poll, if any.
    pub telemetry: Option<MotorTelemetry>,
    /// Calibrated rudder angle, `None` if invalid.
    pub rudder_angle: Option<f32>,
    /// `rudder.range`, degrees.
    pub rudder_range: f32,
    /// `rudder.minmax`.
    pub rudder_minmax: (f32, f32),
    /// `rudder.offset`, `rudder.scale`, `rudder.nonlinearity`.
    pub rudder_calibration: (f32, f32, f32),
}

/// What the servo asks of the motor controller after one poll.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoOutput {
    /// Clear the controller's latched fault first.
    pub reset: bool,
    /// The command to send, if one is due this poll. pypilot throttles
    /// repeated zero commands, so `None` means "send nothing".
    pub command: Option<MotorCommand>,
    /// Parameters to send with the command.
    pub limits: MotorLimits,
}

/// The servo (`servo.*` values).
#[derive(Debug, Clone, PartialEq)]
pub struct Servo {
    /// Pilot or manual speed command, −1..1 (`servo.command`).
    pub command: TimedCommand,
    /// Manual rudder angle command (`servo.position_command`).
    pub position_command: TimedCommand,
    /// Raises minimum speed with duty cycle (`servo.speed_gain`, 0..1).
    pub speed_gain: f32,
    /// Low-passed fraction of time the motor is driven (`servo.duty`).
    pub duty: f32,
    /// Fault count (`servo.faults`).
    pub faults: u32,
    /// Corrected volts (`servo.voltage`).
    pub voltage: Option<f32>,
    /// Corrected amps (`servo.current`).
    pub current: f32,
    /// Current noise floor (`servo.current.noise`).
    pub current_noise: f32,
    /// °C, `None` after 8 s without a reading.
    pub controller_temp: Option<f32>,
    /// °C, `None` after 8 s without a reading.
    pub motor_temp: Option<f32>,
    /// Clutch engaged, per the controller (`servo.engaged`).
    pub engaged: bool,
    /// Amps (`servo.max_current`, 0..20 or 0..50).
    pub max_current: f32,
    max_current_limit: f32,
    /// `servo.current.factor`.
    pub current_factor: f32,
    /// `servo.current.offset`.
    pub current_offset: f32,
    /// `servo.voltage.factor`.
    pub voltage_factor: f32,
    /// `servo.voltage.offset`.
    pub voltage_offset: f32,
    /// °C (`servo.max_controller_temp`).
    pub max_controller_temp: f32,
    /// °C (`servo.max_motor_temp`).
    pub max_motor_temp: f32,
    /// `servo.max_slew_speed`.
    pub max_slew_speed: f32,
    /// `servo.max_slew_slow`.
    pub max_slew_slow: f32,
    /// `servo.gain`; negative reverses the motor.
    pub gain: f32,
    /// `servo.clutch_pwm`, percent.
    pub clutch_pwm: f32,
    /// `servo.use_brake`.
    pub use_brake: bool,
    brake_on: bool,
    /// Command period, seconds (`servo.period`).
    pub period: f32,
    /// `servo.compensate_voltage`.
    pub compensate_voltage: bool,
    /// `servo.amp_hours`.
    pub amp_hours: f32,
    /// `servo.watts`.
    pub watts: f32,
    /// Last applied speed, −1..1 (`servo.speed`).
    pub speed: f32,
    /// Minimum speed, percent (`servo.speed.min`).
    pub speed_min: f32,
    /// Maximum speed, percent (`servo.speed.max`).
    pub speed_max: f32,
    /// Rudder position, measured or estimated, degrees (`servo.position`).
    pub position: f32,
    position_elp: f32,
    /// Position loop gains (`servo.position.p/i/d`).
    pub position_pid: (f32, f32, f32),
    /// Last raw command (`servo.raw_command`).
    pub raw_command: f32,
    /// Calibration.
    pub calibration: ServoCalibration,
    /// `servo.state`.
    pub state: ServoState,
    /// `servo.flags`.
    pub flags: u32,
    /// Whether a controller is connected (`servo.controller` != none).
    pub connected: bool,
    /// Set by the autopilot each step.
    pub ap_enabled: bool,
    last_ap_enabled: bool,
    inttime: Instant,
    windup: f32,
    windup_change: Instant,
    disengaged: bool,
    last_zero_command_time: Instant,
    command_timeout: Instant,
    driver_timeout_start: Option<Instant>,
    lastdir: i8,
    current_lasttime: Instant,
    temp_times: (Instant, Instant),
    out: ServoOutput,
}

impl Servo {
    /// A servo with pypilot's defaults, created at `now`.
    pub fn new(now: Instant) -> Self {
        let zero = Instant::default();
        let mut s = Self {
            command: TimedCommand::new(now),
            position_command: TimedCommand::new(now),
            speed_gain: 0.0,
            duty: 0.0,
            faults: 0,
            voltage: None,
            current: 0.0,
            current_noise: 0.0,
            controller_temp: None,
            motor_temp: None,
            engaged: false,
            max_current: 4.5,
            max_current_limit: 50.0,
            current_factor: 1.0,
            current_offset: 0.0,
            voltage_factor: 1.0,
            voltage_offset: 0.0,
            max_controller_temp: 60.0,
            max_motor_temp: 60.0,
            max_slew_speed: 28.0,
            max_slew_slow: 34.0,
            gain: 1.0,
            clutch_pwm: 100.0,
            use_brake: true,
            brake_on: false,
            period: 0.4,
            compensate_voltage: false,
            amp_hours: 0.0,
            watts: 0.0,
            speed: 0.0,
            speed_min: 100.0,
            speed_max: 100.0,
            position: 0.0,
            position_elp: 0.0,
            position_pid: (0.1, 0.0, 0.01),
            raw_command: 0.0,
            calibration: ServoCalibration::default(),
            state: ServoState::None,
            flags: 0,
            connected: false,
            ap_enabled: false,
            last_ap_enabled: false,
            inttime: zero,
            windup: 0.0,
            windup_change: zero,
            disengaged: true,
            last_zero_command_time: now,
            command_timeout: now,
            driver_timeout_start: None,
            lastdir: 0,
            current_lasttime: now,
            temp_times: (now, now),
            out: ServoOutput {
                reset: false,
                command: None,
                limits: placeholder_limits(),
            },
        };
        s.raw(0.0, now, &ServoInputs::default());
        s
    }

    /// Whether the controller reports an overcurrent fault (`Servo.fault`).
    pub fn fault(&self) -> bool {
        self.connected && self.flags & OVERCURRENT_FAULT != 0
    }

    /// The controller connected (`servo.controller = arduino`). pypilot
    /// sends a disengage straight away.
    pub fn connect(&mut self) {
        self.connected = true;
    }

    /// The controller went away (`close_driver`).
    pub fn disconnect(&mut self) {
        self.connected = false;
        self.flags = 0;
    }

    /// One poll (`Servo.poll` after the driver read). Returns what to send.
    pub fn poll(&mut self, i: &ServoInputs, now: Instant) -> ServoOutput {
        self.out = ServoOutput {
            reset: false,
            command: None,
            limits: self.limits(1.0, i),
        };

        if let Some(tel) = i.telemetry {
            self.apply_telemetry(&tel, i, now);
        }

        if self.fault() {
            if self.flags & (PORT_OVERCURRENT_FAULT | STARBOARD_OVERCURRENT_FAULT) == 0 {
                self.faults += 1;
            }
            // fault in the direction travelled, so it can't drive further
            if self.flags & OVERCURRENT_FAULT != 0 {
                if self.lastdir > 0 {
                    self.flags =
                        (self.flags | PORT_OVERCURRENT_FAULT) & !STARBOARD_OVERCURRENT_FAULT;
                } else if self.lastdir < 0 {
                    self.flags =
                        (self.flags | STARBOARD_OVERCURRENT_FAULT) & !PORT_OVERCURRENT_FAULT;
                }
                if i.rudder_angle.is_none() && self.lastdir != 0 {
                    self.position = f32::from(self.lastdir) * i.rudder_range;
                }
            }
            self.out.reset = self.connected;
        }

        if let Some(angle) = i.rudder_angle {
            self.position = angle;
        }

        self.send_command(i, now);

        if self.controller_temp.is_some() && secs(now, self.temp_times.0) > 8.0 {
            self.controller_temp = None;
        }
        if self.motor_temp.is_some() && secs(now, self.temp_times.1) > 8.0 {
            self.motor_temp = None;
        }

        if self.ap_enabled != self.last_ap_enabled {
            self.last_ap_enabled = self.ap_enabled;
            self.flags &= !(PORT_OVERCURRENT_FAULT | STARBOARD_OVERCURRENT_FAULT);
        }
        self.out
    }

    fn apply_telemetry(&mut self, tel: &MotorTelemetry, i: &ServoInputs, now: Instant) {
        if let Some(v) = tel.voltage {
            let v = self.voltage_factor * v + self.voltage_offset;
            self.voltage = Some(libm::roundf(v * 1000.0) / 1000.0);
        }
        if let Some(t) = tel.controller_temp {
            self.controller_temp = Some(t);
            self.temp_times.0 = now;
        }
        if let Some(t) = tel.motor_temp {
            self.motor_temp = Some(t);
            self.temp_times.1 = now;
        }
        if let Some(mut current) = tel.current {
            if current < self.current_noise * 1.2 {
                current = 0.0;
            } else if current != 0.0 && secs(now, self.command_timeout) > 3.0 {
                self.current_noise = self.current_noise.max(current).min(1.0);
            }
            let mut corrected = self.current_factor * current;
            if current != 0.0 {
                corrected = (corrected + self.current_offset).max(0.0);
            }
            self.current = libm::roundf(corrected * 1000.0) / 1000.0;

            let dt = secs(now, self.current_lasttime);
            self.current_lasttime = now;
            if dt > 0.01 && dt < 0.5 {
                if self.current != 0.0 {
                    self.amp_hours += self.current * dt / 3600.0;
                }
                let lp = 0.003 * dt; // 5 minute time constant
                self.watts =
                    (1.0 - lp) * self.watts + lp * self.voltage.unwrap_or(0.0) * self.current;
            }
        }
        if let Some(driver_flags) = tel.flags {
            let driver_flags = u32::from(driver_flags);
            self.max_current_limit = if driver_flags & CURRENT_RANGE != 0 {
                50.0
            } else {
                20.0
            };
            self.max_current = self.max_current.min(self.max_current_limit);
            let mut f = (self.flags & !DRIVER_MASK) | driver_flags;
            // a rudder angle from an instrument may also need these
            if let Some(angle) = i.rudder_angle
                && angle != 0.0
                && angle.abs() > i.rudder_range
            {
                f |= if angle > 0.0 {
                    MAX_RUDDER_FAULT
                } else {
                    MIN_RUDDER_FAULT
                };
            }
            self.flags = f;
            self.engaged = driver_flags & ENGAGED != 0;
        }
    }

    /// Set `servo.max_current`, within the controller's current range.
    pub fn set_max_current(&mut self, amps: f32) -> bool {
        if (0.0..=self.max_current_limit).contains(&amps) {
            self.max_current = amps;
            true
        } else {
            false
        }
    }

    fn send_command(&mut self, i: &ServoInputs, now: Instant) {
        let dp = secs(now, self.position_command.time);
        let dc = secs(now, self.command.time);
        if dp < dc && i.rudder_angle.is_some() {
            self.disengaged = false;
            if (self.position - self.position_command.value).abs() < 1.0 {
                self.command.command(0.0, now);
            } else {
                self.do_position_command(self.position_command.value, i, now);
                return;
            }
        } else if self.command.value != 0.0 && !self.fault() {
            if dc > 1.0 {
                self.command.command(0.0, now);
            }
            self.disengaged = false;
        }
        self.do_command(self.command.value, i, now);
    }

    fn do_position_command(&mut self, position: f32, i: &ServoInputs, now: Instant) {
        let e = position - self.position;
        let d = self.speed * i.rudder_range;
        self.position_elp = 0.98 * self.position_elp + 0.02 * e.clamp(-30.0, 30.0);
        let (kp, ki, kd) = self.position_pid;
        let pid = kp * e + ki * self.position_elp + kd * d;
        self.do_command(pid, i, now);
    }

    fn do_command(&mut self, speed: f32, i: &ServoInputs, now: Instant) {
        let t = now;
        let dt = secs(t, self.inttime);
        if self.ap_enabled {
            self.disengaged = false;
        } else {
            self.windup = 0.0;
        }
        self.inttime = t;

        if self.fault() {
            self.stop(now, i);
        }

        if speed == 0.0 {
            if !self.ap_enabled && secs(now, self.command.set_time) > 1.0 {
                self.disengaged = true;
            }
            self.raw(0.0, now, i);
            return;
        }

        let mut speed = speed * self.gain;

        // don't drive further into a fault
        if (self.flags & (PORT_OVERCURRENT_FAULT | MAX_RUDDER_FAULT) != 0 && speed > 0.0)
            || (self.flags & (STARBOARD_OVERCURRENT_FAULT | MIN_RUDDER_FAULT) != 0 && speed < 0.0)
        {
            self.stop(now, i);
            return;
        }

        // clear overcurrent faults once moved well away the other way
        let rudder_range = i.rudder_range;
        if self.position < 0.9 * rudder_range {
            self.flags &= !PORT_OVERCURRENT_FAULT;
        }
        if self.position > -0.9 * rudder_range {
            self.flags &= !STARBOARD_OVERCURRENT_FAULT;
        }

        if self.compensate_voltage
            && let Some(v) = self.voltage.filter(|v| *v != 0.0)
        {
            speed *= 12.0 / v;
        }

        let max_speed = self.speed_max / 100.0;
        let mut min_speed = self.speed_min / 100.0;
        min_speed += (max_speed - min_speed) * self.duty * self.speed_gain;
        let min_speed = min_speed.min(max_speed);

        if self.command.use_period {
            // the pilot is steering: move in bursts of at least min_speed
            let period = self.period.max(2.0 * dt);
            self.windup += (speed - self.speed) * dt;
            if self.windup.abs() > period * min_speed / 1.5 {
                if speed.abs() < min_speed {
                    speed = if self.windup > 0.0 {
                        min_speed
                    } else {
                        -min_speed
                    };
                }
            } else {
                speed = 0.0;
            }

            let max_windup = 1.5 * period;
            if self.windup.abs() > max_windup {
                self.flags |= SATURATED;
                self.windup = max_windup * sign(self.windup);
            } else {
                self.flags &= !SATURATED;
            }

            let last_speed = self.speed;
            if speed != 0.0 || last_speed != 0.0 {
                let m = speed * last_speed;
                if m <= 0.0 {
                    // switched direction, started or stopped
                    if secs(t, self.windup_change) < self.period {
                        // keep the previous direction at minimum speed
                        speed = min_speed * sign(last_speed);
                    } else {
                        self.windup_change = t;
                        if m < 0.0 {
                            speed = 0.0;
                        }
                    }
                }
            }
        }

        let speed = speed.clamp(-max_speed, max_speed);
        self.speed = speed;

        let cal = if speed > 0.0 {
            self.calibration.port
        } else if speed < 0.0 {
            self.calibration.starboard
        } else {
            self.raw(0.0, now, i);
            return;
        };
        let mut command = cal[0] + speed.abs() * cal[1];
        if speed < 0.0 {
            command = -command;
        }

        if i.rudder_angle.is_none() {
            // without rudder feedback, integrate a rough position so the
            // overcurrent lockouts above can still clear
            let position = self.position + command * dt * rudder_range;
            self.position = position.clamp(-rudder_range, rudder_range);
        }
        self.raw(command, now, i);
    }

    fn stop(&mut self, now: Instant, i: &ServoInputs) {
        self.brake_on = false;
        self.do_raw_command(0.0, now, i);
        self.lastdir = 0;
        self.state = ServoState::Stop;
    }

    fn raw(&mut self, command: f32, now: Instant, i: &ServoInputs) {
        self.brake_on = self.use_brake;
        self.do_raw_command(command, now, i);
        if command < 0.0 {
            self.state = ServoState::Starboard;
            self.lastdir = -1;
        } else if command == 0.0 {
            self.speed = 0.0;
            self.state = if self.brake_on {
                ServoState::Brake
            } else {
                ServoState::Idle
            };
        } else {
            self.state = ServoState::Port;
            self.lastdir = 1;
        }
    }

    fn do_raw_command(&mut self, command: f32, now: Instant, i: &ServoInputs) {
        self.raw_command = command;
        let lp = 0.001;
        self.duty = lp * if command != 0.0 { 1.0 } else { 0.0 } + (1.0 - lp) * self.duty;

        let t = now;
        if command == 0.0 {
            // after a second of zeros, only send every 0.2 s
            if secs(t, self.command_timeout) > 1.0 && secs(t, self.last_zero_command_time) < 0.2 {
                return;
            }
            self.last_zero_command_time = t;
        } else {
            self.command_timeout = t;
        }

        if !self.connected {
            return;
        }
        if self.disengaged {
            self.out.limits = self.limits(1.0, i);
            self.out.command = Some(MotorCommand::Disengage);
            return;
        }
        self.out.command = Some(MotorCommand::Speed(command));
        // allow more current to free a stuck ram
        let mul = if self.flags & (PORT_OVERCURRENT_FAULT | STARBOARD_OVERCURRENT_FAULT) != 0 {
            2.0
        } else {
            1.0
        };
        self.out.limits = self.limits(mul, i);

        // a command with no current measured means the driver isn't driving
        if self.current != 0.0 {
            self.flags &= !DRIVER_TIMEOUT;
            self.driver_timeout_start = None;
        } else if command != 0.0 {
            match self.driver_timeout_start {
                Some(start) if secs(t, start) > 1.0 => self.flags |= DRIVER_TIMEOUT,
                Some(_) => {}
                None => self.driver_timeout_start = Some(t),
            }
        }
    }

    fn limits(&self, mul: f32, i: &ServoInputs) -> MotorLimits {
        let uncorrected = (self.max_current - self.current_offset).max(0.0) / self.current_factor;
        let (offset, scale, nonlinearity) = i.rudder_calibration;
        MotorLimits {
            raw_max_current: mul * uncorrected,
            rudder_min: i.rudder_minmax.0,
            rudder_max: i.rudder_minmax.1,
            max_current: self.max_current,
            max_controller_temp: self.max_controller_temp,
            max_motor_temp: self.max_motor_temp,
            rudder_range: i.rudder_range,
            rudder_offset: offset,
            rudder_scale: scale,
            rudder_nonlinearity: nonlinearity,
            max_slew_speed: self.max_slew_speed,
            max_slew_slow: self.max_slew_slow,
            current_factor: self.current_factor,
            current_offset: self.current_offset,
            voltage_factor: self.voltage_factor,
            voltage_offset: self.voltage_offset,
            min_speed: self.speed_min,
            max_speed: self.speed_max,
            gain: self.gain,
            clutch_pwm: self.clutch_pwm,
            brake: self.brake_on,
        }
    }
}

fn placeholder_limits() -> MotorLimits {
    MotorLimits {
        raw_max_current: 0.0,
        rudder_min: -0.5,
        rudder_max: 0.5,
        max_current: 0.0,
        max_controller_temp: 0.0,
        max_motor_temp: 0.0,
        rudder_range: 0.0,
        rudder_offset: 0.0,
        rudder_scale: 0.0,
        rudder_nonlinearity: 0.0,
        max_slew_speed: 0.0,
        max_slew_slow: 0.0,
        current_factor: 1.0,
        current_offset: 0.0,
        voltage_factor: 1.0,
        voltage_offset: 0.0,
        min_speed: 0.0,
        max_speed: 0.0,
        gain: 1.0,
        clutch_pwm: 0.0,
        brake: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustpilot_types::Duration;

    fn inputs() -> ServoInputs {
        ServoInputs {
            rudder_range: 45.0,
            rudder_minmax: (-0.45, 0.45),
            rudder_calibration: (0.0, 100.0, 0.0),
            ..Default::default()
        }
    }

    #[test]
    fn pilot_commands_ignored_just_after_start() {
        let t0 = Instant::from_micros(0);
        let mut s = Servo::new(t0);
        s.connect();
        s.command.command(0.5, t0 + Duration::from_millis(500));
        assert_eq!(s.command.value, 0.0);
        s.command.command(0.5, t0 + Duration::from_millis(900));
        assert_eq!(s.command.value, 0.5);
    }

    #[test]
    fn disengaged_until_enabled_then_drives() {
        let t0 = Instant::from_micros(0);
        let mut s = Servo::new(t0);
        s.connect();
        let i = inputs();
        let mut t = t0 + Duration::from_millis(1000);
        let out = s.poll(&i, t);
        assert_eq!(out.command, Some(MotorCommand::Disengage));

        s.ap_enabled = true;
        let mut drove = false;
        for _ in 0..20 {
            t = t + Duration::from_millis(100);
            s.command.command(0.3, t);
            let out = s.poll(&i, t);
            if let Some(MotorCommand::Speed(c)) = out.command {
                assert!(c >= 0.0);
                drove |= c > 0.0;
            }
        }
        assert!(drove, "windup should produce motion at min speed");
    }

    #[test]
    fn rudder_limit_blocks_direction() {
        let t0 = Instant::from_micros(0);
        let mut s = Servo::new(t0);
        s.connect();
        s.ap_enabled = true;
        let mut i = inputs();
        i.rudder_angle = Some(46.0);
        i.telemetry = Some(MotorTelemetry {
            flags: Some(ENGAGED as u16),
            ..Default::default()
        });
        let t = t0 + Duration::from_millis(2000);
        s.command.set(1.0, t);
        let out = s.poll(&i, t);
        assert_ne!(s.flags & MAX_RUDDER_FAULT, 0);
        assert_eq!(out.command, Some(MotorCommand::Speed(0.0)));
        assert_eq!(s.state, ServoState::Stop);
    }
}
