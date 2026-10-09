//! The control loop (`autopilot.py`): one [`Autopilot::step`] per IMU
//! sample, in the same order as pypilot's `Autopilot.iteration`.

use rustpilot_hal::{ControlCommand, DeviceId, Mode, ModeSet, NavMessage, NavReading, PilotState};
use rustpilot_types::{Degrees, Duration, Instant, MotorTelemetry, SensorSource, resolv};

use crate::heading::{HeadingError, clamp_heading_command};
use crate::pilots::{BasicPilot, PilotInputs, PilotKind};
use crate::rudder::Rudder;
use crate::sensors::{Apb, Gps, Water, Wind};
use crate::servo::{Servo, ServoInputs, ServoOutput};
use crate::tacking::{Tack, TackAction, TackInputs};

/// Fused IMU output for one sample (the `imu.*` values the loop reads).
/// Phase 2's `rustpilot-imu` produces it; tests and the simulator fill it
/// directly.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImuData {
    /// Unfiltered heading, degrees (`imu.heading`).
    pub heading: Degrees,
    /// Low-passed heading, degrees (`imu.heading_lowpass`).
    pub heading_lowpass: Degrees,
    /// Low-passed turn rate, degrees per second.
    pub headingrate_lowpass: f32,
    /// Low-passed turn acceleration, degrees per second squared.
    pub headingraterate_lowpass: f32,
    /// Heel, degrees (`imu.heel`).
    pub heel: f32,
    /// Roll rate, degrees per second.
    pub rollrate: f32,
    /// Pitch rate, degrees per second.
    pub pitchrate: f32,
    /// Boat alignment heading offset, degrees (`imu.heading_offset`).
    pub heading_offset: Degrees,
    /// True on the sample after a new compass calibration was applied.
    pub compass_calibration_updated: bool,
}

/// The motor controller's side of one step.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MotorInputs {
    /// A controller is connected.
    pub connected: bool,
    /// Telemetry received since the last step.
    pub telemetry: Option<MotorTelemetry>,
}

/// Everything the loop reads in one step.
#[derive(Debug, Clone, Default)]
pub struct Inputs<'a> {
    /// A new IMU sample, if one arrived. Without one the loop reuses the
    /// last sample, as pypilot does when an IMU read fails.
    pub imu: Option<ImuData>,
    /// Navigation readings since the last step.
    pub nav: &'a [NavReading],
    /// Motor controller state.
    pub motor: MotorInputs,
    /// Commands since the last step, already checked against their link's
    /// control level.
    pub commands: &'a [ControlCommand],
}

/// Everything the loop writes in one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outputs {
    /// What to send the motor controller.
    pub servo: ServoOutput,
    /// State for the control links to publish.
    pub state: PilotState,
}

/// A heading offset that low-passes towards a target (`HeadingOffset`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct HeadingOffset(pub Degrees);

impl HeadingOffset {
    fn update(&mut self, offset: Degrees, d: f32) {
        let offset = resolv(offset, self.0);
        self.0 = resolv(d * offset + (1.0 - d) * self.0, 0.0);
    }
}

/// The autopilot.
#[derive(Debug, Clone)]
pub struct Autopilot {
    /// `ap.enabled`.
    pub enabled: bool,
    /// `ap.mode`.
    pub mode: Mode,
    /// `ap.preferred_mode`: the mode a person chose, restored when its
    /// sensor comes back.
    pub preferred_mode: Mode,
    preferred_command: Option<(Degrees, Instant)>,
    /// `ap.modes`.
    pub modes: ModeSet,
    /// `ap.gps_and_nav_modes`.
    pub gps_and_nav_modes: bool,
    /// `ap.heading_command`.
    pub heading_command: Degrees,
    /// `ap.heading`, in the current mode's frame.
    pub heading: Degrees,
    /// `ap.heading_error` and `ap.heading_error_int`.
    pub error: HeadingError,
    error_int_time: Instant,
    /// `ap.heading_command_rate`, the feed-forward input.
    pub heading_command_rate: f32,
    command_rate_time: Option<Instant>,
    last_heading_command: Degrees,
    /// `ap.gps_compass_offset`.
    pub gps_compass_offset: HeadingOffset,
    /// `ap.wind_compass_offset`.
    pub wind_compass_offset: HeadingOffset,
    /// `ap.true_wind_compass_offset`.
    pub true_wind_compass_offset: HeadingOffset,
    /// `ap.wind_offset_filter`.
    pub wind_offset_filter: f32,
    gps_speed: f32,
    /// `ap.pilot`.
    pub pilot: PilotKind,
    /// The basic pilot and its gains.
    pub basic: BasicPilot,
    /// Tacking.
    pub tack: Tack,
    /// Servo logic.
    pub servo: Servo,
    /// `n2k.control.jog_pulse` generalised to every link: seconds per jog.
    pub jog_pulse: f32,
    jog_end: Option<Instant>,
    /// Latest IMU data.
    pub imu: ImuData,
    /// Sensors.
    pub gps: Gps,
    /// Apparent wind.
    pub wind: Wind,
    /// True wind.
    pub truewind: Wind,
    /// Water speed.
    pub water: Water,
    /// Route steering.
    pub apb: Apb,
    /// Rudder.
    pub rudder: Rudder,

    lastmode: Option<Mode>,
    last_heading_mode: Option<Mode>,
    lastenabled: bool,
    last_heading: Option<Degrees>,
    last_heading_off: Degrees,
    lasttime: Instant,
    hold_heading: bool,
    motor_connected: bool,
}

const SERVO_DEVICE: &str = "servo";

impl Autopilot {
    /// A disengaged autopilot in compass mode, pypilot's start-up state.
    pub fn new(now: Instant) -> Self {
        let mut rudder = Rudder::new(now);
        rudder.update_minmax();
        Self {
            enabled: false,
            mode: Mode::Compass,
            preferred_mode: Mode::Compass,
            preferred_command: None,
            modes: ModeSet::EMPTY,
            gps_and_nav_modes: true,
            heading_command: 0.0,
            heading: 0.0,
            error: HeadingError::default(),
            error_int_time: now,
            heading_command_rate: 0.0,
            command_rate_time: None,
            last_heading_command: 0.0,
            gps_compass_offset: HeadingOffset::default(),
            wind_compass_offset: HeadingOffset::default(),
            true_wind_compass_offset: HeadingOffset::default(),
            wind_offset_filter: 0.1,
            gps_speed: 0.0,
            pilot: PilotKind::Basic,
            basic: BasicPilot::default(),
            tack: Tack::new(now),
            servo: Servo::new(now),
            jog_pulse: 0.3,
            jog_end: None,
            imu: ImuData {
                heading_offset: 90.0,
                ..ImuData::default()
            },
            gps: Gps::default(),
            wind: Wind::default(),
            truewind: Wind::default(),
            water: Water::default(),
            apb: Apb::new(now),
            rudder,
            lastmode: None,
            last_heading_mode: None,
            lastenabled: false,
            last_heading: None,
            last_heading_off: 90.0,
            lasttime: now,
            hold_heading: false,
            motor_connected: false,
        }
    }

    /// The state the control links publish.
    pub fn state(&self) -> PilotState {
        PilotState {
            enabled: self.enabled,
            mode: Some(self.mode),
            heading_command: self.heading_command,
            heading: self.heading,
            heading_error: self.error.error,
            compass_heading: self.imu.heading_lowpass,
            rudder_angle: self.rudder.angle,
            modes: self.modes,
            servo_flags: self.servo.flags,
        }
    }

    /// Set `ap.heading_command` the way `HeadingProperty.set` does.
    pub fn set_heading_command(&mut self, value: Degrees) {
        self.heading_command = clamp_heading_command(value, self.mode);
    }

    /// Set `ap.mode` as a person would (`ModeProperty.set`): this also
    /// becomes the preferred mode.
    pub fn set_mode(&mut self, mode: Mode) {
        self.preferred_mode = mode;
        self.preferred_command = None;
        self.mode = mode;
    }

    /// Run one loop iteration at time `now`.
    pub fn step(&mut self, now: Instant, inputs: &Inputs<'_>) -> Outputs {
        for cmd in inputs.commands {
            self.apply(cmd, now);
        }
        if let Some(end) = self.jog_end
            && now >= end
        {
            self.servo.command.set(0.0, now);
            self.jog_end = None;
        }

        let mut servo_out = None;
        // in standby, command the servo first for lower latency
        if !self.enabled {
            if self.lastenabled {
                self.servo.command.command(0.0, now);
            }
            servo_out = Some(self.poll_servo(&inputs.motor, now));
        }

        self.poll_sensors(inputs.nav, now);

        if let Some(imu) = inputs.imu {
            self.imu = imu;
        }
        self.fix_compass_calibration_change(inputs.imu.as_ref(), now);
        self.compute_offsets(now);
        self.compute_modes();
        self.adjust_mode(now);
        self.compute_heading();
        if self.hold_heading {
            // engaging from standby holds the current heading in the new mode
            self.hold_heading = false;
            self.set_heading_command(self.heading);
            self.lastmode = Some(self.mode);
        }
        self.compute_heading_error(now);

        // reset filters when the autopilot is enabled
        if self.enabled != self.lastenabled {
            self.lastenabled = self.enabled;
            if self.enabled {
                self.command_rate_time = None;
                self.last_heading_mode = None;
            }
        }

        let newmode = self.last_heading_mode != Some(self.mode);
        self.last_heading_mode = Some(self.mode);
        if newmode {
            self.error.reset_integral();
        }

        // reset feed-forward if the mode changed, or the last command is old
        let stale = self
            .command_rate_time
            .is_none_or(|t| now.saturating_since(t) > Duration::from_millis(1000));
        if newmode || stale {
            self.last_heading_command = self.heading_command;
        }

        if self.enabled {
            let mut diff = resolv(self.heading_command - self.last_heading_command, 0.0);
            if !self.mode.is_wind() {
                // wind modes need the opposite sign
                diff = -diff;
            }
            let lp = 0.1;
            self.heading_command_rate = (1.0 - lp) * self.heading_command_rate + lp * diff;
        } else {
            self.heading_command_rate = 0.0;
        }
        self.last_heading_command = self.heading_command;
        self.command_rate_time = Some(now);

        // tacking overrides the pilot
        let tack_inputs = TackInputs {
            enabled: self.enabled,
            heading_command: self.heading_command,
            heading: self.imu.heading_lowpass,
            headingrate: self.imu.headingrate_lowpass,
            headingraterate: self.imu.headingraterate_lowpass,
            wind_direction: self
                .wind
                .slot
                .present()
                .then(|| self.wind.direction.unwrap_or(0.0)),
        };
        match self.tack.process(&tack_inputs, self.mode, now) {
            TackAction::Servo(command) => self.servo.command.command(command, now),
            TackAction::Pilot(new_command) => {
                if let Some(h) = new_command {
                    self.set_heading_command(h);
                }
                let command = self.basic.compute(&PilotInputs {
                    heading_error: self.error.error,
                    headingrate: self.imu.headingrate_lowpass,
                    headingraterate: self.imu.headingraterate_lowpass,
                    heading_command_rate: self.heading_command_rate,
                });
                if self.enabled {
                    self.servo.command.command(command, now);
                }
            }
        }

        // the servo can only disengage under manual control
        self.servo.ap_enabled = self.enabled;
        if self.enabled {
            servo_out = Some(self.poll_servo(&inputs.motor, now));
        }

        self.tack
            .poll(tack_inputs.wind_direction, self.imu.heel, now);

        if self.heading_command_rate != 0.0 {
            // decay the integral as the heading command changes
            let e = self.error.integral;
            let s = if e > 0.0 { 1.0 } else { -1.0 };
            let new_int = (e.abs() - (self.heading_command_rate / 2.0).abs()).max(0.0);
            self.error.integral = s * new_int;
        }

        Outputs {
            servo: servo_out.expect("servo polled in one branch"),
            state: self.state(),
        }
    }

    fn apply(&mut self, cmd: &ControlCommand, now: Instant) {
        match cmd {
            ControlCommand::Standby => self.enabled = false,
            ControlCommand::Engage(mode) => {
                if let Some(m) = *mode
                    && m != self.mode
                {
                    self.set_mode(m);
                }
                if !self.enabled {
                    self.hold_heading = true;
                }
                self.enabled = true;
            }
            ControlCommand::SetMode(m) => self.set_mode(*m),
            ControlCommand::SetHeading(h) => self.set_heading_command(*h),
            ControlCommand::AdjustHeading(d) => self.set_heading_command(self.heading_command + d),
            ControlCommand::Tack(direction) => self.tack.begin(Some(*direction)),
            ControlCommand::Jog { speed } => {
                self.enabled = false;
                self.servo.command.set(*speed, now);
                self.jog_end =
                    (*speed != 0.0).then(|| now + Duration::from_secs_f32(self.jog_pulse));
            }
            ControlCommand::RudderAngle(angle) => {
                self.enabled = false;
                self.servo.position_command.set(*angle, now);
            }
            ControlCommand::SetValue { name, value } => {
                let _ = crate::values::set_json(self, name, value, now);
            }
        }
    }

    fn poll_servo(&mut self, motor: &MotorInputs, now: Instant) -> ServoOutput {
        if motor.connected != self.motor_connected {
            self.motor_connected = motor.connected;
            if motor.connected {
                self.servo.connect();
            } else {
                // pypilot's close_driver also invalidates the rudder angle
                self.rudder.angle = None;
                self.servo.disconnect();
            }
        }
        let device = DeviceId::try_from(SERVO_DEVICE).unwrap_or_default();
        if let Some(raw) = motor.telemetry.and_then(|t| t.rudder_raw)
            && raw != 0.0
        {
            if raw.is_nan() {
                if self.rudder.slot.source == SensorSource::Servo {
                    self.rudder.slot.lose();
                    self.rudder.reset();
                }
            } else if self.rudder.slot.accepts(SensorSource::Servo, &device)
                && self.rudder.update(Some(raw), &device, now)
            {
                self.rudder.slot.commit(SensorSource::Servo, &device, now);
            }
        }
        let inputs = ServoInputs {
            telemetry: motor.telemetry,
            rudder_angle: self.rudder.angle,
            rudder_range: self.rudder.range,
            rudder_minmax: self.rudder.minmax,
            rudder_calibration: (
                self.rudder.offset,
                self.rudder.scale,
                self.rudder.nonlinearity,
            ),
        };
        self.servo.poll(&inputs, now)
    }

    fn poll_sensors(&mut self, nav: &[NavReading], now: Instant) {
        for r in nav {
            self.write_sensor(r, now);
        }
        self.rudder.poll();
        if self.gps.slot.timed_out(now) {
            self.gps.slot.lose();
            self.gps.reset();
        }
        if self.wind.slot.timed_out(now) {
            self.wind.slot.lose();
            self.wind.reset();
        }
        if self.truewind.slot.timed_out(now) {
            self.truewind.slot.lose();
            self.truewind.reset();
        }
        if self.water.slot.timed_out(now) {
            self.water.slot.lose();
            self.water.reset();
        }
        if self.apb.slot.timed_out(now) {
            self.apb.slot.lose();
            self.apb.reset();
        }
        if self.rudder.slot.timed_out(now) {
            self.rudder.slot.lose();
            self.rudder.reset();
        }
    }

    fn write_sensor(&mut self, r: &NavReading, now: Instant) {
        let (src, dev) = (r.source, &r.device);
        let rates = (self.imu.rollrate, self.imu.pitchrate);
        match &r.message {
            NavMessage::Gps(fix) if self.gps.slot.accepts(src, dev) => {
                self.gps.update(fix);
                self.gps.slot.commit(src, dev, now);
            }
            NavMessage::Wind(w) if self.wind.slot.accepts(src, dev) => {
                self.wind.update(Some(w.direction), Some(w.speed), rates);
                self.wind.slot.commit(src, dev, now);
            }
            NavMessage::TrueWind(w) if self.truewind.slot.accepts(src, dev) => {
                self.truewind
                    .update(Some(w.direction), Some(w.speed), rates);
                self.truewind.slot.commit(src, dev, now);
            }
            NavMessage::Water(w) if self.water.slot.accepts(src, dev) => {
                self.water.update(w);
                self.water.slot.commit(src, dev, now);
            }
            NavMessage::Route(route) if self.apb.slot.accepts(src, dev) => {
                if let Some(command) = self.apb.update(route, now) {
                    self.apb.slot.commit(src, dev, now);
                    if let Some(c) = command
                        && self.mode == Mode::Nav
                        && self.enabled
                        && (self.heading_command - c).abs() > 0.1
                    {
                        self.set_heading_command(c);
                    }
                }
            }
            NavMessage::Rudder(raw)
                if self.rudder.slot.accepts(src, dev)
                    && self.rudder.update(Some(*raw), dev, now) =>
            {
                self.rudder.slot.commit(src, dev, now);
            }
            _ => {}
        }
    }

    /// Keep the same course when the compass calibration or boat alignment
    /// changes (`fix_compass_calibration_change`).
    fn fix_compass_calibration_change(&mut self, data: Option<&ImuData>, now: Instant) {
        let headingrate = self.imu.headingrate_lowpass;
        let dt = now.saturating_since(self.lasttime).as_secs_f32().min(0.25);
        self.lasttime = now;
        let mut change = 0.0;
        if let Some(d) = data {
            if d.compass_calibration_updated
                && let Some(last) = self.last_heading
            {
                let last = resolv(last, d.heading);
                change += d.heading - headingrate * dt - last;
            }
            self.last_heading = Some(d.heading);
        }
        let off = self.imu.heading_offset;
        if self.last_heading_off != off {
            let last = resolv(self.last_heading_off, off);
            change += off - last;
            self.last_heading_off = off;
        }
        if change != 0.0 {
            self.gps_compass_offset.0 -= change;
            self.wind_compass_offset.0 += change;
            self.true_wind_compass_offset.0 += change;
            if self.mode == Mode::Compass && self.enabled {
                self.heading_command =
                    clamp_heading_command(self.heading_command + change, Mode::Compass);
            }
            self.command_rate_time = None;
        }
    }

    /// Track the offsets between compass and GPS track and wind angles, and
    /// synthesise true wind (`compute_offsets`).
    fn compute_offsets(&mut self, now: Instant) {
        let compass = self.imu.heading_lowpass;
        if self.gps.slot.present() {
            let d = 0.002;
            let gps_speed = self.gps.speed.unwrap_or(0.0);
            self.gps_speed = (1.0 - d) * self.gps_speed + d * gps_speed;
            if gps_speed > 1.0 {
                let track = self.gps.track.unwrap_or(0.0);
                // weight the offset more at higher speed
                let d = 0.005 * libm::logf(self.gps_speed + 1.0);
                self.gps_compass_offset.update(track - compass, d);
            }
        }

        if self.wind.slot.present() {
            let offset = resolv(
                self.wind.filtered_direction + compass,
                self.wind_compass_offset.0,
            );
            self.wind_compass_offset
                .update(offset, self.wind_offset_filter);

            let mut boat_speed = None;
            if self.water.slot.present() {
                boat_speed = Some(self.water.speed.unwrap_or(0.0));
                if !self.truewind.slot.present() {
                    self.truewind.slot.source = SensorSource::WaterWind;
                }
            } else if self.gps.slot.present() {
                boat_speed = Some(self.gps_speed);
                if !self.truewind.slot.present() {
                    self.truewind.slot.source = SensorSource::GpsWind;
                }
            }
            if let Some(b) = boat_speed {
                let ws = self.wind.speed.unwrap_or(0.0);
                let wd = self.wind.direction.unwrap_or(0.0);
                self.truewind.update_from_apparent(b, ws, wd, now);
            }
        }

        if self.truewind.slot.present() {
            let offset = resolv(
                self.truewind.filtered_direction + compass,
                self.true_wind_compass_offset.0,
            );
            self.true_wind_compass_offset
                .update(offset, self.wind_offset_filter);
        }
    }

    fn compute_modes(&mut self) {
        let gps = self.gps.slot.present();
        let apb = self.apb.slot.present();
        let mut modes = ModeSet::EMPTY;
        modes.insert(Mode::Compass);
        if gps {
            if self.gps_and_nav_modes || !apb {
                modes.insert(Mode::Gps);
            }
            if apb {
                modes.insert(Mode::Nav);
            }
        }
        if self.wind.slot.present() {
            modes.insert(Mode::Wind);
        }
        if self.truewind.slot.present() {
            modes.insert(Mode::TrueWind);
        }
        self.modes = modes;
    }

    /// The best available mode for `mode` (`AutopilotPilot.best_mode`).
    pub fn best_mode(&self, mut mode: Mode) -> Mode {
        while !self.modes.contains(mode) {
            mode = match mode {
                Mode::Nav => Mode::Gps,
                Mode::Gps | Mode::Wind => Mode::Compass,
                Mode::TrueWind => Mode::Wind,
                Mode::Compass => break,
            };
        }
        mode
    }

    fn adjust_mode(&mut self, now: Instant) {
        let newmode = self.best_mode(self.preferred_mode);
        if self.mode != newmode {
            // remember the preferred mode's command so it can be restored
            if self.mode == self.preferred_mode {
                self.preferred_command = Some((self.heading_command, now));
            }
            self.mode = newmode;
        }
    }

    fn compute_heading(&mut self) {
        let compass = self.imu.heading_lowpass;
        self.heading = match self.mode {
            Mode::TrueWind => resolv(self.true_wind_compass_offset.0 - compass, 0.0),
            Mode::Wind => resolv(self.wind_compass_offset.0 - compass, 0.0),
            Mode::Gps | Mode::Nav => resolv(compass + self.gps_compass_offset.0, 180.0),
            Mode::Compass => compass,
        };
    }

    fn compute_heading_error(&mut self, now: Instant) {
        let heading = self.heading;
        // keep the same course if the mode changes
        if self.lastmode != Some(self.mode) {
            match self.preferred_command {
                Some((command, t))
                    if self.mode == self.preferred_mode
                        && now.saturating_since(t) < Duration::from_millis(30_000) =>
                {
                    // the preferred mode's sensor came back
                    self.set_heading_command(command);
                }
                _ => {
                    let mut error = self.error.error;
                    if self.mode.is_wind() {
                        error = -error;
                    }
                    self.set_heading_command(heading - error);
                }
            }
            self.lastmode = Some(self.mode);
        }
        let dt = now.saturating_since(self.error_int_time).as_secs_f32();
        self.error_int_time = now;
        self.error
            .update(heading, self.heading_command, self.mode, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustpilot_hal::MotorCommand;

    fn imu(heading: f32) -> ImuData {
        ImuData {
            heading,
            heading_lowpass: heading,
            heading_offset: 90.0,
            ..ImuData::default()
        }
    }

    fn ms(t: u64) -> Instant {
        Instant::from_micros(t * 1000)
    }

    #[test]
    fn engage_holds_heading_and_steers_back() {
        let mut ap = Autopilot::new(ms(0));
        let motor = MotorInputs {
            connected: true,
            telemetry: None,
        };
        let out = ap.step(
            ms(100),
            &Inputs {
                imu: Some(imu(123.4)),
                motor,
                ..Default::default()
            },
        );
        assert!(!out.state.enabled);
        assert_eq!(out.servo.command, Some(MotorCommand::Disengage));

        let engage = [ControlCommand::Engage(None)];
        let out = ap.step(
            ms(200),
            &Inputs {
                imu: Some(imu(123.4)),
                motor,
                commands: &engage,
                ..Default::default()
            },
        );
        assert!(out.state.enabled);
        assert_eq!(out.state.heading_command, 123.4);

        // drift 10° to starboard of the command: the pilot drives to port,
        // in bursts once the servo's windup passes its period threshold
        let mut port = 0;
        for i in 0..100 {
            let out = ap.step(
                ms(300 + i * 100),
                &Inputs {
                    imu: Some(imu(133.4)),
                    motor,
                    ..Default::default()
                },
            );
            assert!((out.state.heading_error - 10.0).abs() < 1e-3);
            if let Some(MotorCommand::Speed(c)) = out.servo.command {
                assert!(c >= 0.0, "expected port or stop, got {c}");
                port += (c > 0.0) as u32;
            }
        }
        assert!(port > 0);
    }

    #[test]
    fn wind_mode_falls_back_and_restores() {
        let mut ap = Autopilot::new(ms(0));
        let dev = DeviceId::try_from("/dev/ttyUSB0").unwrap();
        let wind = |t| NavReading {
            source: SensorSource::Serial,
            device: dev.clone(),
            time: ms(t),
            message: NavMessage::Wind(rustpilot_types::Wind {
                direction: 40.0,
                speed: 12.0,
            }),
        };
        let r = [wind(100)];
        ap.step(
            ms(100),
            &Inputs {
                imu: Some(imu(0.0)),
                nav: &r,
                ..Default::default()
            },
        );
        let cmds = [ControlCommand::SetMode(Mode::Wind)];
        let out = ap.step(
            ms(200),
            &Inputs {
                imu: Some(imu(0.0)),
                commands: &cmds,
                ..Default::default()
            },
        );
        assert_eq!(out.state.mode, Some(Mode::Wind));
        // wind lost after 8 s: fall back to compass
        let out = ap.step(
            ms(8200),
            &Inputs {
                imu: Some(imu(0.0)),
                ..Default::default()
            },
        );
        assert_eq!(out.state.mode, Some(Mode::Compass));
        assert_eq!(ap.preferred_mode, Mode::Wind);
        // wind back: wind mode returns
        let r = [wind(9000)];
        let out = ap.step(
            ms(9000),
            &Inputs {
                imu: Some(imu(0.0)),
                nav: &r,
                ..Default::default()
            },
        );
        assert_eq!(out.state.mode, Some(Mode::Wind));
    }
}
