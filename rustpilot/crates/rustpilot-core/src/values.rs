//! pypilot's named values (`ap.heading_command`, `servo.max_current`...):
//! the table every client protocol uses to read and write the core.
//!
//! The JSON server (phase 2) and N2K `PP:` text commands both go through
//! [`find`], [`get`] and [`set`], so names, ranges and types stay exactly
//! pypilot's.

use rustpilot_hal::{Mode, TackDirection};
use rustpilot_types::Instant;

use crate::autopilot::Autopilot;
use crate::rudder::CalibrationStep;

/// A value as the protocol sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Value {
    /// No reading (pypilot's `false` for a sensor value).
    None,
    /// A number.
    Number(f32),
    /// A boolean.
    Bool(bool),
    /// An enum choice or fixed string.
    Text(&'static str),
}

impl From<f32> for Value {
    fn from(v: f32) -> Self {
        Value::Number(v)
    }
}

impl From<Option<f32>> for Value {
    fn from(v: Option<f32>) -> Self {
        v.map_or(Value::None, Value::Number)
    }
}

impl From<bool> for Value {
    fn from(v: bool) -> Self {
        Value::Bool(v)
    }
}

impl Value {
    fn number(self) -> Option<f32> {
        match self {
            Value::Number(n) => Some(n),
            _ => None,
        }
    }
}

/// What kind of value it is, matching pypilot's `info` types.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Kind {
    /// A read-only measurement (`SensorValue`).
    Sensor {
        /// Angles that wrap (`directional`).
        directional: bool,
    },
    /// A read-only plain value.
    Value,
    /// A number in a range (`RangeProperty` / `RangeSetting`).
    Range {
        /// Minimum.
        min: f32,
        /// Maximum.
        max: f32,
        /// Units, for `RangeSetting`s.
        units: Option<&'static str>,
    },
    /// A boolean a client may set (`BooleanProperty`).
    Bool,
    /// One of a fixed list (`EnumProperty`).
    Enum(&'static [&'static str]),
    /// A writable number with no range (`Property`, `TimedProperty`).
    Property,
}

type Getter = fn(&Autopilot) -> Value;
type Setter = fn(&mut Autopilot, Value, Instant) -> bool;

/// One named value.
pub struct ValueDef {
    /// Full pypilot name.
    pub name: &'static str,
    /// Type and limits.
    pub kind: Kind,
    /// Saved across restarts.
    pub persistent: bool,
    /// Saved per profile.
    pub profiled: bool,
    get: Getter,
    set: Option<Setter>,
}

impl core::fmt::Debug for ValueDef {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ValueDef")
            .field("name", &self.name)
            .field("kind", &self.kind)
            .finish()
    }
}

impl ValueDef {
    /// Whether clients may write it.
    pub fn writable(&self) -> bool {
        self.set.is_some()
    }
}

const MODES: &[&str] = &["compass", "gps", "nav", "wind", "true wind"];
const PILOTS: &[&str] = &["basic"];
const TACK_STATES: &[&str] = &["none", "begin", "waiting", "tacking"];
const TACK_DIRECTIONS: &[&str] = &["none", "port", "starboard"];
const RUDDER_CAL: &[&str] = &[
    "idle",
    "reset",
    "centered",
    "starboard range",
    "port range",
    "auto gain",
];

const fn sensor(name: &'static str, get: Getter) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Sensor { directional: false },
        persistent: false,
        profiled: false,
        get,
        set: None,
    }
}

const fn heading_sensor(name: &'static str, get: Getter) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Sensor { directional: true },
        persistent: false,
        profiled: false,
        get,
        set: None,
    }
}

const fn value(name: &'static str, get: Getter) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Value,
        persistent: false,
        profiled: false,
        get,
        set: None,
    }
}

const fn range(
    name: &'static str,
    min: f32,
    max: f32,
    persistent: bool,
    get: Getter,
    set: Setter,
) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Range {
            min,
            max,
            units: None,
        },
        persistent,
        profiled: false,
        get,
        set: Some(set),
    }
}

/// A `RangeSetting`: always persistent, with units.
const fn setting(
    name: &'static str,
    min: f32,
    max: f32,
    units: &'static str,
    profiled: bool,
    get: Getter,
    set: Setter,
) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Range {
            min,
            max,
            units: Some(units),
        },
        persistent: true,
        profiled,
        get,
        set: Some(set),
    }
}

const fn boolean(name: &'static str, persistent: bool, get: Getter, set: Setter) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Bool,
        persistent,
        profiled: false,
        get,
        set: Some(set),
    }
}

const fn choice(
    name: &'static str,
    choices: &'static [&'static str],
    persistent: bool,
    get: Getter,
    set: Setter,
) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Enum(choices),
        persistent,
        profiled: persistent,
        get,
        set: Some(set),
    }
}

const fn property(name: &'static str, get: Getter, set: Setter) -> ValueDef {
    ValueDef {
        name,
        kind: Kind::Property,
        persistent: false,
        profiled: false,
        get,
        set: Some(set),
    }
}

/// Set an `f32` field from a number value. Range checks happen in [`set`].
macro_rules! num {
    ($($field:tt)+) => {
        |ap: &mut Autopilot, v: Value, _t: Instant| match v.number() {
            Some(n) => {
                ap.$($field)+ = n;
                true
            }
            None => false,
        }
    };
}

macro_rules! flag {
    ($($field:tt)+) => {
        |ap: &mut Autopilot, v: Value, _t: Instant| match v {
            Value::Bool(b) => {
                ap.$($field)+ = b;
                true
            }
            Value::Number(n) => {
                ap.$($field)+ = n != 0.0;
                true
            }
            _ => false,
        }
    };
}

macro_rules! gain {
    ($i:expr, $name:literal, $max:expr) => {
        ValueDef {
            name: concat!("ap.pilot.basic.", $name),
            kind: Kind::Range {
                min: 0.0,
                max: $max,
                units: None,
            },
            persistent: true,
            profiled: true,
            get: |ap| ap.basic.gains[$i].value.into(),
            set: Some(num!(basic.gains[$i].value)),
        }
    };
}

macro_rules! wind_values {
    ($w:ident, $p:literal) => {
        [
            value(concat!($p, ".source"), |ap| {
                Value::Text(ap.$w.slot.source.name())
            }),
            heading_sensor(concat!($p, ".direction"), |ap| ap.$w.direction.into()),
            sensor(concat!($p, ".speed"), |ap| ap.$w.speed.into()),
            setting(
                concat!($p, ".offset"),
                -180.0,
                180.0,
                "deg",
                false,
                |ap| ap.$w.offset.into(),
                num!($w.offset),
            ),
            setting(
                concat!($p, ".sensors_height"),
                0.0,
                100.0,
                "meters",
                false,
                |ap| ap.$w.compensation_height.into(),
                num!($w.compensation_height),
            ),
            sensor(concat!($p, ".filtered_speed"), |ap| {
                ap.$w.filtered_speed.into()
            }),
            heading_sensor(concat!($p, ".filtered_direction"), |ap| {
                ap.$w.filtered_direction.into()
            }),
            range(
                concat!($p, ".filter_constant"),
                0.01,
                1.0,
                true,
                |ap| ap.$w.filter_constant.into(),
                num!($w.filter_constant),
            ),
            sensor(concat!($p, ".filter_factor"), |ap| {
                ap.$w.filter_factor.into()
            }),
        ]
    };
}

const AP: &[ValueDef] = &[
    boolean(
        "ap.enabled",
        false,
        |ap| ap.enabled.into(),
        |ap, v, _| match v {
            Value::Bool(b) => {
                ap.enabled = b;
                true
            }
            _ => false,
        },
    ),
    choice(
        "ap.mode",
        MODES,
        false,
        |ap| Value::Text(ap.mode.name()),
        |ap, v, _| match v {
            Value::Text(s) => match Mode::from_name(s) {
                Some(m) => {
                    ap.set_mode(m);
                    true
                }
                None => false,
            },
            _ => false,
        },
    ),
    value("ap.preferred_mode", |ap| {
        Value::Text(ap.preferred_mode.name())
    }),
    ValueDef {
        name: "ap.heading_command",
        kind: Kind::Range {
            min: -180.0,
            max: 360.0,
            units: None,
        },
        persistent: false,
        profiled: false,
        get: |ap| ap.heading_command.into(),
        set: Some(|ap, v, _| match v.number() {
            Some(h) => {
                ap.set_heading_command(h);
                true
            }
            None => false,
        }),
    },
    boolean(
        "ap.gps_and_nav_modes",
        true,
        |ap| ap.gps_and_nav_modes.into(),
        flag!(gps_and_nav_modes),
    ),
    heading_sensor("ap.heading", |ap| ap.heading.into()),
    sensor("ap.heading_error", |ap| ap.error.error.into()),
    sensor("ap.heading_error_int", |ap| ap.error.integral.into()),
    sensor("ap.heading_command_rate", |ap| {
        ap.heading_command_rate.into()
    }),
    heading_sensor("ap.gps_compass_offset", |ap| ap.gps_compass_offset.0.into()),
    heading_sensor("ap.wind_compass_offset", |ap| {
        ap.wind_compass_offset.0.into()
    }),
    heading_sensor("ap.true_wind_compass_offset", |ap| {
        ap.true_wind_compass_offset.0.into()
    }),
    range(
        "ap.wind_offset_filter",
        0.01,
        0.5,
        true,
        |ap| ap.wind_offset_filter.into(),
        num!(wind_offset_filter),
    ),
    choice(
        "ap.pilot",
        PILOTS,
        true,
        |ap| Value::Text(ap.pilot.name()),
        |_, v, _| v == Value::Text("basic"),
    ),
    gain!(0, "P", 0.03),
    gain!(1, "D", 0.24),
    gain!(2, "DD", 0.24),
    gain!(3, "PR", 0.02),
    gain!(4, "FF", 2.4),
    sensor("ap.pilot.basic.Pgain", |ap| {
        ap.basic.gains[0].contribution.into()
    }),
    sensor("ap.pilot.basic.Dgain", |ap| {
        ap.basic.gains[1].contribution.into()
    }),
    sensor("ap.pilot.basic.DDgain", |ap| {
        ap.basic.gains[2].contribution.into()
    }),
    sensor("ap.pilot.basic.PRgain", |ap| {
        ap.basic.gains[3].contribution.into()
    }),
    sensor("ap.pilot.basic.FFgain", |ap| {
        ap.basic.gains[4].contribution.into()
    }),
    // tacking
    ValueDef {
        name: "ap.tack.state",
        kind: Kind::Enum(TACK_STATES),
        persistent: false,
        profiled: false,
        get: |ap| Value::Text(ap.tack.state.name()),
        // only begin and none may be set
        set: Some(|ap, v, _| match v {
            Value::Text("begin") => {
                ap.tack.begin(None);
                true
            }
            Value::Text("none") => {
                ap.tack.cancel();
                true
            }
            _ => false,
        }),
    },
    value("ap.tack.timeout", |ap| ap.tack.timeout.into()),
    setting(
        "ap.tack.delay",
        0.0,
        60.0,
        "sec",
        true,
        |ap| ap.tack.delay.into(),
        num!(tack.delay),
    ),
    setting(
        "ap.tack.angle",
        10.0,
        180.0,
        "deg",
        true,
        |ap| ap.tack.angle.into(),
        num!(tack.angle),
    ),
    setting(
        "ap.tack.rate",
        1.0,
        100.0,
        "deg/s",
        true,
        |ap| ap.tack.rate.into(),
        num!(tack.rate),
    ),
    setting(
        "ap.tack.threshold",
        10.0,
        100.0,
        "%",
        true,
        |ap| ap.tack.threshold.into(),
        num!(tack.threshold),
    ),
    value("ap.tack.count", |ap| (ap.tack.count as f32).into()),
    ValueDef {
        name: "ap.tack.direction",
        kind: Kind::Enum(TACK_DIRECTIONS),
        persistent: false,
        profiled: false,
        get: |ap| {
            Value::Text(match ap.tack.direction {
                None => "none",
                Some(TackDirection::Port) => "port",
                Some(TackDirection::Starboard) => "starboard",
            })
        },
        set: Some(|ap, v, _| {
            ap.tack.direction = match v {
                Value::Text("none") => None,
                Value::Text("port") => Some(TackDirection::Port),
                Value::Text("starboard") => Some(TackDirection::Starboard),
                _ => return false,
            };
            true
        }),
    },
];

const IMU: &[ValueDef] = &[
    heading_sensor("imu.heading", |ap| ap.imu.heading.into()),
    heading_sensor("imu.heading_lowpass", |ap| ap.imu.heading_lowpass.into()),
    sensor("imu.headingrate_lowpass", |ap| {
        ap.imu.headingrate_lowpass.into()
    }),
    sensor("imu.headingraterate_lowpass", |ap| {
        ap.imu.headingraterate_lowpass.into()
    }),
    sensor("imu.heel", |ap| ap.imu.heel.into()),
];

const SERVO: &[ValueDef] = &[
    property(
        "servo.command",
        |ap| ap.servo.command.value.into(),
        |ap, v, t| match v.number() {
            Some(n) => {
                ap.servo.command.set(n, t);
                true
            }
            None => false,
        },
    ),
    property(
        "servo.position_command",
        |ap| ap.servo.position_command.value.into(),
        |ap, v, t| match v.number() {
            Some(n) => {
                ap.servo.position_command.set(n, t);
                true
            }
            None => false,
        },
    ),
    range(
        "servo.speed_gain",
        0.0,
        1.0,
        false,
        |ap| ap.servo.speed_gain.into(),
        num!(servo.speed_gain),
    ),
    sensor("servo.duty", |ap| ap.servo.duty.into()),
    value("servo.faults", |ap| (ap.servo.faults as f32).into()),
    sensor("servo.voltage", |ap| ap.servo.voltage.into()),
    sensor("servo.current", |ap| ap.servo.current.into()),
    sensor("servo.current.noise", |ap| ap.servo.current_noise.into()),
    sensor("servo.controller_temp", |ap| {
        ap.servo.controller_temp.into()
    }),
    sensor("servo.motor_temp", |ap| ap.servo.motor_temp.into()),
    value("servo.engaged", |ap| ap.servo.engaged.into()),
    ValueDef {
        name: "servo.max_current",
        kind: Kind::Range {
            min: 0.0,
            max: 50.0,
            units: Some("amps"),
        },
        persistent: true,
        profiled: false,
        get: |ap| ap.servo.max_current.into(),
        set: Some(|ap, v, _| v.number().is_some_and(|n| ap.servo.set_max_current(n))),
    },
    range(
        "servo.current.factor",
        0.8,
        1.2,
        true,
        |ap| ap.servo.current_factor.into(),
        num!(servo.current_factor),
    ),
    range(
        "servo.current.offset",
        -1.2,
        1.2,
        true,
        |ap| ap.servo.current_offset.into(),
        num!(servo.current_offset),
    ),
    range(
        "servo.voltage.factor",
        0.8,
        1.2,
        true,
        |ap| ap.servo.voltage_factor.into(),
        num!(servo.voltage_factor),
    ),
    range(
        "servo.voltage.offset",
        -1.2,
        1.2,
        true,
        |ap| ap.servo.voltage_offset.into(),
        num!(servo.voltage_offset),
    ),
    range(
        "servo.max_controller_temp",
        45.0,
        80.0,
        true,
        |ap| ap.servo.max_controller_temp.into(),
        num!(servo.max_controller_temp),
    ),
    range(
        "servo.max_motor_temp",
        30.0,
        80.0,
        true,
        |ap| ap.servo.max_motor_temp.into(),
        num!(servo.max_motor_temp),
    ),
    setting(
        "servo.max_slew_speed",
        0.0,
        100.0,
        "",
        false,
        |ap| ap.servo.max_slew_speed.into(),
        num!(servo.max_slew_speed),
    ),
    setting(
        "servo.max_slew_slow",
        0.0,
        100.0,
        "",
        false,
        |ap| ap.servo.max_slew_slow.into(),
        num!(servo.max_slew_slow),
    ),
    range(
        "servo.gain",
        -10.0,
        10.0,
        true,
        |ap| ap.servo.gain.into(),
        num!(servo.gain),
    ),
    range(
        "servo.clutch_pwm",
        10.0,
        100.0,
        true,
        |ap| ap.servo.clutch_pwm.into(),
        num!(servo.clutch_pwm),
    ),
    boolean(
        "servo.use_brake",
        true,
        |ap| ap.servo.use_brake.into(),
        flag!(servo.use_brake),
    ),
    setting(
        "servo.period",
        0.1,
        1.0,
        "sec",
        true,
        |ap| ap.servo.period.into(),
        num!(servo.period),
    ),
    boolean(
        "servo.compensate_voltage",
        true,
        |ap| ap.servo.compensate_voltage.into(),
        flag!(servo.compensate_voltage),
    ),
    value("servo.amp_hours", |ap| ap.servo.amp_hours.into()),
    sensor("servo.watts", |ap| ap.servo.watts.into()),
    sensor("servo.speed", |ap| ap.servo.speed.into()),
    // speed.min pushes speed.max up; speed.max can't go below speed.min
    setting(
        "servo.speed.min",
        0.0,
        100.0,
        "%",
        true,
        |ap| ap.servo.speed_min.into(),
        |ap, v, _| match v.number() {
            Some(n) => {
                ap.servo.speed_min = n;
                ap.servo.speed_max = ap.servo.speed_max.max(n);
                true
            }
            None => false,
        },
    ),
    setting(
        "servo.speed.max",
        0.0,
        100.0,
        "%",
        true,
        |ap| ap.servo.speed_max.into(),
        |ap, v, _| match v.number() {
            Some(n) => {
                ap.servo.speed_max = n.max(ap.servo.speed_min);
                true
            }
            None => false,
        },
    ),
    sensor("servo.position", |ap| ap.servo.position.into()),
    range(
        "servo.position.p",
        0.01,
        1.0,
        true,
        |ap| ap.servo.position_pid.0.into(),
        num!(servo.position_pid.0),
    ),
    range(
        "servo.position.i",
        0.0,
        0.1,
        true,
        |ap| ap.servo.position_pid.1.into(),
        num!(servo.position_pid.1),
    ),
    range(
        "servo.position.d",
        0.0,
        0.1,
        true,
        |ap| ap.servo.position_pid.2.into(),
        num!(servo.position_pid.2),
    ),
    sensor("servo.raw_command", |ap| ap.servo.raw_command.into()),
    value("servo.state", |ap| Value::Text(ap.servo.state.name())),
    value("servo.controller", |ap| {
        Value::Text(if ap.servo.connected {
            "arduino"
        } else {
            "none"
        })
    }),
];

const RUDDER: &[ValueDef] = &[
    value("rudder.source", |ap| {
        Value::Text(ap.rudder.slot.source.name())
    }),
    sensor("rudder.angle", |ap| ap.rudder.angle.into()),
    sensor("rudder.speed", |ap| ap.rudder.speed.into()),
    ValueDef {
        name: "rudder.offset",
        kind: Kind::Value,
        persistent: true,
        profiled: false,
        get: |ap| ap.rudder.offset.into(),
        set: None,
    },
    ValueDef {
        name: "rudder.scale",
        kind: Kind::Value,
        persistent: true,
        profiled: false,
        get: |ap| ap.rudder.scale.into(),
        set: None,
    },
    ValueDef {
        name: "rudder.nonlinearity",
        kind: Kind::Value,
        persistent: true,
        profiled: false,
        get: |ap| ap.rudder.nonlinearity.into(),
        set: None,
    },
    choice(
        "rudder.calibration_state",
        RUDDER_CAL,
        false,
        |_| Value::Text("idle"),
        |ap, v, _| {
            let step = match v {
                Value::Text("reset") => CalibrationStep::Reset,
                Value::Text("centered") => CalibrationStep::Centered,
                Value::Text("starboard range") => CalibrationStep::StarboardRange,
                Value::Text("port range") => CalibrationStep::PortRange,
                Value::Text("idle") | Value::Text("auto gain") => return true,
                _ => return false,
            };
            ap.rudder.calibrate(step);
            true
        },
    ),
    range(
        "rudder.range",
        10.0,
        100.0,
        true,
        |ap| ap.rudder.range.into(),
        num!(rudder.range),
    ),
];

const NAV: &[ValueDef] = &[
    value("gps.source", |ap| Value::Text(ap.gps.slot.source.name())),
    heading_sensor("gps.track", |ap| ap.gps.track.into()),
    sensor("gps.speed", |ap| ap.gps.speed.into()),
    sensor("gps.declination", |ap| ap.gps.declination.into()),
    value("water.source", |ap| {
        Value::Text(ap.water.slot.source.name())
    }),
    sensor("water.speed", |ap| ap.water.speed.into()),
    sensor("water.leeway", |ap| ap.water.leeway.into()),
    value("apb.source", |ap| Value::Text(ap.apb.slot.source.name())),
    heading_sensor("apb.track", |ap| ap.apb.track.into()),
    sensor("apb.xte", |ap| ap.apb.xte.into()),
    range(
        "apb.xte.gain",
        0.0,
        3000.0,
        true,
        |ap| ap.apb.gain.into(),
        num!(apb.gain),
    ),
];

const WIND: [ValueDef; 9] = wind_values!(wind, "wind");
const TRUEWIND: [ValueDef; 9] = wind_values!(truewind, "truewind");

const TABLES: [&[ValueDef]; 7] = [AP, IMU, SERVO, RUDDER, NAV, &WIND, &TRUEWIND];

/// Every value, in table order.
pub fn all() -> impl Iterator<Item = &'static ValueDef> {
    TABLES.into_iter().flatten()
}

/// Look up a value by name.
pub fn find(name: &str) -> Option<&'static ValueDef> {
    all().find(|d| d.name == name)
}

/// Read a value by name.
pub fn get(ap: &Autopilot, name: &str) -> Option<Value> {
    find(name).map(|d| (d.get)(ap))
}

/// Why a write was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetError {
    /// No such value.
    Unknown,
    /// The value is read-only.
    ReadOnly,
    /// Wrong type, out of range, or not one of the choices. pypilot ignores
    /// such writes; so does RustPilot, but it says so.
    Invalid,
}

/// Write a value by name, with pypilot's checks: numbers must be in range
/// and enum values one of the choices, else the write is ignored.
pub fn set(ap: &mut Autopilot, name: &str, v: Value, now: Instant) -> Result<(), SetError> {
    let def = find(name).ok_or(SetError::Unknown)?;
    let setter = def.set.ok_or(SetError::ReadOnly)?;
    let v = match (def.kind, v) {
        (Kind::Range { min, max, .. }, Value::Number(n)) if (min..=max).contains(&n) => v,
        (Kind::Range { .. }, _) => return Err(SetError::Invalid),
        (Kind::Enum(choices), Value::Text(s)) => match choices.iter().find(|c| **c == s) {
            Some(c) => Value::Text(c),
            None => return Err(SetError::Invalid),
        },
        (Kind::Enum(_), _) => return Err(SetError::Invalid),
        _ => v,
    };
    if setter(ap, v, now) {
        Ok(())
    } else {
        Err(SetError::Invalid)
    }
}

/// Write a value from its JSON text, as pypilot's protocol and `PP:`
/// commands carry it: a number, `true`/`false`, or a quoted string.
pub fn set_json(ap: &mut Autopilot, name: &str, json: &str, now: Instant) -> Result<(), SetError> {
    let def = find(name).ok_or(SetError::Unknown)?;
    let json = json.trim();
    let v = match json {
        "true" => Value::Bool(true),
        "false" => Value::Bool(false),
        s if s.len() >= 2 && s.starts_with('"') && s.ends_with('"') => {
            let inner = &s[1..s.len() - 1];
            // map to the 'static choice so setters can match on it
            match def.kind {
                Kind::Enum(choices) => match choices.iter().find(|c| **c == inner) {
                    Some(c) => Value::Text(c),
                    None => return Err(SetError::Invalid),
                },
                _ => return Err(SetError::Invalid),
            }
        }
        s => Value::Number(s.parse::<f32>().map_err(|_| SetError::Invalid)?),
    };
    set(ap, name, v, now)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_are_unique() {
        for (i, a) in all().enumerate() {
            for b in all().skip(i + 1) {
                assert_ne!(a.name, b.name);
            }
        }
    }

    #[test]
    fn set_with_pypilot_rules() {
        let t = Instant::from_micros(0);
        let mut ap = Autopilot::new(t);
        assert_eq!(set_json(&mut ap, "ap.pilot.basic.P", "0.005", t), Ok(()));
        assert_eq!(get(&ap, "ap.pilot.basic.P"), Some(Value::Number(0.005)));
        // out of range is ignored
        assert_eq!(
            set_json(&mut ap, "ap.pilot.basic.P", "0.5", t),
            Err(SetError::Invalid)
        );
        assert_eq!(get(&ap, "ap.pilot.basic.P"), Some(Value::Number(0.005)));
        assert_eq!(set_json(&mut ap, "ap.mode", "\"wind\"", t), Ok(()));
        assert_eq!(ap.mode, Mode::Wind);
        assert_eq!(ap.preferred_mode, Mode::Wind);
        assert_eq!(
            set_json(&mut ap, "ap.mode", "\"sideways\"", t),
            Err(SetError::Invalid)
        );
        assert_eq!(set_json(&mut ap, "ap.heading_command", "-10", t), Ok(()));
        assert_eq!(ap.heading_command, -10.0); // wind mode keeps ±180
        assert_eq!(
            set_json(&mut ap, "ap.heading", "5", t),
            Err(SetError::ReadOnly)
        );
        assert_eq!(set_json(&mut ap, "servo.speed.min", "60", t), Ok(()));
        assert_eq!(set_json(&mut ap, "servo.speed.max", "40", t), Ok(()));
        assert_eq!(ap.servo.speed_max, 60.0);
    }
}
