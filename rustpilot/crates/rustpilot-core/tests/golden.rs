//! Parity with pypilot: replay the inputs recorded by
//! `tools/golden/generate.py`, which ran pypilot's own control loop, and
//! compare every step's motor commands and values.

use std::io::{BufRead, BufReader};

use flate2::read::GzDecoder;
use rustpilot_core::values::{self, Value};
use rustpilot_core::{Autopilot, ImuData, Inputs, MotorInputs};
use rustpilot_hal::{ControlCommand, DeviceId, MotorCommand, NavMessage, NavReading};
use rustpilot_types::{GpsFix, Instant, MotorTelemetry, RouteSteer, SensorSource, Wind};
use serde_json::Value as J;

fn instant(t: f64) -> Instant {
    Instant::from_micros((t * 1e6).round() as u64)
}

fn f(v: &J) -> f32 {
    v.as_f64().unwrap_or(f64::NAN) as f32
}

fn source(s: &str) -> SensorSource {
    match s {
        "serial" => SensorSource::Serial,
        "tcp" => SensorSource::Tcp,
        "signalk" => SensorSource::Signalk,
        "gpsd" => SensorSource::Gpsd,
        "servo" => SensorSource::Servo,
        other => panic!("source {other}"),
    }
}

/// Compare one recorded pypilot value with RustPilot's.
fn matches(py: &J, rs: Value) -> bool {
    match (py, rs) {
        (J::Bool(false), Value::None) => true,
        (J::Bool(b), Value::Bool(r)) => *b == r,
        (J::String(s), Value::Text(r)) => s == r,
        (J::Number(n), Value::Number(r)) => {
            let n = n.as_f64().unwrap() as f32;
            (n - r).abs() <= 2e-3 * n.abs().max(1.0)
        }
        // pypilot's sensor values start as False; RustPilot's as 0
        (J::Bool(false), Value::Number(r)) => r == 0.0,
        _ => false,
    }
}

fn replay(name: &str) {
    let path = format!(
        "{}/tests/golden/{name}.jsonl.gz",
        env!("CARGO_MANIFEST_DIR")
    );
    let file = std::fs::File::open(&path).unwrap();
    let lines = BufReader::new(GzDecoder::new(file)).lines();

    let mut ap: Option<Autopilot> = None;
    let mut mismatches = Vec::new();
    for (i, line) in lines.enumerate() {
        let rec: J = serde_json::from_str(&line.unwrap()).unwrap();
        let now = instant(rec["t"].as_f64().unwrap());
        let ap = ap.get_or_insert_with(|| Autopilot::new(now));
        let step = &rec["step"];

        let commands: Vec<ControlCommand> = step["sets"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|s| ControlCommand::SetValue {
                name: s[0].as_str().unwrap().try_into().unwrap(),
                value: s[1].to_string().as_str().try_into().unwrap(),
            })
            .collect();

        let nav: Vec<NavReading> = step["nav"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|n| {
                let data = &n["data"];
                let message = match n["sensor"].as_str().unwrap() {
                    "wind" => NavMessage::Wind(Wind {
                        direction: f(&data["direction"]),
                        speed: f(&data["speed"]),
                    }),
                    "gps" => NavMessage::Gps(GpsFix {
                        track: data.get("track").map(f),
                        speed: f(&data["speed"]),
                        ..Default::default()
                    }),
                    "apb" => NavMessage::Route(RouteSteer {
                        track: f(&data["track"]),
                        xte: f(&data["xte"]),
                        magnetic: data["mode"] != "gps",
                    }),
                    other => panic!("sensor {other}"),
                };
                NavReading {
                    source: source(n["source"].as_str().unwrap()),
                    device: DeviceId::try_from(n["device"].as_str().unwrap()).unwrap(),
                    time: now,
                    message,
                }
            })
            .collect();

        let telemetry = step.get("telemetry").map(|t| {
            let opt = |k: &str| t.get(k).map(|v| if v.is_null() { f32::NAN } else { f(v) });
            MotorTelemetry {
                flags: Some(t["flags"].as_u64().unwrap() as u16),
                current: opt("current"),
                voltage: opt("voltage"),
                controller_temp: opt("controller_temp"),
                motor_temp: opt("motor_temp"),
                rudder_raw: opt("rudder"),
            }
        });

        let imu = step.get("imu").map(|d| ImuData {
            heading: f(&d["heading"]),
            heading_lowpass: f(&d["heading_lowpass"]),
            headingrate_lowpass: f(&d["headingrate_lowpass"]),
            headingraterate_lowpass: f(&d["headingraterate_lowpass"]),
            heel: f(&d["heel"]),
            rollrate: f(&d["rollrate"]),
            pitchrate: f(&d["pitchrate"]),
            heading_offset: f(&d["heading_offset"]),
            compass_calibration_updated: false,
        });

        let out = ap.step(
            now,
            &Inputs {
                imu,
                nav: &nav,
                motor: MotorInputs {
                    connected: true,
                    telemetry,
                },
                commands: &commands,
            },
        );

        // the motor: the last command or disengage pypilot sent, and resets
        let calls = rec["calls"].as_array().unwrap();
        let py_reset = calls.iter().any(|c| c[0] == "reset");
        let py_cmd = calls
            .iter()
            .rev()
            .find_map(|c| match c[0].as_str().unwrap() {
                "command" => Some(MotorCommand::Speed(f(&c[1]))),
                "disengage" => Some(MotorCommand::Disengage),
                _ => None,
            });
        let cmd_ok = match (py_cmd, out.servo.command) {
            (Some(MotorCommand::Speed(a)), Some(MotorCommand::Speed(b))) => (a - b).abs() < 1e-3,
            (a, b) => a == b,
        };
        if !cmd_ok || py_reset != out.servo.reset {
            mismatches.push(format!(
                "step {i}: motor pypilot {py_cmd:?} reset {py_reset}, rustpilot {:?} reset {}",
                out.servo.command, out.servo.reset
            ));
        }

        for (key, py) in rec["state"].as_object().unwrap() {
            // in standby pypilot only computes gain terms when a client
            // watches them; RustPilot always does
            if key.ends_with("gain") && rec["state"]["ap.enabled"] == J::Bool(false) {
                continue;
            }
            let rs = if key == "servo.flags" {
                Value::Number(ap.servo.flags as f32)
            } else {
                values::get(ap, key).unwrap_or_else(|| panic!("no value {key}"))
            };
            if !matches(py, rs) {
                mismatches.push(format!("step {i}: {key} pypilot {py} rustpilot {rs:?}"));
            }
        }
        if mismatches.len() > 20 {
            break;
        }
    }
    assert!(
        mismatches.is_empty(),
        "{name}: {} mismatches, first:\n{}",
        mismatches.len(),
        mismatches.join("\n")
    );
}

#[test]
fn compass() {
    replay("compass");
}

#[test]
fn wind() {
    replay("wind");
}

#[test]
fn gps() {
    replay("gps");
}

#[test]
fn servo() {
    replay("servo");
}
