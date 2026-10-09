# RustPilot

A Rust rewrite of pypilot: one portable autopilot core that builds for
Linux, Raspberry Pi and ESP32-S3, with every hardware or network link behind
a trait each platform implements. Existing pypilot clients (web UI, HAT,
OpenCPN plugin) keep working through a pypilot-compatible JSON server, and
the Arduino motor controller is used unchanged over its serial protocol.

Licensed GPL-3.0-or-later, like pypilot.

## Layout

| Crate | `no_std` | What it holds |
|---|---|---|
| `rustpilot-types` | yes | Angles with pypilot's `resolv`, vectors, quaternions, time, sensor readings |
| `rustpilot-hal` | yes | Connector traits: `NavSource`, `NavSink`, `ControlLink`, `ImuSource`, `MotorController`, `RudderSensor`, `CanBus` |
| `rustpilot-core` | yes | The autopilot: `Autopilot::step(now, inputs) -> outputs`, no I/O |
| `rustpilot-n2k` | no (canboat needs std) | NMEA 2000 on the `canboat` crate; 126208 group functions |

`tools/golden/generate.py` runs pypilot's own control loop on scripted
scenarios and writes the golden files that `rustpilot-core/tests/golden.rs`
replays; rerun it with `python3 tools/golden/generate.py --pypilot ..` after
changing a scenario. `tools/record_pypilot.py` records a running pypilot's IMU, sensor, autopilot
and servo values for replay tests.

## Building

```sh
cargo test --workspace
# the portable crates must also build with no std:
rustup target add thumbv7em-none-eabihf
cargo build --target thumbv7em-none-eabihf -p rustpilot-types -p rustpilot-hal -p rustpilot-core
```

CI runs both, plus `cargo fmt --check` and `cargo clippy -D warnings`.

## Status

Phase 0 (foundations) is done; see `docs/phase0-findings.md`. Phase 1 ports
the core: sensor arbitration, wind and GPS filters, APB route steering, the
basic pilot, tacking, rudder calibration, the servo logic and pypilot's value
names. See `docs/phase1.md`.
