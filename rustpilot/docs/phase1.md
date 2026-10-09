# Phase 1: the core

`rustpilot-core` now ports pypilot's control loop. `Autopilot::step` follows
the order of `Autopilot.iteration` in pypilot/autopilot.py.

| Module | Ports |
|---|---|
| `sensors` | sensors.py: source priority and 8 s timeouts, wind filter and masthead compensation, GPS, water speed, APB (2 Hz, `track + gain * xte`), true wind |
| `heading` | heading error clamp, integral, wind-mode sign |
| `pilots` | pilots/basic.py with its default gains |
| `tacking` | tacking.py |
| `rudder` | rudder.py, including 1, 2 and 3 point calibration |
| `servo` | servo.py: position and speed commands, windup, minimum speed, throttling, telemetry corrections, faults and flags |
| `values` | pypilot's value names, ranges and enums, for the JSON server |

Learning pilots are out of scope for now.

## Parity tests

`tools/golden/generate.py` loads pypilot from a checkout and drives its real
`Autopilot.iteration` with a fake clock, IMU, value client and motor driver.
Four scenarios are recorded:

- `compass`: engage, heading changes, gain changes.
- `wind`: wind and true wind modes, a wind dropout, a tack.
- `gps`: GPS, APB route steering, nav mode, true wind synthesised from GPS.
- `servo`: overcurrent and temperature faults, rudder limits, a position
  command, manual commands.

`tests/golden.rs` replays the inputs and compares the last motor command, the
reset flag and every recorded value within `2e-3 * max(1, |x|)`. Gain sensor
values are skipped in standby, where pypilot does not compute them.

Two details keep the comparison exact: times are 1024 s plus multiples of
1/8 s, so Python's float differences equal RustPilot's integer microseconds,
and inputs are rounded to 6 significant digits. Regenerate after editing a
scenario:

```sh
python3 tools/golden/generate.py --pypilot ..
cargo test -p rustpilot-core --test golden
```

## Still open

- A boat simulator crate for closed-loop runs.
- Replays of real boat logs once some are recorded with
  `tools/record_pypilot.py`.
