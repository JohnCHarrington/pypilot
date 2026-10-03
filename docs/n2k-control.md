# pypilot control over NMEA 2000

Status: **pypilot side implemented** in `pypilot/n2k_control.py`, wired into
`pypilot/n2k.py`, with tests in `tests/test_n2k_control.py` and
`tests/test_n2k_bridge_control.py`. Path A and path C also need hooks in the
`nmea2000` library (§8), which don't exist yet. Until they do, the bridge logs a
warning and those two paths are inactive. Path B, the status PGNs and the
alerts work with the current library.

## 1. Summary

pypilot is operated and configured over NMEA 2000 using **only standard PGNs**.
No proprietary PGNs are needed, so this design doesn't depend on pypilot having
an NMEA manufacturer code. It also works with off-the-shelf displays, keypads
and config tools.

There are three control paths:

| Path | Standard PGNs | Used for | Typical sender |
|---|---|---|---|
| **A. Heading/Track Control** | 127237, written using 126208 commands | Engage/standby, set heading, track mode, follow-up rudder | MFDs and nav software that drive autopilots by the standard |
| **B. Digital switching** | 127502 in, 127501 out | Standby, all five modes, ±1°/±10°, tack, jog, dismiss alarm, next profile | MFD digital-switching pages, N2K keypads, CZone-capable displays |
| **C. Text commands** | 126998 installation description, written using 126208 | Reading and writing **any** pypilot value (gains, profiles, calibration, servo settings…) | Actisense NMEA Reader, Yacht Devices gateways, MFD device-config pages |

Status goes out on 127237, 127245, 127250/127251/127257 and 127501, with faults
on the alert PGNs 126983/126985. Nav mode keeps taking 129283/129284 from the
chartplotter, as it does today.

There's commercial precedent for this design. The Yacht Devices YDAP-04 N2K
autopilot is controlled the same way:
- switch-bank channels for standby, modes, ±1/±10 and tack;
- `YD:` text commands written into its installation description;
- nav data from the plotter.

### Coverage

| Function | Path |
|---|---|
| Engage / standby | A, B |
| Mode: compass, gps, nav | A, B |
| Mode: wind, true wind | B |
| Set an absolute heading (compass, gps, nav) | A |
| ±1° / ±10° adjust, in every mode | B |
| Tack (auto, port, starboard; cancel) | B |
| Manual steering: pulse jog | A (non-follow-up), B |
| Manual steering: rudder angle (needs rudder sensor) | A (follow-up) |
| Choose profile, select mode without engaging | B (next profile, next mode), C |
| Pilot, gains, servo/motor settings, rudder calibration, level, heading offset, compass cal lock, reset counters, NMEA/Signal K/GPS settings | C |
| Faults and warnings shown on MFDs, acknowledged from MFDs | Alerts |

### Not covered

These are either not possible with standard PGNs, or deliberately excluded.

- **Setting an absolute wind angle.** 127237 has no wind steering mode. In wind
  modes the angle can only be changed in ±1°/±10° steps (path B).
- **A settings screen in an app or on a display.** Path C is a line-at-a-time
  command interface. It can't stream value updates or list everything in one
  response, so it suits commissioning, not a settings UI.
- **Servo telemetry** (voltage, current, temperatures, amp-hours). There is no
  honest standard PGN for it. It's readable with path C (`PP:servo.voltage`).
- **Setting up N2K itself**, such as the transport, interface and control level.
  Changing these over the bus could cut the link mid-session (§6).
- **Anything that isn't a pypilot server value.** This covers the hat's Wi-Fi,
  LCD and remote settings, OS updates, logs, and servo controller firmware.

A later proprietary extension (§9) could add a settings UI and telemetry. It
isn't needed for operating the autopilot.

## 2. Conventions

- **Turn direction: +ve means starboard (clockwise).** The bridge converts this
  for pypilot's internal conventions:
  - Wind modes invert the sign of heading adjustments (`hat/page.py:836`).
  - `servo.command` is negative for starboard.
- **Mode names:** `compass`, `gps`, `nav`, `wind`, `true wind`. These are
  pypilot's `ap.mode` values.
- **Mode availability:** a mode can only be selected if it's in `ap.modes`.
  pypilot drops modes whose sensors are missing.
- **"Engage in mode M"** means:
  1. Set `ap.mode = M`.
  2. If not already engaged, set `ap.heading_command = ap.heading`.
  3. Set `ap.enabled = True`.

  This is the same sequence the hat uses. **Standby** means
  `ap.enabled = False` followed by `servo.command = 0`.
- **Jog pulse:** set `servo.command = ±1`, then send 0 after
  `n2k.control.jog_pulse` seconds (default 0.3). The existing 1 s auto-zero in
  `servo.py:346` is the backstop. Jog is ignored while engaged.

## 3. Path A — 127237 Heading/Track Control

### 3.1 Transmit

127237 is sent at 1 Hz, immediately on any change to engaged state, mode or
heading command, and when a 126208 or ISO request asks for it. Changes are
rate-limited to 5 Hz.

| Field (# in 126208 numbering) | Value |
|---|---|
| 1 rudderLimitExceeded | Yes if `servo.flags` has MIN_RUDDER or MAX_RUDDER fault |
| 2 offHeadingLimitExceeded | Not available (pypilot has no off-course limit) |
| 3 offTrackLimitExceeded | Not available |
| 4 override | Yes while a manual `servo.command` or jog is active |
| 5 steeringMode | Standby: 0 *Main Steering*. Compass, gps, wind and true wind: 4 *Heading Control*. Nav: 5 *Track Control*. |
| 6 turnMode | Not available |
| 7 headingReference | Magnetic for compass and wind modes; True for gps and nav |
| 9 commandedRudderDirection | From the sign of the active servo command (No Order when idle) |
| 10 commandedRudderAngle | `servo.position_command`, if there's a rudder sensor |
| 11 headingToSteerCourse | Compass/gps/nav: `ap.heading_command`. Wind modes: `imu.heading_lowpass − ap.heading_error` (the equivalent compass heading). |
| 12 track | Nav mode: `apb.track` |
| 13 rudderLimit | `rudder.range` |
| 18 vesselHeading | `imu.heading_lowpass` |

### 3.2 Receive: 126208 Command → 127237

Commands are accepted only at control level `steer` or above (§6). Every
command gets a **126208 Acknowledge** reply with a per-parameter error code.

| Command contents | Action |
|---|---|
| steeringMode = 0 | Standby. Always accepted unless control is `off`. |
| steeringMode = 3 or 4, no field 7 | Engage, keeping the current mode. If the current mode is nav, use compass instead. |
| steeringMode = 3 or 4, field 7 = Magnetic | Engage in compass mode |
| steeringMode = 3 or 4, field 7 = True | NAK *not supported* on field 7. pypilot has no true-heading mode, and gps mode steers course over ground, which is not the same thing. Status (§3.1) still reports True for gps and nav modes, because those courses are true. |
| steeringMode = 5 | Engage in nav mode. NAK *temporarily unavailable* if nav is not in `ap.modes`. |
| field 11 headingToSteerCourse | Set `ap.heading_command`. Applied after any mode change in the same command. NAK in wind modes, because we must not silently reinterpret a heading as a wind angle. |
| steeringMode = 1 *Non-Follow-Up*, with field 9 | Standby first if engaged. Then: *Move to starboard/port* gives one jog pulse; *No Order* stops. |
| steeringMode = 2 *Follow-Up*, with field 10 | Standby first if engaged, then `servo.position_command = angle`. NAK if there's no rudder sensor. |
| Fields 6, 13–17 (turn mode, limits, turn rate, radius) | NAK *cannot set*. pypilot has no such features yet. |

Commands that set several parameters arrive in one 126208 message. They must
be applied in one go, in this order: steeringMode, headingReference, then
headingToSteerCourse.

## 4. Path B — digital switching (127502 / 127501)

pypilot appears as one **binary switch bank** with instance
`n2k.switch.bank`. The default is `off`, and the installer must pick an
instance. It must be unique on the bus: two switching devices with the same
instance will fight.

### 4.1 Channel map

| Ch | Name | On receiving ON | 127501 indicator |
|---|---|---|---|
| 1 | STANDBY | Standby | ON while disengaged |
| 2 | COMPASS | Engage in compass mode | ON while engaged in this mode |
| 3 | GPS | Engage in gps mode | (same) |
| 4 | NAV | Engage in nav mode | (same) |
| 5 | WIND | Engage in wind mode | (same) |
| 6 | TRUE WIND | Engage in true wind mode | (same) |
| 7 | −1 | Heading command 1° to port | Momentary echo |
| 8 | +1 | Heading command 1° to starboard | Momentary echo |
| 9 | −10 | Heading command 10° to port | Momentary echo |
| 10 | +10 | Heading command 10° to starboard | Momentary echo |
| 11 | TACK | Tack in the direction pypilot detects (`ap.tack.direction`); cancels if a tack is waiting | ON while tack waiting or in progress |
| 12 | TACK PORT | Tack to port; cancels if a tack is waiting | (same as 11) |
| 13 | TACK STBD | Tack to starboard; cancels if a tack is waiting | (same as 11) |
| 14 | JOG PORT | One jog pulse to port (standby only) | Momentary echo |
| 15 | JOG STBD | One jog pulse to starboard (standby only) | Momentary echo |
| 16 | DISMISS ALARM | Acknowledge all active pypilot alerts | ON while any alert is unacknowledged |
| 17 | NEXT PROFILE | Switch to the next entry of `profiles` | Momentary echo |
| 18 | NEXT MODE | Select the next mode in `ap.modes` **without** engaging (the hat's mode key). If engaged, pypilot keeps the same course in the new mode | Momentary echo |
| 19–20 | *(reserved)* | Ignored | Off |
| 21 | — | *(status only)* | ON while a servo fault is active |
| 22 | — | *(status only)* | ON while there's an IMU/compass error |
| 23 | — | *(status only)* | ON while the active mode ≠ preferred mode (sensor-loss fallback) |
| 24–28 | *(reserved)* | Ignored | Off |

### 4.2 Switch semantics

- **Edge-triggered.** An action fires when a channel goes from not-ON to ON.
  Controllers that resend ON without an OFF in between don't repeat the action.
  Each channel re-arms on either an explicit OFF or the 1 s auto-reset below.
  This copes with momentary buttons (which send ON then OFF) and latching
  toggles (which send ON and nothing else).
- **OFF never starts an action.** In particular, turning off a mode channel
  does **not** disengage. A momentary button's OFF arrives milliseconds after
  its ON, so treating OFF as standby would cancel every press. Use STANDBY.
- **Momentary channels** (7–10, 14, 15, 17, 18) show ON in 127501 for 0.5 s after
  acting, then OFF.
- **Rejected presses** have no ack in 127502. When a press is rejected (mode
  not available, adjust while tacking, jog while engaged, control level too
  low), its indicator simply doesn't change, and a *Caution* alert with the
  reason is raised for 5 s.
- **127501** is sent at 1 Hz (the usual digital-switching rate), immediately on
  any change, and when requested.
- **Unlisted channels** are left at "not available" (`0b11`) in 127501, and
  ignored in 127502.

## 5. Path C — text commands in 126998 installation description

126998 Configuration Information has three text fields. Config tools can write
the first two with a 126208 Command (parameter 1 or 2).

- **Installation Description 1** is the input field. Text starting with `PP:`
  is run as a command. pypilot then puts back the installer's real description,
  which is persisted in `n2k.installation_description`. Any other text is
  stored as that description, so a write without the prefix behaves exactly as
  the standard intends.
- **Installation Description 2** is the output field. It holds the result of
  the last command and is read-only: writes are NAKed.
- After every command, pypilot **broadcasts 126998** so tools refresh their
  view without having to request it.

### 5.1 Syntax

Each field holds about 70 characters, so commands and results are short.

| Command | Meaning | Result in Description 2 |
|---|---|---|
| `PP:<name>` | Read a value | `<name>=<json>`, truncated with `...` if too long |
| `PP:<name>@<offset>` | Read part of a long value, starting at character `offset` of its compact JSON | `<name>@<offset>/<total>=<chunk>`. Read again from `offset + len(chunk)` until reaching `total` |
| `PP:<name>=<json>` | Write a value (same syntax as pypilot's TCP protocol) | `OK <name>=<json>` with the value actually applied (after clamping or rounding), or `ERR <reason>` |
| `PP:INFO <name>` | Type, range and choices | e.g. `RangeProperty 0..0.03 persistent profiled` |
| `PP:LIST <prefix> [n]` | Value names starting with prefix, page `n`. The prefix is shown once, then the rest of each name. | `ap.pilot.basic: .D .DD .FF .P .PR`, or with more pages: `… (+2)` |
| `PP:HELP` | Command summary | |

Examples:
- `PP:ap.pilot=basic`
- `PP:ap.pilot.basic.P=0.004`
- `PP:profile="heavy"`
- `PP:imu.alignmentCounter=100` (level the IMU)
- `PP:rudder.calibration_state="centered"`
- `PP:servo.faults=0`

Writes need control level `full`. Reads and `LIST`/`INFO` need `monitor`.

**Request tags.** Prefix a command with a tag of 1–8 letters or digits, as
`PP#<tag>:<command>`, and the result starts with `#<tag> `. For example,
`PP#12:ap.mode` gives `#12 ap.mode="compass"`. Results are broadcast in one
shared field, so a tool that might share the bus with another should tag its
commands and ignore results that aren't its own. Only one command is in
progress at a time; a new command replaces one still waiting for its value.

### 5.2 Timing

- **Reads are answered as soon as the value is known.** pypilot answers
  immediately, inside the command handler, when it already has a live value,
  meaning one its N2K process watches anyway. Otherwise it starts watching the
  value and answers when pypilot sends it.
- **Values read once stay watched for 30 s.** Every read renews this, so a
  control head polling a few values gets immediate answers after the first
  read. At most 32 values are kept this way; the oldest is dropped first.
- **Writes are answered when pypilot reports the new value.** If no update
  matching the requested value arrives within 0.5 s (because it was clamped,
  rounded, or was already the current value), the result reports whatever the
  value is then.
- **Reads that get no value within 2 s** report `ERR unknown <name>`.

## 6. Access control and safety

N2K has no authentication: anything on the bus can send these PGNs. Control is
therefore opt-in, through new persistent settings:

| Setting | Values | Default | Notes |
|---|---|---|---|
| `n2k.control` | `off` / `monitor` / `steer` / `full` | `monitor` | **monitor:** status, alerts and path C reads only, **but standby is always honoured**, like a STBY key on any head. **steer:** adds paths A and B. **full:** adds path C writes. |
| `n2k.switch.bank` | `off` or 0–252 | `off` | Switch-bank instance for path B |
| `n2k.control.allowed` | List of 64-bit ISO NAMEs | empty (= any) | Restricts control to known devices. This prevents mistakes, not attacks: NAMEs can be spoofed. |
| `n2k.control.jog_pulse` | 0.1–1.0 s | 0.3 | Duration of one jog pulse |
| `n2k.installation_description` | text | `""` | Persisted Description 1 |

**Values that path C can't change.** Writes to these are refused:
- `n2k.*`, apart from `n2k.output.*` and `n2k.installation_description`.
  Changing the others could cut the link mid-session.
- Internal state: `servo.calibration`, `imu.alignmentQ`, `*.calibration.points`.

**Manual steering is pulse-only over N2K.** Switch states are latched, and a
lost OFF must not leave the motor running. Each jog pulse ends on pypilot's own
timer, with the existing 1 s `servo.command` timeout as backstop.

**Losing the commanding device** leaves the engaged state unchanged, which is
how pypilot behaves today.

**Several controllers** are handled by:
- relative adjusts (path B);
- an immediate status broadcast on every change, so all displays stay in sync;
- refusing headings in wind modes (path A).

**Bus load.** New periodic traffic is about 7 frames/s, well under 1% of
250 kbit/s:
- 127237: 3 frames at 1 Hz
- 127501: 1 frame at 1 Hz
- alerts, while active: about 3 frames at 1 Hz

**Receive filter.** When control is not `off`, the bridge adds these PGNs to
`include_pgns` automatically, on top of the user's `n2k.pgn_filters`:
126208, 127502, 126984, and 59904 (ISO request).

**What gates the outputs.** The control level only gates what pypilot
*accepts*. Outputs have their own settings:
- 127237 and the alerts follow the existing `n2k.output.autopilot` setting.
- 127501 is sent whenever `n2k.switch.bank` is set.
- 127245 follows `n2k.output.rudder`. It is only sent when the rudder angle
  comes from a local sensor, so a rudder angle that arrived over N2K is never
  echoed back onto the bus.

All of these are added to `transmit_pgns` so they're advertised in 126464.
Changing any of these settings restarts the transport, because the filter
and transmit lists are fixed when it starts.

## 7. Alerts (126983 / 126985 sent, 126984 received)

Alerts are standard N2K alert messages. Garmin plotters, for example, receive
126983/126985 and send 126984. Fields used:
- alertSystem = 1 (autopilot)
- alertSubSystem = 0
- alertId = the IDs in the table below
- dataSourceNetworkIdName = pypilot's ISO NAME
- 126985 carries the text

| ID | Condition (source) | Type | Category |
|---|---|---|---|
| 1 | Servo overcurrent (`servo.flags` OVERCURRENT / PORT_/STARBOARD_OVERCURRENT) | Alarm | Technical |
| 2 | Servo or motor over-temperature (OVERTEMP) | Warning | Technical |
| 3 | Bad supply voltage (BADVOLTAGE) | Warning | Technical |
| 4 | Servo driver timeout or no controller while engaged (DRIVER_TIMEOUT, `servo.controller == 'none'`) | Alarm | Technical |
| 5 | Rudder at limit (MIN_/MAX_RUDDER_FAULT) | Caution | Technical |
| 6 | IMU / compass error (`imu.error`) | Alarm if engaged, else Warning | Technical |
| 7 | Compass calibration warning (`imu.warning`) | Caution | Technical |
| 8 | Mode fallback: engaged and `ap.mode ≠ ap.preferred_mode` because a sensor was lost (e.g. wind → compass) | Warning | Navigational |
| 9 | Command rejected (path B feedback, auto-clears after 5 s) | Caution | Technical |

- **Behaviour:** an alert is sent on activation, then at 1 Hz while active, and
  once more with state *Normal* when it clears.
- **126984 Acknowledge** marks the alert acknowledged; channel 16 does the same.
- **Temporary Silence** marks it silenced for 30 s. pypilot has no sounder of
  its own except the hat buzzer.
- **Not implemented for now:** 126986–126988 (alert configuration, thresholds
  and values).

## 8. Required changes to the `nmea2000` library (fork)

pypilot parses 126208 payloads itself (`n2k_control.parse_command`), using
the field widths of 127237 and 126998. It also keeps 126998 up to date itself:
it sets `installation_description1/2` on the device and broadcasts 126998.

`N2KBridge.attach_control` uses the hooks below if the library has them. These
are the interfaces it expects:

1. **`N2KDevice.set_group_function_handler(handler)`**
   - Signature: `async handler(message: NMEA2000Message, payload: bytes) -> bool`.
   - Called for every 126208 addressed to this device or to broadcast.
   - `payload` is the complete reassembled fast-packet data.
   - Called **before** the default handling in `_handle_group_function`
     (`device.py:456`). If it returns True the library sends nothing, because
     pypilot has sent its own Acknowledge. If it returns False, the current
     NAK behaviour applies.
2. **`N2KDevice.set_iso_request_handler(handler)`**
   - Signature: `async handler(message: NMEA2000Message, requested_pgn: int) -> bool`.
   - Called from `_handle_iso_request` (`device.py:436`) for PGNs the library
     doesn't answer itself.
   - True means the request was answered; False means NAK as now.
3. **`N2KDevice.own_name`.** A public property for the 64-bit ISO NAME
   pypilot claimed. It's used as the alert data source and to recognise alert
   responses meant for pypilot. pypilot falls back to `_own_name`, then to 0.
4. **Management PGNs must reach the device whatever `include_pgns` says.**
   126208 and 59904 have to get to the device's management handling even when
   the application's include filter would drop them. pypilot adds them to the
   filter when control is on, so this only matters if the filter is applied
   before management handling.

Already supported by the current library:
- encoders and decoders for 127237, 127245, 127501, 127502, 126983, 126984,
  126985 and 126998 (lookup fields need their value in `raw_value`);
- delivering 127502 and 126984 to the receive callback;
- answering ISO requests for 126998 from the device's attributes.

## 9. Later and optional

- **Manufacturer code cleanup.** `n2k.py` claims manufacturer code 999, which
  canboat assigns to **Signal K**. Nothing in this design depends on it, but
  pypilot shouldn't identify itself as another product. The options are:
  - get an official NMEA code;
  - ask Signal K for a delegated sub-range;
  - use an unassigned code, held as a single constant.
- **Proprietary extension.** Only for a richer settings UI (list and watch
  values) and servo telemetry. It needs the manufacturer code above plus a
  raw-payload passthrough in the library. Not planned until there's a client
  that needs it.
- **Raymarine / Navico emulation.** Would make Axiom and B&G/Simrad MFDs show
  native autopilot controls, including absolute wind angle. It's
  reverse-engineered and fragile across firmware versions; YDAP-04 ships it
  only as "experimental". It should be off by default.
- **New pypilot features** that would make more of 127237 meaningful:
  - an off-heading alarm (field 14 and 127237's offHeadingLimitExceeded flag);
  - a rudder limit separate from `rudder.range` (field 13).

## 10. Suggested implementation order

1. ~~pypilot side: all paths, status, alerts~~ (done).
2. Library hooks in the `nmea2000` fork (§8).
3. Bench test against real devices:
   - an MFD sending 127237 commands, if one can be found;
   - a digital-switching display or keypad;
   - Actisense NMEA Reader for `PP:` commands;
   - a Garmin plotter for alerts.

## 11. Decisions

- ~~Default switch-bank instance~~ Decided: `off`. With a fixed default,
  switching gear already using that bank (lights, pumps) would also control
  the autopilot.
- ~~127237 True → gps mode~~ Decided: True heading commands are refused (§3.2).
- ~~Default control level~~ Decided: `monitor`, where standby is always honoured.
