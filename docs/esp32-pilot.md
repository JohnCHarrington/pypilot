# An ESP32 pypilot: design study

Status: **design study**. Nothing here is built.

What an ESP32 version of pypilot would look like under these constraints:

- **N2K only.** NMEA 2000 is the only data and control interface: no Wi-Fi, Signal K, NMEA 0183 or web UI.
- **Same control schema.** Control uses the standard PGNs in [n2k-control.md](n2k-control.md): 127237 commands, the switch bank, `PP:` text commands and alerts.
- **Same motor controllers.** The existing pypilot motor controller boards (`arduino/motor/motor.ino`) are reused unchanged.
- **C++ pilot.** The autopilot itself is reimplemented in C++.

The study ends with a comparison against today's Raspberry Pi pypilot (tinypilot or a full Pi) on performance and power.

## 1. Summary

The ESP32 version is a **single small N2K device**. It holds the IMU, the autopilot loop and the N2K interface, and drives the existing motor controller over its serial link. It would:

- **Boot in under a second.** A Pi takes tens of seconds.
- **Run its control loop with microsecond-level timing.** Python on Linux has millisecond jitter and occasional stalls.
- **Draw roughly a third to a quarter of a Pi Zero's power for the electronics.** That's about 0.3–0.5 W against 1–1.5 W. It could also be powered from the N2K bus at a low load rating.

None of that changes total system power much at sea, where the drive motor dominates.

**What it gives up:** pypilot's flexibility. There would be no Python pilots to edit on the boat, no machine-learning pilots, no web UI, no Signal K or 0183 inputs, and no live plotting. Configuration would be through `PP:` text commands only. It would also be a substantial rewrite: about 6–10 thousand lines of C++. Parts can be reused, though:
- pypilot's motor controller driver (`pypilot/arduino_servo/`) is already C++;
- its IMU library (RTIMULib2) is C++;
- an ESP32 NMEA 2000 library with group-function support already exists.

## 2. Scope

| Kept (same behaviour and value names as pypilot) | Dropped or deferred |
|---|---|
| Modes: compass, gps, nav, wind, true wind, with the same availability and fallback rules | Signal K, NMEA 0183, gpsd, the TCP client protocol and the web UI |
| Pilots: basic, simple, absolute, rate, wind, gps | The learning and intellect pilots (TensorFlow), and fuzzy (could be ported later) |
| Tacking, with the same states, delay, rate and threshold | The GPS Kalman filter (`gps_filter.py`, numpy); could come later |
| Profiles and profiled settings | Live plots and long logs |
| IMU fusion, level alignment, heading offset, automatic compass calibration with lock | Servo auto-calibration (`servo_calibration.py`, numpy); could come later |
| Motor controller: commands, limits, faults, telemetry, rudder feedback, rudder calibration | Hat features (LCD, IR/RF remotes): the device has no display |
| All of n2k-control.md: paths A, B and C, status and alerts | |
| Today's N2K outputs (127250/127251/127257/127245) and inputs (GPS, wind, speed, nav, rudder) | |

Value names stay identical (`ap.mode`, `ap.pilot.basic.P`, `servo.max_current`…). That means `PP:` commands, documentation and user habits carry over, and the bench's control tests can drive either implementation (§8).

## 3. Hardware

```
          N2K backbone (12 V, CAN 250 kbit/s)
                │
   ┌────────────┴─────────────┐
   │ isolated CAN transceiver │   bus-powered side: about 1 LEN (§7)
   ├──────────────────────────┤
   │ ESP32-S3                 │
   │  ├─ SPI/I²C: ICM-20948    │   9-axis IMU on the same board
   │  ├─ UART2: motor ctrl     │   38400 baud, through optocouplers
   │  ├─ GPIO: STANDBY button  │   optional, local, always works
   │  └─ GPIO: status LED      │
   └────────────┬─────────────┘
                │ opto-isolated serial
   ┌────────────┴─────────────┐
   │ pypilot motor controller │   unchanged ATmega328 board, 5 V logic,
   │ (motor.ino)              │   powered from the boat's 12/24 V supply
   └────────────┬─────────────┘
              motor / clutch / rudder sensor
```

- **MCU: ESP32-S3.** It has two 240 MHz cores, a single-precision FPU, TWAI (the CAN controller) and USB for flashing and debugging.
  - **Avoid the ESP32-C3 and C6.** Their RISC-V cores have no FPU, and IMU fusion and the pilot maths are all floating point.
  - **The classic ESP32 also works**, but check its TWAI errata against the chip revision.
- **CAN: an isolated transceiver** (e.g. ISO1042, or a transceiver behind an isolated DC-DC converter). The board is also connected to the motor controller, which runs from the boat's main supply. The N2K standard expects isolation on devices powered from anywhere other than the bus. Isolating the bus side also avoids ground loops through the motor return current.
- **IMU: ICM-20948** (the MPU-9250's successor), mounted on the same board. pypilot today supports the MPU-925x family through RTIMULib2. Its alignment quaternion (`imu.alignmentQ`) and heading offset carry over.
- **Motor controller link.** The board's UART runs at 38400 baud with 4-byte packets: a code, a 16-bit value and a CRC-8 (see `arduino_servo.cpp`). Its logic is 5 V, so the 3.3 V ESP32 needs level shifting. The motor controller README already strongly recommends optical isolation, which provides both.
- **A local STANDBY button** (optional, recommended). With no Wi-Fi and no display, a wired button means disengaging never depends on the bus.

## 4. Firmware architecture

Built on ESP-IDF and FreeRTOS. The control core is plain C++ with no IDF dependencies, so it can also be built and tested on Linux (§8).

| Task | Core | Rate | Job |
|---|---|---|---|
| `imu` | 1 | 100 Hz | Read the ICM-20948, run fusion, produce heading, heading rate and heading rate-of-rate (low-passed as `boatimu.py` does), and collect compass calibration points |
| `autopilot` | 1 | 20 Hz (up to 50) | Mode logic and availability, heading error, pilot, tacking, servo command; the same order as `Autopilot.iteration` |
| `servo` | 1 | each autopilot cycle | Send commands and settings to the motor controller, parse telemetry, apply fault and timeout handling (port of `servo.py` plus `arduino_servo.cpp`) |
| `n2k` | 0 | event driven | TWAI driver, address claim, product and configuration info, sensor inputs, status outputs, n2k-control (paths A/B/C, alerts) |
| `values` | 0 | on change | Value registry, profiles, saving to flash (NVS) |
| `calibrate` | 0 | low priority | Compass ellipsoid fit, run every few minutes as pypilot does |

Tasks exchange data through lock-free single-writer snapshots: the IMU publishes a state struct, the autopilot reads the latest. The control loop never waits on N2K or flash.

### 4.1 What can be reused

- **Motor controller driver.** `pypilot/arduino_servo/arduino_servo.cpp` and `arduino_servo_eeprom.cpp` are already C++ (Python uses them through SWIG). Swapping the POSIX `read`/`write` for the IDF UART driver is a small change. The fault and command-timeout logic in `servo.py` needs porting by hand.
- **IMU fusion.** RTIMULib2's `RTFusionRTQF` (the fusion pypilot uses, with `setSlerpPower(.01)`) is portable C++. Alternatively a Mahony or Madgwick filter can be tuned to match. Porting RTQF keeps behaviour closest to pypilot's.
- **NMEA 2000 stack.** Timo Lappalainen's `NMEA2000` library with its ESP32 CAN driver is mature C++. It covers address claim, product information, fast packet, ISO requests, parsers for the needed PGNs, and a **group-function handler framework** (one handler per PGN, for 126208 Request/Command). Paths A and C map onto that framework directly. That is the equivalent of the hooks we had to add to the Python library. It's designed for the Arduino core, but ESP-IDF ports exist. Settle on one early.
- **Control logic.** `pypilot/n2k_control.py` is about 1,000 lines of plain logic with no Python-specific dependencies. It ports almost line for line, and its 90+ unit tests convert into C++ tests.

### 4.2 What needs a careful rewrite

- **Pilots.** The basic pilot is a few lines: gains times error, rate, rate-of-rate, square root of error, and feed-forward (`pilots/basic.py`). Simple, absolute, rate, wind and gps are similar. Each becomes a C++ class with a gain table, and gains keep their names (`ap.pilot.basic.P` and so on).
- **The value registry.** This is a C++ table of typed values (range, enum, boolean, sensor) with the same names, flags (persistent, profiled, writable) and ranges as the Python `values.py` registrations. It's the backbone of `PP:` commands and profiles. Write it once, carefully. Persistent values are saved to NVS in batches with a delay, as pypilot's config file is today, to limit flash wear.
- **Compass calibration.** `calibration_fit.py` uses scipy (ODR) to fit a sphere, then an ellipsoid, to compass points spread evenly over the sphere, and it tracks the age and quality of the fit. A Levenberg–Marquardt fit in float over the same ~100–200 points runs in milliseconds on an S3. Matching pypilot's acceptance rules and point selection is the real work. This is the **highest-risk port**: a bad calibration means bad heading.
- **Floating point.** The S3's FPU is single precision only, and doubles run in software, many times slower. Fusion and the pilot are fine in float. Latitude and longitude need double, or local coordinates, but only at N2K input rates, so the cost is negligible.

## 5. N2K interface

Unchanged from the current Python design, so a display can't tell the two apart.

- **Inputs:** 129029/129026/129033 (GPS), 130306 (wind), 128259 (speed), 129283/129284 (nav), and 127245 when the rudder angle comes from N2K.
- **Outputs:** 127250/127251/127257 (heading, rate of turn, attitude), 127245 (rudder), 127237, 127501, and alerts 126983/126985.
- **Control:** all of [n2k-control.md](n2k-control.md), with the same channel map, control levels, text command syntax and refused names.
- **Configuration without Wi-Fi.** `PP:` commands are the only configuration interface. That's workable for commissioning but clumsy for tuning gains. If that proves too painful, the proprietary "value channel" sketched in n2k-control.md §9 becomes worth building, as would a small PC or phone tool that talks to it through a USB–N2K gateway.
- **Firmware updates.** USB on the device. Updating over N2K is possible (ISO transport plus a bootloader) but a project in itself. A time-limited Wi-Fi update mode switched on by a `PP:` command is the pragmatic middle ground, if "N2K only" can bend that far.

## 6. Performance compared with pypilot on a Pi

| | pypilot (Pi Zero 2 W / tinypilot, or Pi 3/4) | ESP32-S3 |
|---|---|---|
| Boot to steering | Tens of seconds (Linux, then the Python processes, then servo probe) | Under 1 s |
| IMU fusion | RTIMULib2 in C++ in its own process, sampling at 100 Hz | Same algorithm, 100–200 Hz |
| Autopilot loop | 10 or 20 Hz (`imu.rate`), Python, separate processes linked by pipes | 20 Hz to match; 50 Hz is possible |
| Timing jitter | Milliseconds, with occasional longer stalls (garbage collection, I/O, SD card). pypilot runs `chrt` realtime priority and logs when a cycle runs late | Microseconds; fixed-priority tasks, no garbage collector, no filesystem in the loop |
| CPU headroom | Plenty for basic pilots; machine-learning pilots need a Pi 3/4 | Basic pilots and fusion use a small fraction of one core |
| Startup after a power blip | Long, and an SD card can corrupt on unclean shutdown | Immediate; NVS is designed to survive power loss |
| Compass calibration | scipy, robust and proven | Must be re-implemented and re-validated (§4.2) |

The loop rate alone won't make a pypilot steer better. Steering quality comes from the pilot, the gains and the drive, and pypilot at 10–20 Hz is already adequate for boat dynamics. The practical gains are **determinism** (no missed or late cycles) and **instant recovery after a power interruption**, which matters for a safety-relevant device. A faster loop mainly gives cleaner rate-of-rate (DD) terms.

## 7. Power

The figures below are typical values from datasheets and community measurements, not measurements of this design. They're good enough for orders of magnitude. **Measure on real hardware before relying on them.**

| Electronics only (not the drive) | Pi Zero 2 W / tinypilot | ESP32-S3, N2K only |
|---|---|---|
| Processor | about 0.6–1.0 W (idle to light load, Wi-Fi on) | about 0.15–0.25 W (both cores at 240 MHz, radios off) |
| IMU | about 0.01–0.03 W | same |
| CAN interface | USB or SPI CAN adapter: about 0.1–0.3 W | isolated transceiver: about 0.05–0.15 W |
| Display and remote receivers (tinypilot hat) | about 0.1–0.5 W | none |
| Regulator losses | 10–20 % | 10–20 % |
| **Total** | **about 1–1.5 W** (a full Pi 3/4: 2–4 W) | **about 0.3–0.5 W** |
| As an N2K load | Usually powered separately; from the bus it would be about 2–3 LEN | About 1 LEN (50 mA at 12 V is 0.6 W), so it can be bus-powered |

**In context:** while steering, the drive motor and clutch draw from about one to several amps at 12 V, depending on the drive, sea state and gains. That's roughly 10–60 W averaged. The electronics difference of about 1 W is therefore **a few percent at sea**. It matters:
- **in standby**, at the dock or at anchor, when the autopilot is often left powered;
- **on small or solar-powered boats**;
- because a device on **1 LEN can simply be bus-powered**, which makes installation simpler.

Turning off the Pi's Wi-Fi, or using a full Pi with a display, moves the Pi's numbers but not the conclusion.

## 8. Testing with the existing bench

The testbench (`pypilot-testbench`) drives pypilot almost entirely over N2K. It provides simulated instruments, the control head and the motor controller emulator, so most of it applies unchanged:

1. **Host build.** Build the firmware's control core for Linux with small platform shims:
   - SocketCAN instead of TWAI;
   - a pty for the motor controller UART;
   - the bench's UDP IMU feed instead of the ICM-20948.

   Run that as a bench container in place of pypilot.
2. **Same tests.** `tests/test_n2k_control.py` already talks to pypilot over N2K through the control head. The tests that read pypilot values over its TCP port would switch to `PP:` reads. The steering tests compare against the simulator's true heading, so they port as they are.
3. **Hardware in the loop.** For the real board:
   - bridge the bench's `vcan0` to a USB–CAN adapter (`cangw`);
   - connect the motor controller UART to the servo emulator through a USB serial adapter;
   - put the board on a turntable, or feed the IMU from the bench if the firmware has a simulated-IMU input.
4. **Side by side.** Running both implementations against the same scenarios (`measure_steering.py`) gives a direct steering comparison before any sea trial.

## 9. Effort and risks

**Size.** Roughly 6–10 thousand lines of C++, depending on how much of RTIMULib2 and the NMEA2000 library is reused rather than rewritten:

| Area | Lines | Notes |
|---|---|---|
| Value registry, profiles, persistence | 1,000–1,500 | |
| IMU, fusion, alignment and calibration | 1,500–2,500 | Calibration is the risk |
| Autopilot, modes, tacking, pilots | 1,000–1,500 | |
| Servo | 800–1,200 | Mostly existing C++ |
| N2K inputs, outputs and n2k-control | 1,500–2,500 | On top of the library |
| Host build and test harness | 500–1,000 | |

**Main risks**
- **Compass calibration quality.** Port it first and validate it against pypilot's on recorded data.
- **Behaviour drift.** Small differences in filters and low-pass constants change how the boat steers. Comparing the two implementations side by side on the bench is the mitigation.
- **Configuration usability without Wi-Fi.** `PP:` commands are tolerable for commissioning, painful for tuning. Expect to want the value channel or an update/config mode.
- **Two codebases.** Features added to Python pypilot won't appear on the ESP32 by themselves. Sharing the value names and the N2K schema limits the drift. Sharing test suites through the bench is what keeps the two honest.

## 10. Suggested path

1. **Host-build skeleton** on the bench:
   - value registry;
   - n2k-control ported from Python, with its unit tests;
   - motor controller driver (reuse `arduino_servo.cpp`);
   - basic pilot in compass mode.

   Pass the bench's switch-bank, 127237 and steering tests.
2. **IMU on hardware:** ICM-20948 plus RTQF fusion, compared against pypilot on the same board.
3. **Compass calibration**, validated against pypilot's results on recorded data.
4. **Remaining modes and pilots, tacking, and alerts.** Then the full bench suite, then hardware in the loop.
5. **Board design:**
   - isolated CAN interface;
   - opto-isolated motor controller link;
   - local standby button;
   - bus power at 1 LEN.
