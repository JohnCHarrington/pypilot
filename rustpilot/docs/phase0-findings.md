# Phase 0 findings

Phase 0 set up the workspace and checked the questions the plan left for it:
whether canboat works without `std`, whether it handles PGN 126208 group
functions, and which ESP32 runtime that implies.

## canboat needs std

canboat 8.3.0 has no `no_std` support. Its baseline `decode` feature uses
`Vec`, `String` and `std::error::Error` in its public types and pulls in
`indexmap` with std. Building `rustpilot-n2k` for `thumbv7em-none-eabihf`
fails at `indexmap` (`can't find crate for std`).

The `node` feature (address claim, ISO NAME, product information, PGN
lists, heartbeat) needs only `decode`, not the `io` feature's serial,
SocketCAN or libc dependencies, so it should build anywhere `std` exists.

### Size

A release build that only decodes and encodes 126208 is about 3 MB larger
than a hello-world on x86_64 (text +0.95 MB, data +2.1 MB), almost all of it
the embedded database of about 600 PGN definitions. On the 32-bit ESP32 the
pointer-heavy tables should be smaller, but that is inferred, not measured.
Budget for an ESP32-S3 module with 8 MB or more flash and a custom partition
table.

## 126208 group functions

What canboat covers:

- **Decoding Commands.** It picks `nmeaCommandGroupFunction` from the
  function code and decodes each parameter against the target PGN's field
  type and units, so a 127237 heading arrives in degrees and a steering mode
  as a lookup. 126998's STRING_LAU text (`PP:` commands) decodes too.
- **Encoding Acknowledges.** `PgnBuilder` builds `nmeaAcknowledgeGroupFunction`
  with one 4-bit error per parameter through its repeating-set API.
- **One gap.** A value cut short by the end of the payload is zero-padded
  rather than rejected. `rustpilot-n2k` checks each value's bit range
  against the payload length and refuses truncated Commands, matching
  pypilot's `parse_command`.

What it doesn't: its node layer only answers discovery. Nothing dispatches
126208 Commands or ISO requests to the application. `rustpilot-n2k` does
that itself (`group_function::parse_command`, `build_ack`), which is fine
because RustPilot owns its event loop.

So none of the four `nmea2000` library hooks in §8 of
`docs/n2k-control.md` are needed:

| Hook in §8 | In RustPilot |
|---|---|
| 1. group function handler | RustPilot receives every 126208 itself and sends its own Acknowledge |
| 2. ISO request handler | RustPilot receives every 59904 itself; canboat's `device` helpers build the replies |
| 3. `own_name` | RustPilot holds its claimed NAME (canboat `device::Claimer`) |
| 4. management PGNs past the include filter | There is no library-side filter between the bus and RustPilot |

## ESP32 runtime: esp-idf std

The plan's rule was: if canboat needs std, esp-idf std wins over bare-metal
Embassy. It does, so the ESP32 binary will use esp-idf's std runtime with
canboat as-is.

The connector traits are async and don't require `Send`, so they work on
esp-idf's executors as well as Tokio's local task sets.

If flash or RAM turn out too tight on the S3, there are two fallbacks:

- Contribute a `no_std` + `alloc` mode and a PGN subset feature to canboat.
- Use a `no_std` N2K stack such as `korri-n2k` on the ESP32 only, behind the
  same `CanBus` and connector traits.

## Still open for the phase 0 gate

- **ESP32 loop budget.** Not measured: it needs an ESP32-S3 on the bench
  and a core that does real work (today's `step()` holds the loop's shape
  and pypilot's heading-error rules; pilots and fusion arrive in phase 1).
  A `loop_budget` example will land with phase 1, to be run on an S3.
- **Raw logs from a pypilot boat.** `tools/record_pypilot.py` records
  them; it needs a sail with pypilot steering.

## Choices made in code

- **`f32` everywhere in the core.** The ESP32-S3 FPU is single precision.
  Golden tests against pypilot compare within tolerances.
- **The core never reads a clock.** Platforms pass `now` into `step()`, so
  replaying a log is deterministic.
- **NMEA 2000 sensor priority.** N2K readings use source `can` at priority
  1, as on the n2k branch's `sensors.py`.
- **Control levels.** `off`, `monitor`, `steer` and `full` from
  `docs/n2k-control.md` §6 gate every control link, not just N2K. Standby
  needs `monitor`, steering commands need `steer`, and value writes need
  `full`.
