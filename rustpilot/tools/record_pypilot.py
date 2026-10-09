#!/usr/bin/env python3
#
# Copyright (C) 2026 RustPilot contributors
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
'''Record a running pypilot's inputs and outputs for RustPilot replay tests.

Connects to pypilot's JSON server, watches the IMU, sensor, autopilot and
servo values every time they change, and writes one JSON object per line:

    {"t": 12.3456, "name": "imu.accel", "value": [0.01, -0.02, 1.0]}

`t` is seconds since recording started, on this machine's monotonic clock.
Run it on the pypilot host (or anywhere that reaches port 23322) while
sailing, ideally with some engaged steering in calm and rough water:

    python3 record_pypilot.py --host pypilot.local -o boat-2026-10-09.jsonl

Stop with Ctrl-C. Only the standard library is used.
'''

import argparse
import gzip
import json
import socket
import sys
import time

DEFAULT_PORT = 23322

# What RustPilot's IMU fusion, core and servo logic need to reproduce
# pypilot's behaviour. Raw IMU in, fused heading and servo commands out.
VALUES = [
    # raw IMU and pypilot's fusion result
    'imu.accel', 'imu.gyro', 'imu.compass', 'imu.fusionQPose',
    'imu.alignmentQ', 'imu.heading_offset', 'imu.rate',
    'imu.heading', 'imu.heading_lowpass', 'imu.headingrate_lowpass',
    'imu.headingraterate_lowpass', 'imu.pitch', 'imu.roll', 'imu.heel',
    'imu.compass.calibration',
    # navigation inputs
    'gps.source', 'gps.track', 'gps.speed', 'wind.source', 'wind.direction',
    'wind.speed', 'truewind.source', 'truewind.direction', 'truewind.speed',
    'water.speed', 'apb.source', 'apb.track', 'apb.xte',
    'rudder.source', 'rudder.angle',
    # autopilot state and decisions
    'ap.enabled', 'ap.mode', 'ap.pilot', 'ap.heading', 'ap.heading_command',
    'ap.heading_error', 'ap.heading_error_int', 'ap.heading_command_rate',
    'ap.tack.state', 'ap.tack.direction',
    'ap.pilot.basic.P', 'ap.pilot.basic.I', 'ap.pilot.basic.D',
    'ap.pilot.basic.DD', 'ap.pilot.basic.PR', 'ap.pilot.basic.FF',
    # servo commands and telemetry
    'servo.command', 'servo.raw_command', 'servo.position_command',
    'servo.speed', 'servo.current', 'servo.voltage', 'servo.flags',
    'servo.engaged', 'servo.controller_temp', 'servo.motor_temp',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('-o', '--output', required=True,
                        help='output file; a .gz suffix compresses it')
    args = parser.parse_args()

    sock = socket.create_connection((args.host, args.port), timeout=10)
    sock.settimeout(None)
    # The server ignores names it doesn't have, so optional sensors are fine.
    watch = {name: True for name in VALUES}
    sock.sendall(('watch=' + json.dumps(watch) + '\n').encode())

    opener = gzip.open if args.output.endswith('.gz') else open
    start = time.monotonic()
    lines = 0
    buf = b''
    with opener(args.output, 'wt') as out:
        try:
            while True:
                data = sock.recv(65536)
                if not data:
                    print('connection closed by pypilot', file=sys.stderr)
                    break
                t = round(time.monotonic() - start, 4)
                buf += data
                *complete, buf = buf.split(b'\n')
                for line in complete:
                    name, sep, value = line.decode(errors='replace').partition('=')
                    if not sep:
                        continue
                    try:
                        value = json.loads(value)
                    except ValueError:
                        pass  # keep the raw text
                    out.write(json.dumps({'t': t, 'name': name, 'value': value}) + '\n')
                    lines += 1
                    if lines % 10000 == 0:
                        print('%d values, %.0f s' % (lines, t), file=sys.stderr)
        except KeyboardInterrupt:
            pass
    print('wrote %d values to %s' % (lines, args.output), file=sys.stderr)


if __name__ == '__main__':
    main()
