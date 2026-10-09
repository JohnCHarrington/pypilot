#!/usr/bin/env python3
#
# Copyright (C) 2026 RustPilot contributors
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
'''Run pypilot's own control loop on scripted inputs and record what it does.

This drives the real `Autopilot.iteration` from pypilot/autopilot.py, with
the real sensors, servo, tacking and basic pilot code. Only the edges are
faked: the clock, the IMU (fed from the script), the value server, and the
motor controller (which records every call pypilot makes to it).

Each scenario is written as gzipped JSON lines to
crates/rustpilot-core/tests/golden/<name>.jsonl.gz: one line per loop step with
the inputs and pypilot's resulting state. The Rust test replays the inputs
through `Autopilot::step` and compares.

    python3 tools/golden/generate.py [--pypilot ../]

Needs only the standard library; pypilot's optional dependencies are
stubbed.
'''

import argparse
import builtins
import gzip
import json
import math
import os
import random
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', '..', 'crates', 'rustpilot-core', 'tests', 'golden')
# Times are exact binary fractions (1024 s plus multiples of 1/8 s) so that
# pypilot's float time differences equal RustPilot's integer microseconds;
# otherwise a 0.19999999 vs 0.2 comparison flips a throttle decision.
T0 = 1024.0  # pypilot's monotonic clock is never near 0
RATE = 8     # loop rate, Hz


def rounded(x):
    '''Round floats to 6 significant digits, keeping files small. Inputs
    are rounded before pypilot sees them, so both sides use the same
    numbers.'''
    if isinstance(x, float):
        return float('%.6g' % x)
    if isinstance(x, list):
        return [rounded(i) for i in x]
    if isinstance(x, dict):
        # times stay exact: they are binary fractions already
        return {k: v if k == 't' else rounded(v) for k, v in x.items()}
    return x


class Clock:
    def __init__(self):
        self.t = T0

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        pass  # the loop's own sleep; steps are driven explicitly


def load_pypilot(root):
    sys.path[:0] = [root, os.path.join(root, 'pypilot')]
    builtins._ = lambda s: s
    sys.modules.setdefault('serial', types.ModuleType('serial'))
    clock = Clock()
    import time
    time.monotonic = clock.monotonic
    time.sleep = clock.sleep

    import sensors
    class NoFilter:
        def __init__(self, client):
            pass
        def update(self, *args):
            pass
        def predict(self, *args):
            pass
    sensors.GPSFilterProcess = NoFilter
    stdout = sys.stdout
    sys.stdout = open(os.devnull, 'w')  # pilot discovery is chatty
    try:
        import autopilot, servo, tacking, rudder
        from pypilot.pilots import basic
    finally:
        sys.stdout.close()
        sys.stdout = stdout
    return clock, types.SimpleNamespace(autopilot=autopilot, servo=servo, tacking=tacking,
                                        sensors=sensors, rudder=rudder, basic=basic)


class Client:
    '''Stands in for pypilotClient: registers values locally.'''
    def __init__(self):
        self.values = types.SimpleNamespace(values={})

    def register(self, value):
        value.client = self
        value.watch = False
        value.pwatch = False
        self.values.values[value.name] = value
        return value

    def send(self, msg):
        pass

    def receive(self, timeout=0):
        return {}


class Driver:
    '''Stands in for ArduinoServo: telemetry from the script, calls recorded.'''
    def __init__(self):
        self.calls = []
        self.telemetry = 0
        self.flags = 0
        self.current = self.voltage = 0
        self.controller_temp = self.motor_temp = 0
        self.rudder = 0

    def command(self, command):
        self.calls.append(['command', command])

    def disengage(self):
        self.calls.append(['disengage'])

    def reset(self):
        self.calls.append(['reset'])

    def params(self, *args):
        self.last_params = args

    def poll(self):
        t, self.telemetry = self.telemetry, 0
        return t

    def fault(self):
        return bool(self.flags & 4)  # OVERCURRENT_FAULT


class IMU:
    '''Stands in for BoatIMU: the fused values come from the script.'''
    def __init__(self, client, values):
        self.SensorValues = {}
        for name in ['heading', 'heading_lowpass', 'headingrate_lowpass', 'headingraterate_lowpass',
                     'heel', 'rollrate', 'pitchrate', 'accel', 'fusionQPose']:
            self.SensorValues[name] = client.register(values.SensorValue('imu.' + name))
        self.rate = types.SimpleNamespace(value=RATE)
        self.heading_off = client.register(values.RangeProperty('imu.heading_offset', 90, -180, 180))
        self.heel = 0
        self.data = False

    def read(self):
        data, self.data = self.data, False
        if data:
            for name in ['heading', 'heading_lowpass', 'headingrate_lowpass', 'headingraterate_lowpass',
                         'heel', 'rollrate', 'pitchrate']:
                self.SensorValues[name].set(data[name])
            self.heel = data['heel']
            self.heading_off.set(data['heading_offset'])
        return data

    def poll(self, calibrate):
        pass


def build(clock, m):
    '''An Autopilot with real logic and fake edges, as Autopilot.__init__
    would build it without its server, processes and hardware.'''
    import values
    stub = lambda **kw: types.SimpleNamespace(**kw)
    client = Client()
    ap = object.__new__(m.autopilot.Autopilot)
    ap.watchdog_device = False
    ap.server = stub(poll=lambda: None, __del__=lambda: None)
    ap.client = client
    ap.boatimu = IMU(client, values)

    s = object.__new__(m.sensors.Sensors)
    s.client = client
    s.nmea = s.signalk = s.gpsd = stub(poll=lambda: None)
    s.gps = m.sensors.gps(client)
    s.wind = m.sensors.Wind(client, ap.boatimu)
    s.truewind = m.sensors.TrueWind(client, ap.boatimu)
    s.rudder = m.rudder.Rudder(client)
    s.apb = m.sensors.APB(client)
    s.water = m.sensors.Water(client)
    s.sensors = {'gps': s.gps, 'wind': s.wind, 'truewind': s.truewind,
                 'rudder': s.rudder, 'apb': s.apb, 'water': s.water}
    s.rudder.update_minmax()
    ap.sensors = s

    m.servo.Servo.calibration_filename = '/nonexistent'
    sv = m.servo.Servo(client, s)
    sv.calibration.set({'port': [.2, .8], 'starboard': [.2, .8]})
    sv.driver = Driver()
    sv.device = stub(path='servo', port='servo', baudrate=38400)
    sv.lastpolltime = clock.t
    sv.controller.set('arduino')
    ap.servo = sv

    A = m.autopilot
    ap.version = ap.register(values.Value, 'version', 'test')
    ap.timestamp = client.register(A.TimeStamp())
    ap.starttime = clock.t
    ap.preferred_mode = ap.register(values.Value, 'preferred_mode', 'compass')
    ap.preferred_mode.command = None
    ap.mode = ap.register(A.ModeProperty, 'mode', ap)
    ap.modes = ap.register(values.JSONValue, 'modes', [])
    ap.modes.sensors = {}
    ap.gps_and_nav_modes = ap.register(values.BooleanProperty, 'gps_and_nav_modes', True, persistent=True)
    ap.lastmode = False
    ap.heading_command = ap.register(A.HeadingProperty, 'heading_command', ap.mode)
    ap.enabled = ap.register(values.BooleanProperty, 'enabled', False)
    ap.lastenabled = False
    ap.last_heading = False
    ap.last_heading_off = ap.boatimu.heading_off.value
    ap.heading = ap.register(values.SensorValue, 'heading', directional=True)
    ap.heading_error = ap.register(values.SensorValue, 'heading_error')
    ap.heading_error_int = ap.register(values.SensorValue, 'heading_error_int')
    ap.heading_error_int_time = clock.t
    ap.heading_command_rate = ap.register(values.SensorValue, 'heading_command_rate')
    ap.heading_command_rate.time = 0
    ap.pilots = {}
    pilot = m.basic.BasicPilot(ap)
    ap.pilots[pilot.name] = pilot
    ap.pilot = ap.register(values.EnumProperty, 'pilot', 'basic', ['basic'], persistent=True, profiled=True)
    ap.tack = m.tacking.Tack(ap)
    ap.gps_compass_offset = ap.register(values.HeadingOffset, 'gps_compass_offset')
    ap.gps_speed = 0
    ap.wind_compass_offset = ap.register(values.HeadingOffset, 'wind_compass_offset')
    ap.true_wind_compass_offset = ap.register(values.HeadingOffset, 'true_wind_compass_offset')
    ap.wind_offset_filter = ap.register(values.RangeProperty, 'wind_offset_filter', 0.1, 0.01, 0.5, persistent=True)
    ap.runtime = ap.register(m.autopilot.TimeValue, 'runtime')
    ap.timings = ap.register(values.SensorValue, 'timings', False)
    ap.last_heading_mode = False
    ap.lasttime = clock.t
    return ap


RECORD = ['ap.enabled', 'ap.mode', 'ap.heading', 'ap.heading_command', 'ap.heading_error',
          'ap.heading_error_int', 'ap.heading_command_rate', 'ap.gps_compass_offset',
          'ap.wind_compass_offset', 'ap.true_wind_compass_offset',
          'ap.pilot.basic.Pgain', 'ap.pilot.basic.Dgain', 'ap.pilot.basic.DDgain',
          'ap.pilot.basic.PRgain', 'ap.pilot.basic.FFgain',
          'ap.tack.state', 'ap.tack.direction', 'ap.tack.count',
          'servo.command', 'servo.speed', 'servo.raw_command', 'servo.position', 'servo.state',
          'servo.duty', 'servo.current', 'servo.voltage',
          'rudder.angle', 'rudder.source', 'gps.source', 'wind.source', 'truewind.source',
          'wind.direction', 'wind.filtered_direction', 'truewind.direction', 'truewind.speed']


def apply_step(ap, step):
    '''Feed one step's inputs to pypilot and run one loop iteration.'''
    driver = ap.servo.driver
    for vname, value in step.get('sets', []):
        ap.client.values.values[vname].set(value)
    for nav in step.get('nav', []):
        ap.sensors.write(nav['sensor'], dict(nav['data'], device=nav['device']), nav['source'])
    tel = step.get('telemetry')
    if tel:
        mask = 1  # FLAGS
        driver.flags = tel['flags']
        for key, bit in [('current', 2), ('voltage', 4), ('controller_temp', 32),
                         ('motor_temp', 64), ('rudder', 128)]:
            if key in tel:
                setattr(driver, key, tel[key] if tel[key] is not None else float('nan'))
                mask |= bit
        driver.telemetry = mask
    if step.get('imu'):
        ap.boatimu.data = dict(step['imu'])
    driver.calls = []
    ap.iteration()
    values = ap.client.values.values
    state = {v: values[v].value for v in RECORD}
    state['servo.flags'] = ap.servo.flags.value
    return driver.calls, state


def write(name, out):
    path = os.path.join(OUT, name + '.jsonl.gz')
    # mtime=0 keeps the file byte-identical across runs
    with open(path, 'wb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', mtime=0) as f:
        for line in out:
            f.write((json.dumps(rounded(line), separators=(',', ':')) + '\n').encode())
    print('wrote', len(out), 'steps to', os.path.relpath(path))


def imu_sample(heading, rate, raterate=0.0, heel=0.0, offset=90):
    return {'heading': heading % 360, 'heading_lowpass': heading % 360,
            'headingrate_lowpass': rate, 'headingraterate_lowpass': raterate,
            'heel': heel, 'rollrate': 0.0, 'pitchrate': 0.0, 'heading_offset': offset}


class Boat:
    '''A crude yaw model so headings respond to the rudder pypilot drives.
    It only has to make the inputs realistic; parity is checked open loop.'''
    def __init__(self, rng, heading=40.0):
        self.rng = rng
        self.heading = heading
        self.rate = 0.0
        self.rudder = 0.0

    def advance(self, raw_command, dt, disturbance):
        self.rudder = max(-30, min(30, self.rudder + raw_command * 20 * dt))
        accel = -0.6 * self.rudder - 0.8 * self.rate + disturbance
        self.rate += accel * dt
        self.heading += self.rate * dt
        return imu_sample(self.heading + self.rng.gauss(0, 0.2), self.rate, accel)


def wind_reading(boat, rng, wind_from=200):
    direction = ((wind_from - boat.heading + 180) % 360) - 180 + rng.gauss(0, 2)
    return {'sensor': 'wind', 'source': 'serial', 'device': '/dev/ttyUSB0',
            'data': {'direction': direction, 'speed': 14 + rng.gauss(0, 1)}}


def last_command(calls, previous):
    for c in reversed(calls):
        if c[0] == 'command':
            return c[1]
        if c[0] == 'disengage':
            return 0.0
    return previous


def scenario(clock, m, name, seconds, events, seed, nav=None, telemetry=None):
    '''Run pypilot against the boat model and record every step. `events`
    maps a time in seconds to value sets; `nav` (a list of sensor readings)
    and `telemetry` generate those inputs per step.'''
    rng = random.Random(seed)
    boat = Boat(rng)
    clock.t = T0
    ap = build(clock, m)
    out = []
    command = 0.0
    for i in range(seconds * RATE):
        t = i / RATE
        clock.t = T0 + t
        disturbance = 3 * math.sin(t / 3.0) + rng.gauss(0, 1.5)
        step = {'t': t, 'imu': boat.advance(command, 1 / RATE, disturbance)}
        if t in events:
            step['sets'] = events[t]
        if nav:
            readings = nav(t, boat, rng)
            if readings:
                step['nav'] = readings
        step['telemetry'] = telemetry(t, boat, rng) if telemetry else {'flags': 9}
        step = rounded(step)
        calls, state = apply_step(ap, step)
        command = last_command(calls, command)
        out.append({'t': clock.t, 'step': step, 'calls': calls, 'state': state})
    write(name, out)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--pypilot', default=os.path.join(HERE, '..', '..', '..'),
                        help='pypilot checkout (default: the repository this sits in)')
    args = parser.parse_args()
    clock, m = load_pypilot(os.path.abspath(args.pypilot))
    os.makedirs(OUT, exist_ok=True)

    # 1. compass mode: engage, hold, change course, standby
    scenario(clock, m, 'compass', 120, {
        2.0: [('ap.heading_command', 40), ('ap.enabled', True)],
        40.0: [('ap.heading_command', 70)],
        70.0: [('ap.heading_command', 10)],
        100.0: [('ap.pilot.basic.P', 0.006), ('servo.period', 0.6)],
        115.0: [('ap.enabled', False)],
    }, seed=1)

    # 2. wind mode with the wind instrument coming and going, and a tack
    def wind(t, boat, rng):
        if 50 < t < 70:
            return None  # instrument drops out: fall back to compass
        return [wind_reading(boat, rng)]
    scenario(clock, m, 'wind', 120, {
        2.0: [('ap.mode', 'wind'), ('ap.enabled', True)],
        30.0: [('ap.heading_command', -150)],
        80.0: [('ap.tack.direction', 'port'), ('ap.tack.state', 'begin')],
    }, seed=2, nav=wind)

    # 3. servo faults and rudder feedback
    def telemetry(t, boat, rng):
        flags = 9  # SYNC | ENGAGED
        if 30 < t < 31:
            flags |= 4  # overcurrent
        tel = {'flags': flags, 'current': max(0, 2 + rng.gauss(0, .5)), 'voltage': 12.4}
        if t > 10:
            tel['rudder'] = boat.rudder / 100.0
        return tel
    scenario(clock, m, 'servo', 60, {
        2.0: [('ap.heading_command', 40), ('ap.enabled', True)],
        45.0: [('ap.enabled', False)],
        46.0: [('servo.position_command', 10)],
        52.0: [('servo.command', -0.5)],
    }, seed=3, telemetry=telemetry)

    # 4. GPS, route following and synthesised true wind
    def gps_nav(t, boat, rng):
        out = [wind_reading(boat, rng)]
        if t == int(t):  # GPS at 1 Hz
            out.append({'sensor': 'gps', 'source': 'serial', 'device': '/dev/ttyUSB1',
                        'data': {'speed': 6 + rng.gauss(0, .2), 'track': (boat.heading + 5 + rng.gauss(0, 1)) % 360}})
        if t > 40 and t * 2 == int(t * 2):  # APB at 2 Hz once a route is active
            out.append({'sensor': 'apb', 'source': 'tcp', 'device': 'tcp',
                        'data': {'track': 70.0, 'xte': 0.01 * math.sin(t / 10), 'mode': 'gps'}})
        return out
    scenario(clock, m, 'gps', 120, {
        2.0: [('ap.enabled', True)],
        20.0: [('ap.mode', 'gps')],
        50.0: [('ap.mode', 'nav')],
        90.0: [('ap.mode', 'true wind')],
    }, seed=4, nav=gps_nav)


if __name__ == '__main__':
    main()
