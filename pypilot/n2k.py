#!/usr/bin/env python

# canboatjs and n2k-signalk bridge for pypilot

import json
import os
import select
import shlex
import subprocess
import time

from client import pypilotClient
from signalk_table import signalk_table
from sensors import source_priority
from values import BooleanProperty, Property, StringValue

DEFAULT_WORKER_COMMAND = 'node %s' % os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'n2k_worker', 'index.js'))


class N2KSignalK(object):
    def __init__(self, sensors=False):
        self.sensors = sensors
        if sensors:
            server = sensors.client.server
            self.client = pypilotClient(server)
        else:
            self.client = pypilotClient()

        self.process = False
        self.poller = select.poll()
        self.fd_to_name = {}
        self.stdout_fd = False
        self.stderr_fd = False
        self.stdout_buffer = ''
        self.stderr_buffer = ''
        self.initialized = False
        self.worker_config = False
        self.last_values = {
            'gps.filtered.output': False,
            'ap.mode': 'compass',
            'ap.enabled': False,
            'ap.heading_command': 0,
            'gps.source': 'none',
            'wind.source': 'none',
            'truewind.source': 'none',
            'rudder.source': 'none',
            'water.source': 'none',
            'apb.source': 'none'
        }
        self.signalk_cache = {}
        self.signalk_times = {}
        self.output_times = {}

    def setup(self):
        self.enabled = self.client.register(BooleanProperty('n2k.enabled', False, persistent=True))
        self.interface = self.client.register(Property('n2k.interface', 'can0', persistent=True))
        self.worker_command = self.client.register(Property('n2k.canboatjs.command',
                                                            DEFAULT_WORKER_COMMAND,
                                                            persistent=True))
        self.address = self.client.register(Property('n2k.address', 25, persistent=True))
        self.device_name = self.client.register(Property('n2k.name', 'pypilot', persistent=True))
        self.status = self.client.register(StringValue('n2k.status', 'disabled'))
        self.error = self.client.register(StringValue('n2k.error', ''))

        self.output_enable = {
            'attitude': self.client.register(BooleanProperty('n2k.output.attitude', True, persistent=True)),
            'heading': self.client.register(BooleanProperty('n2k.output.heading', True, persistent=True)),
            'rate_of_turn': self.client.register(BooleanProperty('n2k.output.rate_of_turn', True, persistent=True)),
            'autopilot': self.client.register(BooleanProperty('n2k.output.autopilot', True, persistent=True)),
            'wind': self.client.register(BooleanProperty('n2k.output.wind', False, persistent=True)),
            'rudder': self.client.register(BooleanProperty('n2k.output.rudder', False, persistent=True)),
            'gps_filtered': self.client.register(BooleanProperty('n2k.output.gps_filtered', False, persistent=True)),
        }

        self.setup_watches()
        self.initialized = True

    def setup_watches(self):
        for name in self.last_values:
            self.client.watch(name)

        watchlist = [
            ('imu.pitch', .1),
            ('imu.roll', .1),
            ('imu.heading_lowpass', .1),
            ('imu.headingrate_lowpass', .1),
            ('wind.direction', .25),
            ('wind.speed', .25),
            ('truewind.direction', .25),
            ('truewind.speed', .25),
            ('rudder.angle', .25),
        ]
        for name, period in watchlist:
            self.client.watch(name, period)

        for name in ['wind.rate', 'truewind.rate', 'rudder.rate']:
            self.client.watch(name)

        self.update_gps_fix_watch()

    def update_gps_fix_watch(self):
        watch = self.last_values.get('gps.filtered.output') and self.output_enable['gps_filtered'].value
        self.client.watch('gps.filtered.fix', watch)

    def current_worker_config(self):
        return (bool(self.enabled.value), self.interface.value,
                self.worker_command.value, int(self.address.value or 25),
                self.device_name.value)

    def set_status(self, status, error=''):
        self.status.set(status)
        self.error.set(error)

    def register_stream(self, name, stream):
        fd = stream.fileno()
        self.poller.register(fd, select.POLLIN)
        self.fd_to_name[fd] = name
        if name == 'stdout':
            self.stdout_fd = fd
        else:
            self.stderr_fd = fd

    def unregister_stream(self, fd):
        if fd is False:
            return
        try:
            self.poller.unregister(fd)
        except Exception:
            pass
        self.fd_to_name.pop(fd, None)

    def stop_worker(self):
        self.unregister_stream(self.stdout_fd)
        self.unregister_stream(self.stderr_fd)
        self.stdout_fd = False
        self.stderr_fd = False
        self.stdout_buffer = ''
        self.stderr_buffer = ''

        if not self.process:
            return

        try:
            self.send_command({'type': 'shutdown'})
        except Exception:
            pass

        try:
            self.process.stdin.close()
        except Exception:
            pass

        try:
            self.process.terminate()
            self.process.wait(2)
        except Exception:
            try:
                self.process.kill()
            except Exception:
                pass

        self.process = False

    def start_worker(self):
        self.stop_worker()

        command = shlex.split(self.worker_command.value)
        if not command:
            self.set_status('configuration required', 'set n2k.canboatjs.command')
            return

        try:
            self.process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            )
        except Exception as error:
            self.process = False
            self.set_status('worker error', str(error))
            return

        self.register_stream('stdout', self.process.stdout)
        self.register_stream('stderr', self.process.stderr)
        self.send_command({
            'type': 'configure',
            'config': {
                'canDevice': self.interface.value,
                'preferredAddress': int(self.address.value or 25),
                'modelId': self.device_name.value,
                'modelVersion': 'pypilot',
                'softwareVersionCode': '0.1.0'
            }
        })
        self.send_command({'type': 'set-enabled', 'enabled': True})
        self.set_status('starting')

    def ensure_worker(self):
        config = self.current_worker_config()
        if config == self.worker_config:
            return

        self.worker_config = config
        if not self.enabled.value:
            self.stop_worker()
            self.set_status('disabled')
            return

        self.start_worker()

    def send_command(self, command):
        if not self.process or not self.process.stdin:
            return False
        payload = json.dumps(command) + '\n'
        self.process.stdin.write(payload.encode('utf-8'))
        self.process.stdin.flush()
        return True

    def read_stream(self, name):
        fd = self.stdout_fd if name == 'stdout' else self.stderr_fd
        if fd is False:
            return

        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            return
        except Exception as error:
            self.set_status('worker error', str(error))
            self.stop_worker()
            self.worker_config = False
            return

        if not data:
            self.set_status('worker stopped')
            self.stop_worker()
            self.worker_config = False
            return

        if name == 'stdout':
            buffer = self.stdout_buffer + data.decode('utf-8', 'replace')
        else:
            buffer = self.stderr_buffer + data.decode('utf-8', 'replace')

        lines = buffer.split('\n')
        remainder = lines.pop()

        if name == 'stdout':
            self.stdout_buffer = remainder
        else:
            self.stderr_buffer = remainder

        for line in lines:
            line = line.strip()
            if line:
                self.handle_line(name, line)

    def handle_line(self, name, line):
        if name == 'stderr':
            self.error.set(line)
            return

        try:
            event = json.loads(line)
        except Exception:
            self.set_status('worker error', 'invalid worker output')
            self.error.set(line)
            return

        event_type = event.get('type')
        if event_type == 'status':
            self.set_status(event.get('state', 'worker'), event.get('details', {}).get('error', ''))
        elif event_type == 'delta':
            self.receive_delta(event.get('details', {}).get('delta', {}))
        elif event_type in ['error', 'worker-error']:
            details = event.get('details', {})
            self.set_status('worker error', event.get('error', details.get('error', details.get('message', 'worker error'))))

    def event_source_name(self, update):
        source = update.get('source', {})
        if 'canName' in source:
            return source['canName']
        if 'src' in source:
            return 'src%s' % source['src']
        if 'label' in source and source['label']:
            return source['label']
        return 'n2k'

    def receive_delta(self, delta):
        if not self.sensors:
            return

        for update in delta.get('updates', []):
            source_name = self.event_source_name(update)
            if source_name not in self.signalk_cache:
                self.signalk_cache[source_name] = {}

            timestamp = update.get('timestamp')
            for value in update.get('values', []):
                path = value.get('path')
                if not path:
                    continue
                self.signalk_cache[source_name][path] = value.get('value')
                if timestamp:
                    self.signalk_times[(source_name, path)] = timestamp

        self.emit_sensor_messages()

    def emit_sensor_messages(self):
        for sensor, sensor_table in signalk_table.items():
            for source_name, values in self.signalk_cache.items():
                data = {}
                for signalk_path_conversion, pypilot_path in sensor_table.items():
                    signalk_path, signalk_conversion = signalk_path_conversion
                    if signalk_path in values and values[signalk_path] is not None:
                        value = values[signalk_path]
                        timestamp = self.signalk_times.get((source_name, signalk_path))
                        if 'timestamp' not in data and timestamp:
                            try:
                                try:
                                    ts = time.strptime(timestamp, '%Y-%m-%dT%H:%M:%S.%fZ')
                                except Exception:
                                    ts = time.strptime(timestamp, '%Y-%m-%dT%H:%M:%SZ')
                                data['timestamp'] = time.mktime(ts)
                            except Exception:
                                pass

                        if type(pypilot_path) == type({}):
                            for signalk_key, pypilot_key in pypilot_path.items():
                                if value.get(signalk_key) is not None:
                                    data[pypilot_key] = value[signalk_key] / signalk_conversion
                        else:
                            data[pypilot_path] = value / signalk_conversion
                    elif signalk_conversion != 1:
                        break
                else:
                    for signalk_path_conversion in sensor_table:
                        signalk_path, signalk_conversion = signalk_path_conversion
                        if signalk_path in values:
                            del values[signalk_path]
                    data['device'] = 'n2k-' + source_name
                    self.sensors.write(sensor, data, 'n2k')
                    break

    def send_topic(self, payload):
        self.send_command({'type': 'send', 'payload': payload})

    def output_messages(self):
        if not self.enabled.value or not self.process:
            return

        values = self.client.values.values
        now = time.monotonic()

        if self.output_enable['attitude'].value and 'imu.pitch' in values and 'imu.roll' in values:
            last = self.output_times.get('attitude', 0)
            if now - last >= 0.1:
                self.send_topic({
                    'topic': 'attitude',
                    'pitch': values['imu.pitch'].value,
                    'roll': values['imu.roll'].value
                })
                self.output_times['attitude'] = now

        if self.output_enable['heading'].value and 'imu.heading_lowpass' in values:
            last = self.output_times.get('heading', 0)
            if now - last >= 0.1:
                self.send_topic({
                    'topic': 'heading',
                    'heading': values['imu.heading_lowpass'].value
                })
                self.output_times['heading'] = now

        if self.output_enable['rate_of_turn'].value and 'imu.headingrate_lowpass' in values:
            last = self.output_times.get('rate_of_turn', 0)
            if now - last >= 0.1:
                self.send_topic({
                    'topic': 'rate_of_turn',
                    'rate': values['imu.headingrate_lowpass'].value
                })
                self.output_times['rate_of_turn'] = now

        for name in ['wind', 'truewind', 'rudder']:
            source = self.last_values.get(name + '.source', 'none')
            if source_priority.get(source, 7) <= source_priority['n2k']:
                continue

            rate_name = name + '.rate'
            rate_value = values[rate_name].value if rate_name in values else 4
            period = 1.0 / rate_value if rate_value > 0 else 0.25
            last = self.output_times.get(name, 0)
            if now - last < period:
                continue

            if name == 'wind' and 'wind.direction' in values and 'wind.speed' in values and self.output_enable['wind'].value:
                self.send_topic({
                    'topic': 'wind',
                    'direction': values['wind.direction'].value,
                    'speed': values['wind.speed'].value
                })
                self.output_times[name] = now
            elif name == 'truewind' and 'truewind.direction' in values and 'truewind.speed' in values and self.output_enable['wind'].value:
                self.send_topic({
                    'topic': 'truewind',
                    'direction': values['truewind.direction'].value,
                    'speed': values['truewind.speed'].value
                })
                self.output_times[name] = now
            elif name == 'rudder' and 'rudder.angle' in values and self.output_enable['rudder'].value:
                self.send_topic({
                    'topic': 'rudder',
                    'angle': values['rudder.angle'].value
                })
                self.output_times[name] = now

        if self.output_enable['autopilot'].value and \
           'ap.mode' in values and 'ap.enabled' in values and 'ap.heading_command' in values:
            last = self.output_times.get('autopilot', 0)
            if now - last >= 0.5:
                self.send_topic({
                    'topic': 'autopilot',
                    'mode': values['ap.mode'].value,
                    'enabled': values['ap.enabled'].value,
                    'heading_command': values['ap.heading_command'].value
                })
                self.output_times['autopilot'] = now

        if self.output_enable['gps_filtered'].value and self.last_values.get('gps.filtered.fix'):
            fix = self.last_values['gps.filtered.fix']
            self.last_values['gps.filtered.fix'] = False
            self.send_topic({
                'topic': 'gps',
                'lat': fix.get('lat'),
                'lon': fix.get('lon'),
                'speed': fix.get('speed'),
                'track': fix.get('track'),
                'timestamp': fix.get('timestamp')
            })

    def poll_client(self):
        messages = self.client.receive()
        for name in messages:
            self.last_values[name] = messages[name]
            if name == 'gps.filtered.output' or name == 'n2k.output.gps_filtered':
                self.update_gps_fix_watch()

    def poll(self):
        if not self.initialized:
            self.setup()
            return

        self.ensure_worker()
        self.poll_client()
        self.output_messages()

        events = self.poller.poll(0)
        for fd, flag in events:
            name = self.fd_to_name.get(fd)
            if not name:
                continue
            if flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                self.set_status('worker error', 'worker disconnected')
                self.stop_worker()
                self.worker_config = False
                return
            self.read_stream(name)


    pass
