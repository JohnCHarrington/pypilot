#!/usr/bin/env python

# NMEA2000 bridge for pypilot

import asyncio
import hashlib
import multiprocessing
import random
import select
import threading
import time
import version

from client import pypilotClient
from nonblockingpipe import NonBlockingPipe
from sensors import source_priority
from values import EnumProperty, Property, StringValue
import logging

import nmea2000

log = logging.getLogger('n2k')

ISO_REQUEST_PGN = 59904
ISO_ADDRESS_CLAIM_PGN = 60928
ISO_ACKNOWLEDGEMENT_PGN = 59392
GROUP_FUNCTION_PGN = 126208
HEARTBEAT_PGN = 126993
PRODUCT_INFORMATION_PGN = 126996
CONFIGURATION_INFORMATION_PGN = 126998
PGN_LIST_PGN = 126464

STARTUP_ADDRESS_CLAIM_SOURCE = 254
ADDRESS_CLAIM_DETECTION_TIME = 5.0
HEARTBEAT_INTERVAL = 60.0
TRANSMIT_PGN_LIST_FUNCTION = 0

BASE_TRANSMIT_PGNS = [
    ISO_REQUEST_PGN,
    ISO_ADDRESS_CLAIM_PGN,
    ISO_ACKNOWLEDGEMENT_PGN,
    HEARTBEAT_PGN,
    PRODUCT_INFORMATION_PGN,
    CONFIGURATION_INFORMATION_PGN,
    PGN_LIST_PGN,
]


class N2KBridge(object):
    def __init__(self, server):
        self.client = pypilotClient(server)
        self.client.connection.name += 'n2kbridge'
        self.multiprocessing = server.multiprocessing
        self.pipe, self.pipe_out = NonBlockingPipe('n2k pipe', self.multiprocessing)
        # Require multiprocessing mode — N2K runs only in a separate process.
        if not self.multiprocessing:
            raise RuntimeError('N2KBridge requires server multiprocessing to be enabled')

        self.process = multiprocessing.Process(target=lambda: asyncio.run(self.n2k_process()), daemon=True)
        self.process.start()

    def set_status(self, value, error=''):
        print("N2KBridge: Status update: %s, error: %s" % (value, error))
        self.n2k_status.set(value)
        self.n2k_error.set(error)

    async def setup(self):
        self.n2k_interface = self.client.register(Property('n2k.interface', 'can0', persistent=True))
        self.n2k_transport = self.client.register(EnumProperty('n2k.transport', 'none',
                                                               ['none', 'socketcan', 'actisense', 'ebyte', 'usb'],
                                                               persistent=True))
        self.n2k_host = self.client.register(Property('n2k.host', '', persistent=True))
        self.n2k_port = self.client.register(Property('n2k.port', 0, persistent=True))
        self.n2k_usb_device = self.client.register(Property('n2k.usb_device', '', persistent=True))
        self.n2k_address = self.client.register(Property('n2k.address', 25, persistent=True))
        self.n2k_name = self.client.register(Property('n2k.name', 'pypilot', persistent=True))
        self.n2k_unique_number = self.client.register(Property('n2k.unique_number', 0, persistent=True))
        self.n2k_status = self.client.register(StringValue('n2k.status', 'initializing'))
        self.n2k_error = self.client.register(StringValue('n2k.error', ''))

        self.n2k_output_enable = {
            'attitude': self.client.register(Property('n2k.output.attitude', True, persistent=True)),
            'heading': self.client.register(Property('n2k.output.heading', True, persistent=True)),
            'rate_of_turn': self.client.register(Property('n2k.output.rate_of_turn', True, persistent=True)),
            'autopilot': self.client.register(Property('n2k.output.autopilot', True, persistent=True)),
            'wind': self.client.register(Property('n2k.output.wind', False, persistent=True)),
            'rudder': self.client.register(Property('n2k.output.rudder', False, persistent=True)),
            'gps_filtered': self.client.register(Property('n2k.output.gps_filtered', False, persistent=True)),
        }

        old_default_pgn_filters = [129029, 129026, 129033, 130306, 127245, 128259]
        default_pgn_filters = [129029, 129026, 129033, 129283, 129284, 130306, 127245, 128259]
        self.pgn_filters = self.client.register(Property('n2k.pgn_filters',
            default_pgn_filters, persistent=True))
        if self.pgn_filters.value == old_default_pgn_filters:
            self.pgn_filters.set(default_pgn_filters)

        names = ['gps.source', 'wind.source', 'truewind.source', 'rudder.source',
                 'apb.source', 'water.source', 'gps.filtered.output',
                 'ap.mode', 'ap.enabled', 'ap.heading_command']
        self.last_values = {'gps.filtered.output': False,
                            'ap.mode': 'compass',
                            'ap.enabled': False,
                            'ap.heading_command': 0}
        for name in names:
            if not name in self.last_values:
                self.last_values[name] = 'none'

        self.watch_gps_fix = False
        self.gateway = None
        self.transport_config = False
        self.poller = select.poll()
        self.fd_to_source = {}
        self.msgs = {}
        self.n2k_times = {}
        self.n2k_response_times = {}
        self.last_imu_time = time.monotonic()
        self.gps_devices = {}
        self.nav_devices = {}
        self.n2k_sid = 0
        self.devices = {}
        self.cansend = False
        self.claim_in_progress = False
        self.found_conflict = False
        self.address_claim_sent_at = 0.0
        self.heartbeat_counter = 0
        self.next_heartbeat_time = 0.0

        if not self.n2k_unique_number.value:
            unique_number = random.getrandbits(21) & 0x1fffff
            self.n2k_unique_number.set(unique_number or 1)

        self.setup_watches()
        await self.init_transport()

    def setup_watches(self):
        for name in self.last_values:
            self.client.watch(name)

        watchlist = ['imu.pitch', 'imu.roll', 'imu.heading_lowpass',
                     'imu.headingrate_lowpass', 'wind.direction', 'wind.speed',
                     'truewind.direction', 'truewind.speed', 'rudder.angle']
        for name in watchlist:
            self.client.watch(name)

        for name in ['wind.rate', 'truewind.rate', 'rudder.rate']:
            self.client.watch(name)

        self.update_gps_fix_watch()

    def update_gps_fix_watch(self):
        watch = self.last_values.get('gps.filtered.output') and self.n2k_output_enable['gps_filtered'].value
        if watch != self.watch_gps_fix:
            self.client.watch('gps.filtered.fix', watch)
            self.watch_gps_fix = watch

    def next_sid(self):
        # For single-frame messages, SID should be 0. Return 0.
        return 0

    def current_transport_config(self):
        pgn_filters = self.pgn_filters.value
        if type(pgn_filters) == type([]):
            pgn_filters = tuple(pgn_filters)
        else:
            pgn_filters = ()
        return (self.n2k_transport.value, self.n2k_interface.value, self.n2k_host.value,
                int(self.n2k_port.value or 0), self.n2k_usb_device.value, pgn_filters)

    def transport_include_pgns(self):
        pgn_filters = self.pgn_filters.value
        if not isinstance(pgn_filters, list):
            pgn_filters = []

        include_pgns = list(pgn_filters)
        for required_pgn in [ISO_REQUEST_PGN, ISO_ADDRESS_CLAIM_PGN, GROUP_FUNCTION_PGN]:
            if required_pgn not in include_pgns:
                include_pgns.append(required_pgn)
        return include_pgns

    def claim_unique_number(self):
        unique_number = int(self.n2k_unique_number.value or 0) & 0x1fffff
        if not unique_number:
            identity = '%s:%s:%s' % (self.n2k_name.value, self.n2k_address.value, time.time_ns())
            unique_number = int(hashlib.sha1(identity.encode('utf-8')).hexdigest()[:8], 16) & 0x1fffff
            unique_number = unique_number or 1
            self.n2k_unique_number.set(unique_number)
        return unique_number

    def get_iso_name_value(self, address=None):
        if address is None:
            address = int(self.n2k_address.value)
        unique_number = self.claim_unique_number() & 0x1fffff
        manufacturer_code = 78
        device_instance_lower = 0
        device_instance_upper = 0
        device_function = 150
        spare = 0
        device_class = 40
        system_instance = 0
        industry_group = 4
        arbitrary_address_capable = 1

        return (
            unique_number
            | (manufacturer_code << 21)
            | (device_instance_lower << 32)
            | (device_instance_upper << 35)
            | (device_function << 40)
            | (spare << 48)
            | (device_class << 49)
            | (system_instance << 56)
            | (industry_group << 60)
            | (arbitrary_address_capable << 63)
        )

    def get_iso_name_value_from_message(self, message):
        try:
            unique_number = int(message.get_field_by_id('uniqueNumber').raw_value)
            manufacturer_code = int(message.get_field_by_id('manufacturerCode').raw_value)
            device_instance_lower = int(message.get_field_by_id('deviceInstanceLower').raw_value)
            device_instance_upper = int(message.get_field_by_id('deviceInstanceUpper').raw_value)
            device_function = int(message.get_field_by_id('deviceFunction').raw_value)
            spare = int(message.get_field_by_id('spare').raw_value)
            device_class = int(message.get_field_by_id('deviceClass').raw_value)
            system_instance = int(message.get_field_by_id('systemInstance').raw_value)
            industry_group = int(message.get_field_by_id('industryGroup').raw_value)
            arbitrary_address_capable = int(message.get_field_by_id('arbitraryAddressCapable').raw_value)
        except Exception:
            return None

        return (
            unique_number
            | (manufacturer_code << 21)
            | (device_instance_lower << 32)
            | (device_instance_upper << 35)
            | (device_function << 40)
            | (spare << 48)
            | (device_class << 49)
            | (system_instance << 56)
            | (industry_group << 60)
            | (arbitrary_address_capable << 63)
        )

    def build_lau_string(self, value):
        if value is None:
            value = ''
        encoded = str(value).encode('utf-8', errors='ignore')
        return bytes([len(encoded) + 2, 1]) + encoded

    def build_configuration_information_payload(self):
        transport = self.n2k_transport.value
        details = self.n2k_interface.value or self.n2k_host.value or self.n2k_usb_device.value or ''
        install_1 = '%s:%s' % (transport, details) if details else str(transport)
        install_2 = '%s autopilot' % (self.n2k_name.value or 'pypilot')
        manufacturer_information = 'pypilot %s' % version.strversion
        return (
            self.build_lau_string(install_1)
            + self.build_lau_string(install_2)
            + self.build_lau_string(manufacturer_information)
        )

    def build_configuration_information_message(self):
        return nmea2000.NMEA2000Message(
            PGN=CONFIGURATION_INFORMATION_PGN,
            source=self.n2k_address.value,
            destination=255,
            priority=6,
            raw_can_data=self.build_configuration_information_payload())

    def get_transmit_pgns(self):
        pgns = list(BASE_TRANSMIT_PGNS)
        if self.n2k_output_enable['attitude'].value:
            pgns.append(127257)
        if self.n2k_output_enable['heading'].value:
            pgns.append(127250)
        if self.n2k_output_enable['rate_of_turn'].value:
            pgns.append(127251)
        return sorted(set(pgns))


    async def init_transport(self):
        print("N2KBridge: Initializing transport...")
        await self.close_gateway()
        self.transport_config = self.current_transport_config()

        transport = self.n2k_transport.value
        if transport == 'none':
            self.set_status('disabled')
            return

        include_pgns = self.transport_include_pgns()

        device_options = {
            "preferred_address": 100,
            "unique_number": None,
            "manufacturer_code": 999,
            "device_function": 150,
            "device_class": 40,
            "model_id": "PyPilot",
            "model_version": "0.1.0",
            "product_code": None,
            "manufacturer_information": "PyPilot Autopilot",
            "installation_description1": "PyPilot Autopilot",
            "installation_description2": None,
            "transmit_pgns": self.get_transmit_pgns(),
            "address_claim_startup_delay": 0.1,
            "address_claim_detection_time": 0.25,
            "heartbeat_interval": 60.0,
            "persistence_path": None,
            "persistence_key": None,
        }

        transport_classes = {
            'socketcan': (nmea2000.N2KDevice.for_python_can, {"interface": "socketcan", "channel": self.n2k_interface.value or "can0", "client_options": {"include_pgns": include_pgns}, **device_options}),
            # 'actisense': (nmea2000.ActisenseNmea2000Gateway, {}),
            # 'ebyte': (nmea2000.EByteNmea2000Gateway, {}),
            # 'usb': (nmea2000.WaveShareNmea2000Gateway, {}),
        }

        transport_class, transport_kwargs = transport_classes.get(transport, (None, None))

        if not transport_class:
            self.set_status('invalid transport', 'unsupported transport: %s' % transport)
            return

        try:
            self.gateway = transport_class(**transport_kwargs)
            self.gateway.set_recieve_callback(self.parse_pgn)
            await self.gateway.start()
        except Exception as e:
            self.set_status('transport error', str(e))
            return

        self.set_status('connected')

    async def close_gateway(self):
        if self.gateway:
            try:
                await self.gateway.close()
            finally:
                self.gateway = None

    async def ensure_transport(self):
        current = self.current_transport_config()
        if current != self.transport_config:
            await self.init_transport()

    def update_gps_message(self, device, updates):
        gps = self.gps_devices.get(device, {'device': device, 'speed': 0})
        gps.update(updates)
        self.gps_devices[device] = gps
        if not 'lat' in gps or not 'lon' in gps:
            return

        msg = {'device': device, 'lat': gps['lat'], 'lon': gps['lon'], 'speed': gps.get('speed', 0)}
        for name in ['track', 'timestamp', 'declination']:
            if name in gps:
                msg[name] = gps[name]
        self.msgs['gps'] = msg

    def angle_to_degrees(self, value):
        if value is None:
            return None
        value = float(value)
        if abs(value) <= 6.5:
            return value * 180.0 / 3.141592653589793
        return value

    def course_reference_mode(self, reference):
        if reference is None:
            return 'gps'
        reference = str(reference).lower()
        if 'mag' in reference or reference == 'm':
            return 'compass'
        return 'gps'

    def signed_xte(self, value, direction=None):
        if value is None:
            return None
        xte = float(value) / 1852.0
        if direction is None:
            return xte
        direction = str(direction).lower()
        if direction in ['l', 'left', 'port']:
            return -abs(xte)
        if direction in ['r', 'right', 'starboard', 'stbd']:
            return abs(xte)
        return xte

    def update_apb_message(self, device, updates):
        nav = self.nav_devices.get(device, {'device': device})
        nav.update(updates)
        self.nav_devices[device] = nav
        if not 'track' in nav:
            return

        msg = {'device': device, 'track': nav['track']}
        if 'xte' in nav:
            msg['xte'] = nav['xte']
        if 'mode' in nav:
            msg['mode'] = nav['mode']
        self.msgs['apb'] = msg

    def set_client_value(self, name, value, epsilon=None):
        current = self.last_values.get(name)
        if epsilon is not None:
            try:
                if abs(float(current) - float(value)) <= epsilon:
                    return
            except Exception:
                pass
        elif current == value:
            return
        self.client.set(name, value)
        self.last_values[name] = value

    def parse_pgn(self, message: nmea2000.NMEA2000Message):
        device = 'N2K%d' % message.source if message.source is not None else 'N2K'
        self.all_pgns = self.all_pgns if hasattr(self, 'all_pgns') else set()
        self.all_pgns.add(message.PGN)
        print(self.all_pgns)

        if message.PGN == 130306:
            try:
                angle_rad = message.get_field_by_id('windAngle').value
                speed_ms = message.get_field_by_id('windSpeed').value
                ref = message.get_field_by_id('reference').value or 'Apparent'
            except Exception:
                return
            if angle_rad is None or speed_ms is None:
                return
            direction_deg = angle_rad * 180.0 / 3.141592653589793
            speed_knots = speed_ms * 1.94384
            name = 'truewind' if str(ref).lower() in ['true', 't', 'true wind'] else 'wind'
            self.msgs[name] = {'direction': direction_deg, 'speed': speed_knots, 'device': device}
            return

        if message.PGN == 127245:
            try:
                pos_rad = message.get_field_by_id('position').value
            except Exception:
                return
            if pos_rad is None:
                return
            angle_deg = pos_rad * 180.0 / 3.141592653589793
            self.msgs['rudder'] = {'angle': -angle_deg, 'device': device}
            return

        if message.PGN == 129029:
            try:
                lat = message.get_field_by_id('latitude').value
                lon = message.get_field_by_id('longitude').value
            except Exception:
                return
            if lat is None or lon is None:
                return
            updates = {'lat': lat, 'lon': lon}
            try:
                sog_ms = message.get_field_by_id('sog').value
            except Exception:
                sog_ms = None
            try:
                cog_rad = message.get_field_by_id('cog').value
            except Exception:
                cog_rad = None
            try:
                timestamp = message.get_field_by_id('time').value
            except Exception:
                timestamp = None
            if sog_ms is not None:
                updates['speed'] = sog_ms * 1.94384
            if cog_rad is not None:
                updates['track'] = cog_rad * 180.0 / 3.141592653589793
            if timestamp is not None:
                updates['timestamp'] = timestamp
            self.update_gps_message(device, updates)
            return

        if message.PGN == 129026:
            updates = {}
            try:
                sog_ms = message.get_field_by_id('sog').value
            except Exception:
                sog_ms = None
            try:
                cog_rad = message.get_field_by_id('cog').value
            except Exception:
                cog_rad = None
            if sog_ms is not None:
                updates['speed'] = sog_ms * 1.94384
            if cog_rad is not None:
                updates['track'] = cog_rad * 180.0 / 3.141592653589793
            if updates:
                self.update_gps_message(device, updates)
            return

        if message.PGN == 129033:
            try:
                timestamp = message.get_field_by_id('time').value
            except Exception:
                timestamp = None
            if timestamp is not None:
                self.update_gps_message(device, {'timestamp': timestamp})
            return

        if message.PGN == 128259:
            try:
                speed_ms = message.get_field_by_id('speedWaterReferenced').value
            except Exception:
                speed_ms = None
            if speed_ms is None:
                return
            self.msgs['water'] = {'speed': speed_ms * 1.94384, 'device': device}
            return

        if message.PGN == 129283:
            try:
                xte = message.get_field_by_id('xte').value
            except Exception:
                xte = None
            if xte is None:
                return
            try:
                direction = message.get_field_by_id('xteMode').value
            except Exception:
                direction = None
            self.update_apb_message(device, {'xte': self.signed_xte(xte, direction)})
            return

        if message.PGN == 129284:
            try:
                track = message.get_field_by_id('bearingPositionToDestinationWaypoint').value
            except Exception:
                track = None
            if track is None:
                return
            try:
                course_ref = message.get_field_by_id('courseBearingReference').value
            except Exception:
                course_ref = None
            updates = {'track': self.angle_to_degrees(track),
                       'mode': self.course_reference_mode(course_ref)}
            try:
                xte = message.get_field_by_id('xte').value
            except Exception:
                xte = None
            if xte is not None:
                try:
                    direction = message.get_field_by_id('xteMode').value
                except Exception:
                    direction = None
                updates['xte'] = self.signed_xte(xte, direction)
            self.update_apb_message(device, updates)
            return


    async def send_message(self, message: nmea2000.NMEA2000Message):
        if not self.gateway:
            return False
        try:
            await self.gateway.send(message)
            return True
        except Exception as e:
            self.set_status('n2k output error', str(e))
            return False

    def publish_msgs(self):
        if self.msgs and self.pipe.send(self.msgs):
            self.msgs = {}

    async def update_claim_state(self):
        if self.claim_in_progress and time.monotonic() - self.address_claim_sent_at >= ADDRESS_CLAIM_DETECTION_TIME:
            await self.finish_address_claim()

    async def send_heartbeat_if_due(self, now):
        if not self.cansend or not self.next_heartbeat_time or now < self.next_heartbeat_time:
            return

        self.heartbeat_counter = (self.heartbeat_counter + 1) % 253
        await self.send_message(self.build_heartbeat_message())
        self.next_heartbeat_time = now + HEARTBEAT_INTERVAL
    
    async def output_pgns(self):
        if not self.cansend:
            return

        values = self.last_values
        t = time.monotonic()

        await self.send_heartbeat_if_due(t)

        if self.n2k_output_enable['attitude'].value or self.n2k_output_enable['heading'].value or self.n2k_output_enable['rate_of_turn'].value:
            if t - self.last_imu_time > 0.1:
                if self.n2k_output_enable['attitude'].value and 'imu.pitch' in values and 'imu.roll' in values:
                    pitch_rad = values['imu.pitch'] * 3.141592653589793 / 180.0
                    roll_rad = values['imu.roll'] * 3.141592653589793 / 180.0
                    # PGN 127257 requires: sid, yaw, pitch, roll, reserved_56
                    fields = [
                        nmea2000.NMEA2000Field(id='sid', value=self.next_sid()),
                        nmea2000.NMEA2000Field(id='yaw', value=0.0),
                        nmea2000.NMEA2000Field(id='pitch', value=pitch_rad),
                        nmea2000.NMEA2000Field(id='roll', value=roll_rad),
                        nmea2000.NMEA2000Field(id='reserved_56', value=0),
                    ]
                    message = nmea2000.NMEA2000Message(PGN=127257, source=self.n2k_address.value, destination=255, priority=3, fields=fields)
                    await self.send_message(message)

                if self.n2k_output_enable['heading'].value and 'imu.heading_lowpass' in values:
                    heading_rad = values['imu.heading_lowpass'] * 3.141592653589793 / 180.0
                    # Include required PGN 127250 fields: sid, heading, deviation, variation, reference, reserved_58
                    fields = [
                        nmea2000.NMEA2000Field(id='sid', value=self.next_sid()),
                        nmea2000.NMEA2000Field(id='heading', value=heading_rad),
                        nmea2000.NMEA2000Field(id='deviation', value=0.0),
                        nmea2000.NMEA2000Field(id='variation', value=0.0),
                        nmea2000.NMEA2000Field(id='reference', value='Magnetic'),
                        nmea2000.NMEA2000Field(id='reserved_58', value=0),
                    ]
                    message = nmea2000.NMEA2000Message(PGN=127250, source=self.n2k_address.value, destination=255, priority=3, fields=fields)
                    await self.send_message(message)

                if self.n2k_output_enable['rate_of_turn'].value and 'imu.headingrate_lowpass' in values:
                    rot_rad_per_s = values['imu.headingrate_lowpass'] * 3.141592653589793 / 180.0
                    # PGN 127251 requires: sid, rate, reserved_40
                    fields = [
                        nmea2000.NMEA2000Field(id='sid', value=self.next_sid()),
                        nmea2000.NMEA2000Field(id='rate', value=rot_rad_per_s),
                        nmea2000.NMEA2000Field(id='reserved_40', value=0),
                    ]
                    message = nmea2000.NMEA2000Message(PGN=127251, source=self.n2k_address.value, destination=255, priority=3, fields=fields)
                    await self.send_message(message)

                self.last_imu_time = t

    async def poll(self, timeout=100):
        await self.ensure_transport() 

        await self.update_claim_state()

        if self.gateway:
            try:
                data = await asyncio.wait_for(self.gateway.queue.get(), timeout / 1000.0)
                self.parse_pgn(data)
            except asyncio.TimeoutError:
                pass

        self.publish_msgs()

        pypilot_msgs = self.client.receive()
        for name in pypilot_msgs:
            value = pypilot_msgs[name]
            self.last_values[name] = value
            if name == 'gps.filtered.output' or name == 'n2k.output.gps_filtered':
                self.update_gps_fix_watch()

        await self.output_pgns()


    async def n2k_process(self):
        await self.setup()
        while True:
            await self.poll(100)



class N2K(object):
    def __init__(self, sensors):
        self.client = sensors.client
        self.sensors = sensors
        self.n2k_bridge = N2KBridge(self.client.server)
        self.process = self.n2k_bridge.process
        self.pipe = self.n2k_bridge.pipe_out
        self.poller = select.poll()
        self.process_fd = self.pipe.fileno()
        self.poller.register(self.process_fd, select.POLLIN)

    def read_process_pipe(self):
        while True:
            msgs = self.pipe.recv()
            if not msgs:
                return
            if isinstance(msgs, dict):
                for name in msgs:
                    self.sensors.write(name, msgs[name], 'can')

    def poll(self):
        if not self.n2k_bridge.process:
            self.n2k_bridge.poll(0)

        while True:
            events = self.poller.poll(0)
            if not events:
                break
            for fd, flag in events:
                if fd == self.process_fd and flag == select.POLLIN:
                    self.read_process_pipe()
