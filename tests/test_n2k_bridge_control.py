"""
Smoke test of N2KBridge's wiring of N2KControl: settings registration,
PGN filters, library hooks and sending, without a CAN bus or pypilot server.
"""
import asyncio

import pytest

nmea2000 = pytest.importorskip('nmea2000')

import n2k
import n2k_control as nc
from test_n2k_control import command_payload, encode, switch_message, Source


class FakeClient:
    def __init__(self):
        self.registered = {}
        self.sets = []
        self.watches = {}
        self.received = {}

    def register(self, value):
        self.registered[value.name] = value
        value.client = self
        return value

    def set(self, name, value):
        self.sets.append((name, value))

    def send(self, msg):
        pass

    def watch(self, name, value=True):
        self.watches[name] = value

    def receive(self):
        received, self.received = self.received, {}
        return received


class FakeGateway:
    ready = True
    own_name = 0x1234

    def __init__(self):
        self.sent = []
        self.group_function_handler = None
        self.iso_request_handler = None

    def set_group_function_handler(self, handler):
        self.group_function_handler = handler

    def set_iso_request_handler(self, handler):
        self.iso_request_handler = handler

    async def send(self, message):
        encode(message)  # everything the bridge sends must be encodable
        self.sent.append(message)


def make_bridge(gateway):
    bridge = n2k.N2KBridge.__new__(n2k.N2KBridge)
    bridge.client = FakeClient()
    asyncio.run(bridge.setup())  # transport defaults to none, so no gateway yet
    bridge.gateway = gateway
    bridge.attach_control()
    return bridge


def test_settings_watches_and_filters():
    bridge = make_bridge(FakeGateway())
    registered = bridge.client.registered
    for name in ['n2k.control', 'n2k.switch.bank', 'n2k.control.allowed',
                 'n2k.control.jog_pulse', 'n2k.installation_description']:
        assert name in registered
    assert registered['n2k.control'].value == 'monitor'
    for name in nc.WATCHES:
        assert bridge.client.watches[name] is True
    assert {126208, 127502, 126984, 59904} <= set(bridge.transport_include_pgns())
    assert 127237 in bridge.get_transmit_pgns()
    assert 127501 not in bridge.get_transmit_pgns()  # no switch bank until configured

    registered['n2k.control'].value = 'off'
    assert 126208 not in bridge.transport_include_pgns()


def test_hooks_answer_commands():
    gateway = FakeGateway()
    bridge = make_bridge(gateway)
    bridge.client.registered['n2k.control'].value = 'steer'
    bridge.last_values.update({'ap.modes': ['compass'], 'ap.heading': 10.0})
    handled = asyncio.run(gateway.group_function_handler(
        Source(), command_payload(127237, [(5, bytes([4]))])))
    assert handled
    assert ('ap.enabled', True) in bridge.client.sets
    assert [m.PGN for m in gateway.sent if m.PGN == 126208] == [126208]
    assert asyncio.run(gateway.iso_request_handler(Source(), 127237))
    assert not asyncio.run(gateway.iso_request_handler(Source(), 130306))


def test_switch_message_and_periodic_output():
    gateway = FakeGateway()
    bridge = make_bridge(gateway)
    bridge.client.registered['n2k.control'].value = 'steer'
    bridge.client.registered['n2k.switch.bank'].value = 3
    # a new bank changes the advertised PGNs, which normally restarts the transport
    assert bridge.current_transport_config() != bridge.transport_config
    bridge.transport_config = bridge.current_transport_config()
    bridge.last_values.update({'ap.enabled': True})
    asyncio.run(bridge.parse_pgn(switch_message(instance=3, ch1=nc.SWITCH_ON)))
    assert ('ap.enabled', False) in bridge.client.sets

    bridge.client.received = {'ap.mode': 'compass'}
    asyncio.run(bridge.poll(0))
    pgns = {m.PGN for m in gateway.sent}
    assert {127237, 127501} <= pgns


def test_missing_library_hooks_are_reported(capsys):
    class OldGateway(FakeGateway):
        set_group_function_handler = None
        set_iso_request_handler = None
    make_bridge(OldGateway())
    assert 'nmea2000 library has no' in capsys.readouterr().out
