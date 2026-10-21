"""The heading simulator must yield on failures and use only valid SIDs."""
import asyncio
from unittest.mock import AsyncMock

import pytest

pytest.importorskip('nmea2000')
from scripts import n2k_random_heading as heading


@pytest.mark.parametrize('fail_send', [False, True])
def test_heading_loop_always_sleeps(monkeypatch, fail_send):
    device = AsyncMock()
    if fail_send:
        device.send.side_effect = [RuntimeError('send failed'), asyncio.CancelledError]
    sleep = AsyncMock(side_effect=asyncio.CancelledError)
    monkeypatch.setattr(heading.asyncio, 'sleep', sleep)
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(heading.send_random_heading(device, .25))
    device.send.assert_awaited_once()
    sleep.assert_awaited_once_with(.25)


def test_heading_sid_wraps_before_reserved_values(monkeypatch):
    device = AsyncMock()
    sids = []

    def build(sid, radians):
        sids.append(sid)
        return object()

    async def sleep(interval):
        if len(sids) == 255:
            raise asyncio.CancelledError

    monkeypatch.setattr(heading, 'build_heading_message', build)
    monkeypatch.setattr(heading.asyncio, 'sleep', sleep)
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(heading.send_random_heading(device, .1))
    assert sids == list(range(253)) + [0, 1]
