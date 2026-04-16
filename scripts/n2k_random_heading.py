#!/usr/bin/env python3

import argparse
import asyncio
import math
import random

import nmea2000


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Register an autopilot NMEA2000 device and send random heading at 10 Hz."
    )
    parser.add_argument("--channel", default="can0", help="SocketCAN channel to use")
    parser.add_argument("--address", type=int, default=100, help="Preferred source address")
    parser.add_argument("--unique-number", type=int, default=None, help="Unique number for address claim")
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between heading updates")
    return parser


def build_heading_message(sid: int, heading_rad: float) -> nmea2000.NMEA2000Message:
    return nmea2000.NMEA2000Message(
        PGN=127250,
        source=0,
        destination=255,
        priority=3,
        fields=[
            nmea2000.NMEA2000Field(id="sid", value=sid),
            nmea2000.NMEA2000Field(id="heading", value=heading_rad),
            nmea2000.NMEA2000Field(id="deviation", value=0.0),
            nmea2000.NMEA2000Field(id="variation", value=0.0),
            nmea2000.NMEA2000Field(id="reference", value="Magnetic"),
            nmea2000.NMEA2000Field(id="reserved_58", value=0),
        ],
    )


async def send_random_heading(device: nmea2000.N2KDevice, interval: float) -> None:
    sid = 0
    while True:
        try:
            heading_deg = random.uniform(0.0, 359.9)
            heading_rad = math.radians(heading_deg)
            await device.send(build_heading_message(sid, heading_rad))
            sid = (sid + 1) % 256
            await asyncio.sleep(interval)
        except Exception as e:
            print(f"Error sending message: {e}")


async def main() -> None:
    args = build_parser().parse_args()

    device = nmea2000.N2KDevice.for_python_can(
        "socketcan",
        args.channel,
        preferred_address=args.address,
        unique_number=args.unique_number,
        device_function=150,
        device_class=40,
        manufacturer_information="pypilot random heading simulator",
        installation_description1="pypilot autopilot simulator",
        model_id="PyPilot Heading Sim",
        model_version="1.0",
        transmit_pgns=[127250],
        address_claim_startup_delay=0.1,
        address_claim_detection_time=0.25,
        heartbeat_interval=60.0,
        client_options={"include_pgns": [127250]},
    )

    try:
        await device.start()
        await device.wait_ready(timeout=5)
        print(f"claimed address {device.address} on {args.channel}")
        await send_random_heading(device, args.interval)
    finally:
        await device.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass