#!/usr/bin/env python3
"""Live view of every distance_sensor, labelled by CAN node instead of uORB instance.

uORB instance numbers are handed out in whatever order the CAN nodes start publishing,
so they change between boots. Node IDs do not, so the columns stay put.

Wave your hand in front of one sensor at a time: the column that follows your hand is
that sensor. The MOVING marker flags whichever one changed most in the last second.

Usage:
  python3 Tools/identify_distance_sensors.py [/dev/ttyACM0]
"""
from __future__ import annotations

import re
import sys
import time
from collections import defaultdict, deque

from pymavlink import mavutil

# pymavlink crashes in add_message() on messages carrying an instance field
# (DISTANCE_SENSOR has one) when _instances is None. Last message per type is enough.
mavutil.add_message = lambda messages, mtype, msg: messages.__setitem__(mtype, msg)

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
MAX_INSTANCES = 4
MOVEMENT_THRESHOLD_M = 0.05

SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
SHELL_FLAGS = (mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
               mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE)


def shell_command(m, command: str, quiet_s: float = 0.4, max_s: float = 3.0) -> str:
    data = command.encode() + b"\n"
    m.mav.serial_control_send(SHELL, SHELL_FLAGS, 0, 0, len(data), bytes(data) + bytes(70 - len(data)))

    out = bytearray()
    start = last = time.time()

    while time.time() - start < max_s:
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)

        if msg is not None and msg.count > 0:
            out += bytes(msg.data[:msg.count])
            last = time.time()

        elif time.time() - last > quiet_s:
            break

    return out.decode("utf-8", errors="replace")


def discover_nodes(m) -> dict[int, int]:
    """Map uORB instance -> DroneCAN node ID, read once at startup."""
    nodes = {}

    for instance in range(MAX_INSTANCES):
        text = shell_command(m, f"listener distance_sensor -i {instance} -n 1")
        match = re.search(r"device_id:\s*(\d+)", text)

        if match:
            # PX4 device ID packs the DroneCAN node ID into the address byte
            nodes[instance] = (int(match.group(1)) >> 8) & 0xFF

    return nodes


def main() -> None:
    print(f"Connecting to {PORT} ...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=2000000)
    m.wait_heartbeat(timeout=15)
    print(f"heartbeat ok (sys {m.target_system})", flush=True)

    downward_node = 0
    text = shell_command(m, "param show UAVCAN_RNG_DNID")
    match = re.search(r"UAVCAN_RNG_DNID.*:\s*(\d+)", text)

    if match:
        downward_node = int(match.group(1))

    print("Discovering CAN nodes ...", flush=True)
    nodes = discover_nodes(m)

    if not nodes:
        print("No distance sensors found.", flush=True)
        return

    for instance, node in sorted(nodes.items()):
        role = "  <-- configured as DOWNWARD" if node == downward_node else ""
        print(f"  uORB instance {instance} = node {node}{role}", flush=True)

    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        100000,  # us -> 10 Hz per instance
        0, 0, 0, 0, 0,
    )

    latest: dict[int, tuple[float, int]] = {}
    history: dict[int, deque] = defaultdict(lambda: deque(maxlen=10))
    last_print = 0.0

    print("\nWave your hand in front of ONE sensor at a time. Ctrl-C to stop.\n", flush=True)

    while True:
        msg = m.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=2.0)

        if msg is not None:
            distance_m = msg.current_distance / 100.0
            latest[msg.id] = (distance_m, msg.signal_quality)
            history[msg.id].append(distance_m)

        now = time.time()

        if now - last_print < 0.25 or not latest:
            continue

        last_print = now

        spreads = {i: (max(h) - min(h)) for i, h in history.items() if len(h) > 2}
        mover = max(spreads, key=spreads.get) if spreads else None

        if mover is not None and spreads[mover] < MOVEMENT_THRESHOLD_M:
            mover = None

        cells = []

        for instance in sorted(latest):
            distance_m, quality = latest[instance]
            node = nodes.get(instance, instance)
            label = f"node{node}"

            if node == downward_node:
                label += "(DOWN)"

            if quality <= 1:
                cells.append(f"{label}: NO RETURN ")

            else:
                marker = " <-- MOVING" if instance == mover else ""
                cells.append(f"{label}: {distance_m:6.3f}m{marker}")

        print("   ".join(cells).ljust(110), end="\r", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
