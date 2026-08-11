#!/usr/bin/env python3
"""Stream PX4 LOCAL_POSITION_NED to PlotJuggler over UDP JSON.

Usage:
  1. PlotJuggler → Streaming → UDP Server → gear:
       Address 0.0.0.0, Port 9870, Protocol: JSON → OK → Start
  2. python3 Tools/plotjuggler_local_position.py [/dev/ttyACM0]

Plot: local_position/x y z vx vy vz
"""
from __future__ import annotations

import json
import socket
import sys
import time

from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
UDP_HOST = "127.0.0.1"
UDP_PORT = 9870


def main() -> None:
    print(f"Connecting to {PORT} ...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=57600)
    m.wait_heartbeat(timeout=15)
    print(
        f"heartbeat ok (sys {m.target_system} comp {m.target_component})",
        flush=True,
    )

    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        20000,  # us → 50 Hz
        0,
        0,
        0,
        0,
        0,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Streaming JSON → {UDP_HOST}:{UDP_PORT}", flush=True)
    print("PlotJuggler Protocol must be: JSON", flush=True)

    n = 0
    t0 = time.time()
    while True:
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
        if msg is None:
            print("waiting for LOCAL_POSITION_NED...", flush=True)
            continue

        payload = {
            "timestamp": time.time() - t0,
            "local_position": {
                "x": float(msg.x),
                "y": float(msg.y),
                "z": float(msg.z),
                "vx": float(msg.vx),
                "vy": float(msg.vy),
                "vz": float(msg.vz),
            },
        }
        sock.sendto(json.dumps(payload).encode("utf-8"), (UDP_HOST, UDP_PORT))
        n += 1
        if n % 50 == 0:
            print(
                f"x={msg.x:+.3f} y={msg.y:+.3f} z={msg.z:+.3f}  "
                f"vx={msg.vx:+.3f} vy={msg.vy:+.3f} vz={msg.vz:+.3f}",
                flush=True,
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
