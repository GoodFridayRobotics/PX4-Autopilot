#!/usr/bin/env python3
"""Stream an optical flow bench-test dashboard to PlotJuggler over UDP JSON.

Collects every flow-related signal MAVLink exposes into one aligned snapshot so
sensor health, flow scaling and the estimator output can be watched together
while the fixture is moved by hand.

PlotJuggler exposes no stream for vehicle_optical_flow_vel, so body velocity is
recomputed here exactly as VehicleOpticalFlow.cpp does it:
    vel_body[0] = -range * (flow_y - gyro_y) / dt
    vel_body[1] =  range * (flow_x - gyro_x) / dt

Usage:
  1. PlotJuggler -> Streaming -> UDP Server -> gear:
       Address 0.0.0.0, Port 9870, Protocol: JSON, "use field as timestamp": t
     -> OK -> Start
  2. python3 Tools/plotjuggler_flow_check.py [/dev/ttyACM0]

Series worth plotting:
  flow/quality          flow camera health, 0-255
  flow/distance         AGL the flow module is scaling by, -1 means no return
  flow/vel_body_x|y     flow-implied body velocity, the key scaling check
  flow/comp_x|y         gyro-compensated flow, should sit at 0 when rotating
  dist_<node>/m         each rangefinder, labelled by CAN node
  pos/x|y|z vx|vy|vz    EKF2 output
  est/*_ratio           innovation test ratios, must stay below 1
"""
from __future__ import annotations

import json
import math
import re
import socket
import sys
import time

from pymavlink import mavutil

# pymavlink's add_message() dereferences _instances for messages carrying an
# instance field (DISTANCE_SENSOR has one) and trips over None. Keeping only the
# newest message per type is all this tool needs.
mavutil.add_message = lambda messages, mtype, msg: messages.__setitem__(mtype, msg)

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
UDP_HOST = "127.0.0.1"
UDP_PORT = 9870
SEND_HZ = 50.0

SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
SHELL_FLAGS = (mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
               mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE)

# message id -> rate in Hz
WANTED = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 20.0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 50.0,
    mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: 50.0,
    mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 20.0,
    mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS: 10.0,
}


def finite_only(group: dict) -> dict:
    """Drop non-finite values. json.dumps writes a bare NaN, which is not valid
    JSON, and PlotJuggler discards the whole packet rather than the one field."""
    return {k: v for k, v in group.items()
            if not (isinstance(v, float) and not math.isfinite(v))}


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


def discover_nodes(m, max_instances: int = 4) -> dict[int, int]:
    """Map uORB instance -> DroneCAN node ID so series names survive a reboot."""
    nodes = {}

    for instance in range(max_instances):
        text = shell_command(m, f"listener distance_sensor -i {instance} -n 1")
        match = re.search(r"device_id:\s*(\d+)", text)

        if match:
            nodes[instance] = (int(match.group(1)) >> 8) & 0xFF

    return nodes


def main() -> None:
    print(f"Connecting to {PORT} ...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=2000000)
    m.wait_heartbeat(timeout=15)
    print(f"heartbeat ok (sys {m.target_system})", flush=True)

    downward_node = 0
    match = re.search(r"UAVCAN_RNG_DNID.*:\s*(\d+)", shell_command(m, "param show UAVCAN_RNG_DNID"))

    if match:
        downward_node = int(match.group(1))

    nodes = discover_nodes(m)

    for instance, node in sorted(nodes.items()):
        role = "  <-- DOWNWARD" if node == downward_node else ""
        print(f"  distance_sensor instance {instance} = node {node}{role}", flush=True)

    for msg_id, rate_hz in WANTED.items():
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1e6 / rate_hz),
            0, 0, 0, 0, 0,
        )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Streaming JSON to {UDP_HOST}:{UDP_PORT} (PlotJuggler protocol must be JSON)", flush=True)

    snapshot: dict[str, object] = {}
    t0 = time.time()
    next_send = 0.0
    seen: set[str] = set()

    while True:
        msg = m.recv_match(blocking=True, timeout=2.0)

        if msg is not None:
            kind = msg.get_type()

            if kind not in seen and kind in (
                    "ATTITUDE", "LOCAL_POSITION_NED", "OPTICAL_FLOW_RAD",
                    "DISTANCE_SENSOR", "ESTIMATOR_STATUS"):
                seen.add(kind)
                print(f"  receiving {kind}", flush=True)

            if kind == "OPTICAL_FLOW_RAD":
                dt = msg.integration_time_us / 1e6
                # the stream substitutes -1 when the module has no valid range
                distance = msg.distance if msg.distance >= 0 else float("nan")
                comp_x = msg.integrated_x - msg.integrated_xgyro
                comp_y = msg.integrated_y - msg.integrated_ygyro

                flow = {
                    "quality": msg.quality,
                    "distance": distance,
                    "raw_x": msg.integrated_x,
                    "raw_y": msg.integrated_y,
                    "gyro_x": msg.integrated_xgyro,
                    "gyro_y": msg.integrated_ygyro,
                    "comp_x": comp_x,
                    "comp_y": comp_y,
                }

                if dt > 0 and distance == distance:
                    flow["vel_body_x"] = -distance * comp_y / dt
                    flow["vel_body_y"] = distance * comp_x / dt

                snapshot["flow"] = flow

            elif kind == "DISTANCE_SENSOR":
                node = nodes.get(msg.id, msg.id)
                label = f"dist_node{node}"

                if node == downward_node:
                    label += "_DOWN"

                snapshot[label] = {
                    "m": msg.current_distance / 100.0,
                    "quality": msg.signal_quality,
                    "orientation": msg.orientation,
                }

            elif kind == "LOCAL_POSITION_NED":
                snapshot["pos"] = {
                    "x": msg.x, "y": msg.y, "z": msg.z,
                    "vx": msg.vx, "vy": msg.vy, "vz": msg.vz,
                }

            elif kind == "ATTITUDE":
                snapshot["att"] = {
                    "roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw,
                    "rollspeed": msg.rollspeed, "pitchspeed": msg.pitchspeed,
                }

            elif kind == "ESTIMATOR_STATUS":
                snapshot["est"] = {
                    "vel_ratio": msg.vel_ratio,
                    "pos_horiz_ratio": msg.pos_horiz_ratio,
                    "pos_vert_ratio": msg.pos_vert_ratio,
                    "hagl_ratio": msg.hagl_ratio,
                    "pos_horiz_accuracy": msg.pos_horiz_accuracy,
                    "flags": msg.flags,
                }

        now = time.time()

        if now >= next_send and snapshot:
            next_send = now + 1.0 / SEND_HZ
            payload = {k: finite_only(v) for k, v in snapshot.items()}
            payload["t"] = now - t0
            sock.sendto(json.dumps(payload).encode("utf-8"), (UDP_HOST, UDP_PORT))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
