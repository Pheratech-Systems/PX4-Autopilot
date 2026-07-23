#!/usr/bin/env python3
"""
Trigger a payload drop on a PX4 drone via MAV_CMD_DO_GRIPPER, using pymavlink.

This is the scripted counterpart to a QGC "Release payload" mission item: it
sends the same MAV_CMD_DO_GRIPPER command that the payload_deliverer module
turns into a `gripper` uORB RELEASE, which the gz_bridge shim converts into the
DetachableJoint detach message (see the payload-dropper guide).

Run it AFTER the drone is airborne (e.g. once multi_drone_takeoff_pymavlink.py
has taken off), otherwise you just drop the box on the launch pad.

Usage:
    python3 payload_release_pymavlink.py [--drone-id 0] [--instance 1] [--grab]

Requires:
    pip install pymavlink

Port layout (from px4-rc.mavlink):
    PX4 sends heartbeats TO  14540+i  <- we listen here  (udpin:0.0.0.0:14540+i)
    PX4 listens for commands ON 14580+i <- we send here once connected
"""

import argparse
import time
from pymavlink import mavutil

# MAV_CMD_DO_GRIPPER param2 (gripper action)
GRIPPER_ACTION_RELEASE = 0
GRIPPER_ACTION_GRAB = 1


def connect(drone_id: int, timeout: float = 30.0) -> mavutil.mavudp:
    """Bind to 14540+i, wait for PX4's heartbeat, return the connection."""
    port = 14540 + drone_id
    # udpin: binds our socket to this port so we receive PX4's output stream.
    # After the first heartbeat arrives, pymavlink knows PX4's source address
    # (127.0.0.1:14580+i) and will route our sends back there.
    conn = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{port}",
        source_system=255,
    )
    print(f"[{drone_id}] listening on udp port {port}, waiting for heartbeat...")
    msg = conn.wait_heartbeat(timeout=timeout)
    if msg is None:
        raise TimeoutError(f"[{drone_id}] no heartbeat after {timeout}s on port {port}")
    print(f"[{drone_id}] heartbeat from system {conn.target_system} component {conn.target_component}")
    return conn


def send_command_long(conn, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0) -> None:
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        command, 0,
        p1, p2, p3, p4, p5, p6, p7,
    )


def wait_ack(conn, command, timeout: float = 10.0) -> bool:
    # payload_deliverer replies IN_PROGRESS first, then ACCEPTED once the gripper
    # actuation is acknowledged (after PD_GRIPPER_TO), so keep waiting past IN_PROGRESS.
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
        if msg and msg.command == command:
            if msg.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                print("  ACK in progress, waiting for final result...")
                continue
            ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            if not ok:
                print(f"  ACK result={msg.result} (not accepted)")
            return ok
    return False


def release_payload(drone_id: int, instance: int, action: int) -> None:
    conn = connect(drone_id)

    verb = "grabbing" if action == GRIPPER_ACTION_GRAB else "releasing"
    print(f"[{drone_id}] {verb} payload (gripper instance {instance})...")
    send_command_long(
        conn,
        mavutil.mavlink.MAV_CMD_DO_GRIPPER,
        p1=instance,  # gripper instance number
        p2=action,    # 0 = release, 1 = grab
    )
    if wait_ack(conn, mavutil.mavlink.MAV_CMD_DO_GRIPPER):
        print(f"[{drone_id}] gripper command accepted")
    else:
        print(f"[{drone_id}] gripper ACK not received "
              "(check payload_deliverer is running and PD_GRIPPER_EN=1)")


def main() -> None:
    parser = argparse.ArgumentParser(description="Trigger a PX4 payload drop via MAV_CMD_DO_GRIPPER.")
    parser.add_argument("--drone-id", type=int, default=0,
                        help="drone index; connects on udp 14540+id (default: 0)")
    parser.add_argument("--instance", type=int, default=1,
                        help="gripper instance number (default: 1)")
    parser.add_argument("--grab", action="store_true",
                        help="send GRAB instead of RELEASE")
    args = parser.parse_args()

    action = GRIPPER_ACTION_GRAB if args.grab else GRIPPER_ACTION_RELEASE
    release_payload(args.drone_id, args.instance, action)


if __name__ == "__main__":
    main()
