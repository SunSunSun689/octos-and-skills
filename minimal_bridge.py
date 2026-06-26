#!/usr/bin/env python3
"""Minimal SPEC-compatible bridge for SO101 via LeRobot direct drive.

Replaces the full dora bridge (moveit_arm_node + planner + trajectory_executor + ...)
with a thin HTTP server that:
  - POST /tools/vendor.moveit.arm.move_to_joint_state  → interpolate & write motors
  - POST /tools/vendor.moveit.arm.move_to_named         → home/safe/zero/up
  - POST /tools/vendor.moveit.arm.gripper.set           → gripper open/close
  - POST /tools/get_state                               → read current joint angles
  - GET  /healthz                                       → liveness

arm_skills.py does the MuJoCo IK; this bridge just executes the resulting
joint targets on the real hardware with linear interpolation for safety.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np

# ---------- LeRobot setup ----------
_calib_data = None
_bus = None
_motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _init_bus():
    global _calib_data, _bus
    if _bus is not None:
        return
    calib_paths = [
        "/home/dora/dorobot2/third_party/robodriver/components/legacy/arm_normal_so101_v1/.calibration/SO101-follower.json",
    ]
    for cp in calib_paths:
        if os.path.exists(cp):
            _calib_data = json.loads(open(cp).read())
            break
    if _calib_data is None:
        raise RuntimeError("SO101 calibration file not found")

    sys.path.insert(0, "/home/dora/dorobot2/third_party/lerobot/src")
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode

    motors, calib = {}, {}
    for name in _motor_names + ["gripper"]:
        c = _calib_data[name]
        norm = MotorNormMode.DEGREES if name != "gripper" else MotorNormMode.RANGE_0_100
        motors[name] = Motor(c["id"], "sts3215", norm)
        calib[name] = MotorCalibration(
            id=c["id"], drive_mode=c["drive_mode"],
            homing_offset=c["homing_offset"], range_min=c["range_min"], range_max=c["range_max"],
        )
    # Try common ports — USB enumeration may change across replugs
    for port in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]:
        try:
            _bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
            _bus.connect()
            print(f"[bridge] connected on {port}", flush=True)
            break
        except Exception:
            continue
    else:
        raise RuntimeError("SO101 motors not found on /dev/ttyACM0/1/2")
    print("[bridge] LeRobot connected", flush=True)


# ---------- motor I/O ----------
def _read_joints_deg() -> list[float]:
    return [_bus.read("Present_Position", n) for n in _motor_names]


def _write_joints_deg(targets_deg: list[float]):
    for name, deg in zip(_motor_names, targets_deg):
        _bus.write("Goal_Position", name, deg)


def _gripper_pct(pct: float):
    _bus.write("Goal_Position", "gripper", pct)


# ---------- interpolation ----------
INTERP_STEPS = 100  # 100 steps
INTERP_DT = 0.02     # 20ms per step → 2 seconds total


def _move_interpolated(target_rad: list[float], control_source: str = ""):
    """Read current joints, interpolate to target over ~1 second."""
    target_deg = [r * 180.0 / math.pi for r in target_rad]
    current_deg = _read_joints_deg()
    print(f"[bridge] move {[f'{r:.2f}' for r in target_rad]} (src={control_source})", flush=True)

    for step in range(1, INTERP_STEPS + 1):
        t = step / INTERP_STEPS
        interp = [c + t * (g - c) for c, g in zip(current_deg, target_deg)]
        _write_joints_deg(interp)
        time.sleep(INTERP_DT)
    # Settle: write final position once more, wait for motors to arrive
    _write_joints_deg(target_deg)
    time.sleep(0.5)
    final = _read_joints_deg()
    err = [abs(t - f) for t, f in zip(target_deg, final)]
    print(f"[bridge] move done, max_err={max(err):.1f}°", flush=True)


# ---------- named poses (degrees) ----------
NAMED_POSES = {
    "home": [-10.59, -4.18, 11.87, 67.91, -0.18],
    "safe": [-10.59, -4.18, 11.87, 67.91, -0.18],
    "zero": [0.0, 0.0, 0.0, 0.0, 0.0],
    "up":    [0.0, -30.0, 30.0, 0.0, 0.0],
}


# ---------- HTTP ----------
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # silence access logs

    def _respond(self, code: int, body: dict):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.rstrip("/") == "/healthz":
            self._respond(200, {"status": "ok"})
        else:
            self._respond(404, {})

    def do_POST(self):
        path = self.path.rstrip("/")
        body_len = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(body_len) if body_len else b"{}"
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            self._respond(400, {"error": "invalid JSON"})
            return
        args = payload.get("args", {})

        try:
            if path == "/tools/vendor.moveit.arm.move_to_joint_state":
                joints = args.get("joints", [])
                if len(joints) != 5:
                    self._respond(400, {"error": f"need 5 joints, got {len(joints)}"})
                    return
                _move_interpolated([float(v) for v in joints],
                                   control_source=args.get("control_source", ""))
                self._respond(200, {"status": "ok", "joints": joints})

            elif path == "/tools/vendor.moveit.arm.move_to_named":
                name = args.get("name", "home")
                if name not in NAMED_POSES:
                    self._respond(400, {"error": f"unknown named pose: {name}"})
                    return
                deg = NAMED_POSES[name]
                rad = [d * math.pi / 180.0 for d in deg]
                _move_interpolated(rad, control_source=args.get("control_source", ""))
                self._respond(200, {"status": "ok", "name": name})

            elif path == "/tools/vendor.moveit.arm.gripper.set":
                width = float(args.get("width", 0.06))
                # Map width (meters) to gripper percentage (linear):
                #   width=0.06 → 100% (fully open), width=0.0 → 0% (fully closed)
                pct = max(0.0, min(100.0, width / 0.06 * 100.0))
                _gripper_pct(pct)
                time.sleep(0.3)
                self._respond(200, {"status": "ok", "width": width, "pct": pct})

            elif path == "/tools/vendor.moveit.arm.gripper.open":
                _gripper_pct(100.0)
                time.sleep(0.3)
                self._respond(200, {"status": "ok"})

            elif path == "/tools/vendor.moveit.arm.gripper.close":
                _gripper_pct(0.0)
                time.sleep(0.3)
                self._respond(200, {"status": "ok"})

            elif path == "/tools/get_state":
                deg = _read_joints_deg()
                rad = [d * math.pi / 180.0 for d in deg]
                self._respond(200, {
                    "joint_positions": [round(r, 6) for r in rad],
                    "joint_names": _motor_names,
                })

            elif path == "/tools/robot.estop":
                # Emergency: disable torque
                for name in _motor_names:
                    try:
                        _bus.write("Torque_Enable", name, 0)
                    except Exception:
                        pass
                self._respond(200, {"status": "estop"})

            else:
                self._respond(404, {"error": f"unknown tool: {path}"})

        except Exception as e:
            print(f"[bridge] error: {e}", flush=True)
            self._respond(500, {"error": str(e)})


# ---------- main ----------
def main():
    host = os.environ.get("BRIDGE_HOST", "127.0.0.1")
    port = int(os.environ.get("BRIDGE_PORT", "8768"))

    print(f"[bridge] initializing LeRobot bus...", flush=True)
    _init_bus()
    print(f"[bridge] current joints (deg): {[f'{v:.1f}' for v in _read_joints_deg()]}", flush=True)

    server = HTTPServer((host, port), _Handler)
    print(f"[bridge] listening on http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        print("[bridge] stopped", flush=True)


if __name__ == "__main__":
    main()
