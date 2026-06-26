#!/usr/bin/env python3
"""Hand-eye calibration for eye-in-hand SO101 + Orbbec Gemini 335.

Computes ee_T_camera (camera pose in end-effector frame) using the classic
AX=XB formulation. Uses a 9×6 checkerboard as the calibration target. The
checkerboard stays FIXED on the table; the user moves the arm to N >= 3
distinct poses. At each pose we record:
  - base_T_ee: end-effector pose from FK (joint angles via bridge get_state)
  - cam_T_target: checkerboard pose in camera frame (cv2.solvePnP, full 6-DOF)

From N pose pairs we construct the relative motions A_i and B_i (i=1..N-1):
  A_i = base_T_ee_j^{-1} @ base_T_ee_i    (end-effector motion)
  B_i = cam_T_target_j^{-1} @ cam_T_target_i  (target motion in camera)
  X = ee_T_camera (unknown)

Then opencv's calibrateHandEye solves AX = XB (Tsai method).

Usage:
  python calibrate_hand_eye.py collect \
      --square-size 0.025 --orbbec-sn CP1E542000CJ \
      --bridge http://127.0.0.1:8768 \
      --model /path/to/so101_new_calib.xml \
      --pairs /tmp/so101_hand_eye_pairs.json

  python calibrate_hand_eye.py solve \
      --model /path/to/so101_new_calib.xml \
      --pairs /tmp/so101_hand_eye_pairs.json \
      --out ee_T_camera.npy

  python calibrate_hand_eye.py check \
      --model /path/to/so101_new_calib.xml \
      --ee-t-camera ee_T_camera.npy \
      --pairs /tmp/so101_hand_eye_pairs.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

# ---------- FK (reads end-effector site from MuJoCo model) ----------
_mj_model = None
_mj_data = None
_site_id = None
# This model (so101_new_calib.xml) has 6 joints: 5 arm + 1 gripper, NO free joint.
# Arm joints are qpos[0:5], gripper is qpos[5].
ARM_QPOS_START = 0
NUM_JOINTS = 5
ARM_QPOS = slice(ARM_QPOS_START, ARM_QPOS_START + NUM_JOINTS)


def _load_model(model_path: str) -> None:
    global _mj_model, _mj_data, _site_id
    if _mj_model is not None:
        return
    import mujoco
    _mj_model = mujoco.MjModel.from_xml_path(model_path)
    _mj_data = mujoco.MjData(_mj_model)
    # Try "pinch" (arm_skills convention) then "gripperframe" (calib model convention)
    for site_name in ("pinch", "gripperframe", "attachment"):
        sid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid >= 0:
            _site_id = sid
            print(f"[FK] using site '{site_name}' (id={_site_id})", flush=True)
            break
    if _site_id is None or _site_id < 0:
        raise RuntimeError(f"No known end-effector site found in {model_path}")


def compute_base_T_ee(joint_angles: list[float]) -> np.ndarray:
    """Return 4x4 base_T_ee from joint angles using MuJoCo FK."""
    import mujoco
    _mj_data.qpos[ARM_QPOS] = joint_angles
    mujoco.mj_forward(_mj_model, _mj_data)
    pos = _mj_data.site_xpos[_site_id].copy()
    R = _mj_data.site_xmat[_site_id].reshape(3, 3).copy()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


# ---------- HTTP helpers ----------
def _get_joints(bridge_url: str) -> list[float]:
    """Query bridge get_state for current joint angles."""
    body = json.dumps({"args": {}}).encode()
    req = urllib.request.Request(
        f"{bridge_url}/tools/get_state", data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        resp = json.loads(r.read().decode())
    state = resp.get("output", resp)
    jp = state.get("joint_positions") or state.get("joints") or state.get("position")
    if jp is None:
        raise RuntimeError(f"get_state returned no joint positions: {list(state.keys())[:5]}")
    return [float(v) for v in jp]


# ---------- LeRobot direct hardware read ----------
_lerobot_bus = None


def _get_joints_lerobot(port: str = "/dev/ttyACM0") -> list[float]:
    """Read joint angles directly from SO101 via LeRobot Feetech bus (degrees → radians)."""
    global _lerobot_bus
    import math
    if _lerobot_bus is None:
        calib_paths = [
            "/home/dora/dorobot2/third_party/robodriver/components/legacy/arm_normal_so101_v1/.calibration/SO101-follower.json",
        ]
        calib_data = None
        for cp in calib_paths:
            if os.path.exists(cp):
                calib_data = json.loads(open(cp).read())
                break
        if calib_data is None:
            raise RuntimeError("SO101 calibration file not found")

        # Lazy import so file is parseable without lerobot installed
        sys.path.insert(0, "/home/dora/dorobot2/third_party/lerobot/src")
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode

        motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        motors = {}
        calib = {}
        for name in motor_names:
            c = calib_data[name]
            norm = MotorNormMode.DEGREES if name != "gripper" else MotorNormMode.RANGE_0_100
            motors[name] = Motor(c["id"], "sts3215", norm)
            calib[name] = MotorCalibration(
                id=c["id"], drive_mode=c["drive_mode"],
                homing_offset=c["homing_offset"], range_min=c["range_min"], range_max=c["range_max"],
            )

        _lerobot_bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
        _lerobot_bus.connect()
        print(f"[lerobot] connected to SO101 on {port}", flush=True)

    deg = []
    for name in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]:
        deg.append(_lerobot_bus.read("Present_Position", name))
    return [float(d) * math.pi / 180.0 for d in deg]


def _make_4x4(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).ravel()
    return T


# ---------- Orbbec color stream + checkerboard ----------
def _open_orbbec_color(sn: str | None, warmup_s: float = 1.5):
    """Open Gemini 335 color stream, return (pipeline, camera_matrix, dist_coeffs)."""
    # Lazy import so the file is parseable even without pyorbbecsdk installed
    from pyorbbecsdk import (
        Config, Context, OBError, OBFormat, OBSensorType, Pipeline,
    )

    ctx = Context()
    dl = ctx.query_devices()
    if dl.get_count() == 0:
        raise RuntimeError("no Orbbec devices visible")

    if sn:
        dev = None
        for i in range(dl.get_count()):
            d = dl.get_device_by_index(i)
            if d.get_device_info().get_serial_number() == sn:
                dev = d
                break
        if dev is None:
            raise RuntimeError(f"Orbbec device with SN={sn} not found")
        print(f"[orbbec] picked by SN={sn}", flush=True)
    else:
        dev = dl.get_device_by_index(0)
        info = dev.get_device_info()
        print(f"[orbbec] picked first device: {info.get_name()} SN={info.get_serial_number()}", flush=True)

    pipeline = Pipeline(dev)
    config = Config()

    # Color stream: try 1280×720 MJPG first (better corner detection), fall back to 640×480
    color_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    color_profile = None
    for w, h in [(1280, 720), (640, 480)]:
        for fmt_name in ("MJPG", "YUYV", "RGB", "BGR"):
            if not hasattr(OBFormat, fmt_name):
                continue
            try:
                color_profile = color_list.get_video_stream_profile(
                    w, h, getattr(OBFormat, fmt_name), 30)
                print(f"[orbbec] color stream: {w}x{h} @30 {fmt_name}", flush=True)
                break
            except OBError:
                continue
        if color_profile is not None:
            break
    if color_profile is None:
        color_profile = color_list.get_default_video_stream_profile()
        print(f"[orbbec] color stream: default profile", flush=True)

    config.enable_stream(color_profile)
    pipeline.start(config)
    time.sleep(warmup_s)

    # Get camera intrinsics from SDK
    cam_param = pipeline.get_camera_param()
    intr = cam_param.rgb_intrinsic
    camera_matrix = np.array([
        [intr.fx, 0, intr.cx],
        [0, intr.fy, intr.cy],
        [0, 0, 1],
    ], dtype=np.float64)
    dist_coeffs = np.array([
        cam_param.rgb_distortion.k1, cam_param.rgb_distortion.k2,
        cam_param.rgb_distortion.p1, cam_param.rgb_distortion.p2,
        cam_param.rgb_distortion.k3, cam_param.rgb_distortion.k4,
        cam_param.rgb_distortion.k5, cam_param.rgb_distortion.k6,
    ], dtype=np.float64) if hasattr(cam_param, 'rgb_distortion') else np.zeros(5, dtype=np.float64)

    print(f"[orbbec] intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} "
          f"cx={intr.cx:.1f} cy={intr.cy:.1f} ({intr.width}x{intr.height})", flush=True)
    return pipeline, camera_matrix, dist_coeffs


def _detect_checkerboard(color_bgr: np.ndarray, pattern_size: tuple[int, int],
                         square_size: float, camera_matrix: np.ndarray,
                         dist_coeffs: np.ndarray):
    """Detect checkerboard and return (R, t) via solvePnP, or None if not found.

    Returns:
        (R_3x3, t_3x1) — checkerboard pose in camera frame (cam_T_target)
        or None if detection failed
    """
    cols, rows = pattern_size
    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)

    # Try with adaptive threshold + normalization for robustness
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
    if not found:
        # Retry with histogram equalization for low-contrast scenes
        gray_eq = cv2.equalizeHist(gray)
        found, corners = cv2.findChessboardCorners(gray_eq, (cols, rows), None)

    if not found:
        return None

    # Sub-pixel refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners_refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

    # Object points: checkerboard corners in the board's own frame (z=0 plane)
    objp = np.zeros((cols * rows, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size

    ok, rvec, tvec = cv2.solvePnP(
        objp, corners_refined, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    return R.astype(np.float64), tvec.astype(np.float64).ravel()


def _grab_color_frame(pipeline, color_format) -> np.ndarray | None:
    """Grab one color frame from the pipeline, return BGR ndarray."""
    from pyorbbecsdk import OBFormat

    # Drain stale frames
    for _ in range(3):
        pipeline.wait_for_frames(100)

    frames = pipeline.wait_for_frames(2000)
    if frames is None:
        return None
    color_frame = frames.get_color_frame()
    if color_frame is None:
        return None

    data = np.asanyarray(color_frame.get_data())
    fmt = color_frame.get_format()
    w, h = color_frame.get_width(), color_frame.get_height()

    if fmt == OBFormat.RGB:
        img = np.resize(data, (h, w, 3))
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif fmt == OBFormat.BGR:
        return np.resize(data, (h, w, 3))
    elif fmt == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    elif fmt == OBFormat.YUYV:
        img = np.resize(data, (h, w, 2))
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    elif fmt == OBFormat.UYVY:
        img = np.resize(data, (h, w, 2))
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)
    else:
        print(f"  [warn] unsupported color format: {fmt}", flush=True)
        return None


# ---------- collect (checkerboard version) ----------
def cmd_collect(args: argparse.Namespace) -> int:
    _load_model(args.model)
    pattern_size = (args.pattern_cols, args.pattern_rows)

    # Pre-check bridge (skip in preview mode)
    if not args.preview:
        try:
            urllib.request.urlopen(f"{args.bridge}/healthz", timeout=2)
            print(f"Bridge OK: {args.bridge}/healthz")
        except Exception:
            print(f"WARNING: Bridge not reachable at {args.bridge}/healthz")
            print(f"You can still collect data — enter joint angles manually when prompted.")
            print(f"To start bridge later: cd /opt/octos-dora-bridge && dora up && dora start dataflows/so101-hw-bridge.yaml")
            print()

    print(f"Opening Orbbec color stream...")
    pipeline, camera_matrix, dist_coeffs = _open_orbbec_color(args.orbbec_sn)
    print(f"Checkerboard: {pattern_size[0]}×{pattern_size[1]} internal corners, "
          f"square={args.square_size*1000:.0f}mm")
    if not args.preview:
        print(f"Bridge: {args.bridge}")
    print()
    print(f"Place the checkerboard FIXED on the table within the camera's view.")
    if args.preview:
        print(f"Preview mode: showing checkerboard detection status. Ctrl+C to exit.")
        print()
    else:
        print(f"For each pose: move the arm, press ENTER to capture.")
        print()

    pairs = []
    try:
        while True:
            if not args.preview and len(pairs) >= 3:
                print(f"[{len(pairs)} pairs, minimum met]")

            # Live preview: grab a frame and try detection to give feedback
            color_bgr = _grab_color_frame(pipeline, None)
            if color_bgr is not None:
                result = _detect_checkerboard(
                    color_bgr, pattern_size, args.square_size,
                    camera_matrix, dist_coeffs,
                )
                status = "CHECKERBOARD VISIBLE" if result is not None else "not found"
                h, w = color_bgr.shape[:2]
                print(f"  preview: {w}x{h} — {status}", flush=True)

            if args.preview:
                time.sleep(0.3)
                continue

            cmd = input(f"  Pose {len(pairs)+1} — move arm then press ENTER "
                        f"(or 'done'/'d' to finish, 'del' to undo): ").strip()
            if cmd.lower() in ("done", "d"):
                if len(pairs) < 3:
                    print(f"Need at least 3 pairs, have {len(pairs)}. Keep going.")
                    continue
                break
            if cmd.lower() == "del" and pairs:
                removed = pairs.pop()
                print(f"  Removed pose {len(pairs)+1}")
                continue

            # --- capture joints (try LeRobot hardware → bridge → manual) ---
            joints = None
            lerobot_ok = os.path.exists("/dev/ttyACM0")
            if lerobot_ok:
                try:
                    joints = _get_joints_lerobot()
                    print(f"  joints from LeRobot: {[f'{v:.2f}' for v in joints]}")
                except Exception as e:
                    print(f"  LeRobot read failed ({e}), trying bridge...")
            if joints is None:
                try:
                    joints = _get_joints(args.bridge)
                    print(f"  joints from bridge: {[f'{v:.2f}' for v in joints]}")
                except Exception as e:
                    print(f"  Bridge unavailable ({e})")
                    raw = input(f"  Enter 5 joint angles (radians, space-separated): ").strip()
                    try:
                        joints = [float(x) for x in raw.split()]
                        if len(joints) != 5:
                            print(f"  ERROR: need exactly 5 joint angles, got {len(joints)}")
                            continue
                    except ValueError:
                        print(f"  ERROR: invalid numbers")
                        continue
            base_T_ee = compute_base_T_ee(joints)

            # Grab a fresh frame and detect
            color_bgr = _grab_color_frame(pipeline, None)
            if color_bgr is None:
                print(f"  ERROR: no color frame")
                continue

            detected = _detect_checkerboard(
                color_bgr, pattern_size, args.square_size,
                camera_matrix, dist_coeffs,
            )
            if detected is None:
                debug_path = Path(args.pairs).with_suffix(".debug.jpg")
                cv2.imwrite(str(debug_path), color_bgr)
                print(f"  ERROR: checkerboard not detected.")
                print(f"  Debug image saved to {debug_path}")
                print(f"  Tips: ensure board is well-lit, flat, and fills >30% of frame")
                continue

            R_cam_target, t_cam_target = detected
            cam_T_target = _make_4x4(R_cam_target, t_cam_target)

            pair = {
                "joints": [round(v, 6) for v in joints],
                "base_T_ee": base_T_ee.tolist(),
                "cam_R_target": R_cam_target.tolist(),
                "cam_t_target": t_cam_target.tolist(),
            }
            pairs.append(pair)
            print(f"  OK — joints={[f'{v:.2f}' for v in joints]}  "
                  f"t_cam=({t_cam_target[0]:.3f}, {t_cam_target[1]:.3f}, {t_cam_target[2]:.3f})m")

    finally:
        pipeline.stop()

    out = Path(args.pairs)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(pairs, indent=2))
    print(f"\nSaved {len(pairs)} pairs to {out}")
    print(f"Next: python calibrate_hand_eye.py solve --model {args.model} --pairs {out} --out ee_T_camera.npy")
    return 0


# ---------- solve ----------
def cmd_solve(args: argparse.Namespace) -> int:
    _load_model(args.model)

    pairs = json.loads(Path(args.pairs).read_text())
    if len(pairs) < 3:
        print(f"ERROR: need at least 3 pairs, got {len(pairs)}", file=sys.stderr)
        return 1

    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []

    for p in pairs:
        T_be = compute_base_T_ee(p["joints"])
        R_gripper2base.append(T_be[:3, :3])
        t_gripper2base.append(T_be[:3, 3])

        # cam_T_target: use full rotation from checkerboard (solvePnP)
        R_ct = np.array(p.get("cam_R_target", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]), dtype=np.float64)
        t_ct = np.array(p.get("cam_t_target", p.get("cam_xyz", [0, 0, 0])), dtype=np.float64).ravel()
        R_target2cam.append(R_ct)
        t_target2cam.append(t_ct)

    try:
        R_ee2cam, t_ee2cam = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )
    except cv2.error as e:
        print(f"ERROR: opencv calibrateHandEye failed: {e}", file=sys.stderr)
        print("Check that your poses have sufficient translation + rotation diversity.", file=sys.stderr)
        return 1

    ee_T_camera = _make_4x4(R_ee2cam, t_ee2cam)

    # Compute residuals: project checkerboard origin to base frame via each pose
    residuals = []
    if len(pairs) > 1:
        T0 = compute_base_T_ee(pairs[0]["joints"])
        R_ct0 = np.array(pairs[0].get("cam_R_target", np.eye(3)), dtype=np.float64)
        t_ct0 = np.array(pairs[0].get("cam_t_target", pairs[0].get("cam_xyz", [0, 0, 0])), dtype=np.float64).ravel()
        base_target_0 = (T0 @ ee_T_camera @ _make_4x4(R_ct0, t_ct0))[:3, 3]
        for i in range(1, len(pairs)):
            Ti = compute_base_T_ee(pairs[i]["joints"])
            R_i = np.array(pairs[i].get("cam_R_target", np.eye(3)), dtype=np.float64)
            t_i = np.array(pairs[i].get("cam_t_target", pairs[i].get("cam_xyz", [0, 0, 0])), dtype=np.float64).ravel()
            base_target_i = (Ti @ ee_T_camera @ _make_4x4(R_i, t_i))[:3, 3]
            resid = np.linalg.norm(base_target_i - base_target_0)
            residuals.append(resid)

    print(f"ee_T_camera =")
    print(np.array2string(ee_T_camera, precision=6, suppress_small=True))
    if residuals:
        print(f"\nCheckerboard reprojection residuals (mm):")
        for i, r in enumerate(residuals):
            print(f"  pose 0 → {i+1}: {r*1000:.1f} mm")
        print(f"  mean: {np.mean(residuals)*1000:.1f} mm  max: {np.max(residuals)*1000:.1f} mm")

    out = Path(args.out)
    np.save(str(out), ee_T_camera)
    print(f"\nSaved to {out}")
    print(f"Set: export EE_T_CAMERA_PATH={out.resolve()}")
    print(f"     export HAND_EYE_MODE=eye_in_hand")
    return 0


# ---------- check ----------
def cmd_check(args: argparse.Namespace) -> int:
    _load_model(args.model)
    ee_T_camera = np.load(args.ee_t_camera)

    pairs = json.loads(Path(args.pairs).read_text())
    base_targets = []
    for p in pairs:
        T_be = compute_base_T_ee(p["joints"])
        R_ct = np.array(p.get("cam_R_target", np.eye(3)), dtype=np.float64)
        t_ct = np.array(p.get("cam_t_target", p.get("cam_xyz", [0, 0, 0])), dtype=np.float64).ravel()
        cam_T_target = _make_4x4(R_ct, t_ct)
        base_pt = T_be @ ee_T_camera @ cam_T_target @ np.array([0, 0, 0, 1.0], dtype=np.float64)
        base_targets.append(base_pt[:3])

    base_targets = np.array(base_targets)
    mean = base_targets.mean(axis=0)
    std = base_targets.std(axis=0)
    dists = np.linalg.norm(base_targets - mean, axis=1)

    print(f"Checkerboard base position from {len(pairs)} poses:")
    print(f"  mean: ({mean[0]:.4f}, {mean[1]:.4f}, {mean[2]:.4f}) m")
    print(f"  std:  ({std[0]:.4f}, {std[1]:.4f}, {std[2]:.4f}) m")
    print(f"  max deviation: {dists.max()*1000:.1f} mm")
    if dists.max() > 0.02:
        print(f"  WARNING: max deviation > 20 mm — calibration may be poor.")
        print(f"  Recollect with more diverse orientations (rotate wrist ≥30° between poses).")
    else:
        print(f"  Good — sub-2cm consistency across poses.")
    return 0


# ---------- CLI ----------
def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SO101 + Orbbec hand-eye calibration (AX=XB, checkerboard)")
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("collect", help="Collect (joints, checkerboard pose) pairs via Orbbec color stream")
    c.add_argument("--square-size", type=float, required=True,
                   help="Checkerboard square size in meters (e.g. 0.025 for 25mm)")
    c.add_argument("--pattern-cols", type=int, default=9,
                   help="Internal corners columns (default 9)")
    c.add_argument("--pattern-rows", type=int, default=6,
                   help="Internal corners rows (default 6)")
    c.add_argument("--orbbec-sn", default=None,
                   help="Orbbec serial number (default: first device found)")
    c.add_argument("--bridge", default="http://127.0.0.1:8768")
    c.add_argument("--model", required=True,
                   help="MuJoCo XML (e.g. so101_new_calib.xml)")
    c.add_argument("--pairs", default="/tmp/so101_hand_eye_pairs.json")
    c.add_argument("--preview", action="store_true",
                   help="Live preview only: show checkerboard detection status, no data collection")

    s = sub.add_parser("solve", help="Solve AX=XB -> ee_T_camera")
    s.add_argument("--model", required=True)
    s.add_argument("--pairs", required=True)
    s.add_argument("--out", default="ee_T_camera.npy")

    k = sub.add_parser("check", help="Check calibration quality")
    k.add_argument("--model", required=True)
    k.add_argument("--ee-t-camera", required=True)
    k.add_argument("--pairs", required=True)

    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    if args.cmd == "collect":
        sys.exit(cmd_collect(args))
    elif args.cmd == "solve":
        sys.exit(cmd_solve(args))
    elif args.cmd == "check":
        sys.exit(cmd_check(args))
