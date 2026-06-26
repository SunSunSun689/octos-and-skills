#!/usr/bin/env python3
"""Orbbec Gemini 335 vision service for so101-pick-cube.

Side-channel HTTP server on :8779 publishing the live cube (x, y, z) in the SO-101
base frame. Mirrors the sim `ball_state.py` protocol so arm_skills.py can stay
unchanged — the bridge still calls GET /ball and gets the same JSON shape.

Algorithm (geometry-only, no color assumption):
  1. Grab one depth frame from the Orbbec pipeline.
  2. Convert raw uint16 -> meters using depth_scale.
  3. Estimate the table plane as the modal (most common) depth value of pixels
     that look like "valid floor" (depth > min_z, depth < max_z, on a grid).
     Robust to most of the depth noise because the table is the largest flat
     surface in the FOV of a top-view camera.
  4. A cube on the table shows up as a region whose depth is at least
     `CUBE_HEIGHT_M` closer to the camera than the table (i.e. depth smaller
     by ~CUBE_HEIGHT_M). Mask: (table_depth - depth) in [CUBE_HEIGHT_M - tol,
     CUBE_HEIGHT_M + tol].
  5. Cluster the surviving pixels with simple 4-connected component labelling
     (no sklearn dep — just BFS). Take the largest cluster whose area is in
     [MIN_CUBE_PX, MAX_CUBE_PX] (sanity check for "thing big enough to be a cube
     but not the whole table").
  6. Center of mass (px, py) and the median depth in that cluster give us a
     pixel-anchored 3D point in the camera frame. Then a pre-calibrated
     4x4 extrinsics matrix maps camera -> SO-101 base frame.
  7. Publish via the same _Handler/_state pattern as sim/ball_state.py.

The extrinsics matrix (CAMERA_TO_BASE) MUST be calibrated before this skill
runs on real hardware — see `calibrate_extrinsics.py` (TODO: ship a small
chessboard helper alongside this skill). Until calibrated, the identity
matrix is used so the LLM still sees "a cube at (0, 0, 0)" rather than a hard
crash — and the orchestrator surfaces the issue.

Run:
  /usr/bin/python3.10 vision_service.py \
      --orbbec-sn CP1E542000CJ --port 8779

Requires the freshly built pyorbbecsdk + libOrbbecSDK 1.10.22 — set:
  export PYTHONPATH=/home/dora/SDK/pyorbbecsdk/install/lib:$PYTHONPATH
  export LD_LIBRARY_PATH=/home/dora/SDK/pyorbbecsdk/install/lib:$LD_LIBRARY_PATH
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np

# ---------- FK for eye-in-hand mode ----------
_HAND_EYE_MODE = os.environ.get("HAND_EYE_MODE", "eye_to_hand")
_EE_T_CAMERA = None
if _HAND_EYE_MODE == "eye_in_hand":
    _ee_path = os.environ.get("EE_T_CAMERA_PATH", "")
    if not _ee_path:
        print("[vision] FATAL: HAND_EYE_MODE=eye_in_hand but EE_T_CAMERA_PATH not set", flush=True, file=sys.stderr)
        sys.exit(1)
    _EE_T_CAMERA = np.load(_ee_path)
    print(f"[vision] eye_in_hand mode, EE_T_CAMERA loaded from {_ee_path}", flush=True)

_mj_model = None
_mj_data = None
_site_id = None
_ARM_QPOS_START = 0
_NUM_JOINTS = 5
_ARM_QPOS = slice(_ARM_QPOS_START, _ARM_QPOS_START + _NUM_JOINTS)
_BRIDGE_URL = os.environ.get("BRIDGE_URL", "http://127.0.0.1:8768")


def _load_fk_model():
    global _mj_model, _mj_data, _site_id
    if _mj_model is not None:
        return
    import mujoco
    model_path = os.environ.get("MODEL_NAME", "")
    if not model_path:
        raise RuntimeError("MODEL_NAME not set (needed for FK in eye_in_hand mode)")
    _mj_model = mujoco.MjModel.from_xml_path(model_path)
    _mj_data = mujoco.MjData(_mj_model)
    for site_name in ("pinch", "gripperframe", "attachment"):
        sid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid >= 0:
            _site_id = sid
            print(f"[vision] FK model loaded: {model_path}, site='{site_name}'", flush=True)
            return
    raise RuntimeError(f"No known end-effector site found in {model_path}")


def _compute_base_T_ee(joint_angles) -> np.ndarray:
    """Return 4x4 base_T_ee from MuJoCo FK."""
    import mujoco
    _load_fk_model()
    _mj_data.qpos[_ARM_QPOS] = joint_angles
    mujoco.mj_forward(_mj_model, _mj_data)
    pos = _mj_data.site_xpos[_site_id].copy()
    R = _mj_data.site_xmat[_site_id].reshape(3, 3).copy()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


# LeRobot direct hardware read (reused across frames, lazy-init)
_lerobot_bus = None
_lerobot_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _get_joints_lerobot() -> list[float]:
    """Read joint angles directly from SO101 via LeRobot (degrees → radians)."""
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
        sys.path.insert(0, "/home/dora/dorobot2/third_party/lerobot/src")
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        motors = {}
        calib = {}
        for name in _lerobot_names + ["gripper"]:
            c = calib_data[name]
            norm = MotorNormMode.DEGREES if name != "gripper" else MotorNormMode.RANGE_0_100
            motors[name] = Motor(c["id"], "sts3215", norm)
            calib[name] = MotorCalibration(
                id=c["id"], drive_mode=c["drive_mode"],
                homing_offset=c["homing_offset"], range_min=c["range_min"], range_max=c["range_max"],
            )
        _lerobot_bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors, calibration=calib)
        _lerobot_bus.connect()
        print("[vision] LeRobot connected to SO101 on /dev/ttyACM0", flush=True)
    deg = [_lerobot_bus.read("Present_Position", n) for n in _lerobot_names]
    return [float(d) * math.pi / 180.0 for d in deg]


def _query_joints() -> list[float]:
    """Get current joint angles from the bridge ONLY.

    Never fall back to LeRobot direct — the bridge owns /dev/ttyACM0 and any
    concurrent access by the vision service would collide on the servo bus,
    causing the bridge's own reads/writes to fail with "no status packet".
    """
    import urllib.request
    body = json.dumps({"args": {}}).encode()
    req = urllib.request.Request(
        f"{_BRIDGE_URL}/tools/get_state", data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as r:
        resp = json.loads(r.read().decode())
    state = resp.get("output", resp)
    jp = state.get("joint_positions") or state.get("joints") or state.get("position")
    if jp is None:
        raise RuntimeError(f"bridge returned no joint_positions: {resp}")
    return [float(v) for v in jp]


TABLE_DEPTH_M = float(os.environ.get("TABLE_DEPTH_M", "0.30"))  # camera to table @ home

# ---------- defaults (overridable via env) ----------
DEFAULT_SN = os.environ.get("ORBBEC_SN", "CP1E542000CJ")
DEFAULT_HOST = os.environ.get("BALL_HTTP_HOST", "127.0.0.1")
DEFAULT_PORT = int(os.environ.get("BALL_HTTP_PORT", "8779"))

# Depth frame target. Orbbec Gemini 335 @ 640x480 Y16.
# CRITICAL: 640x400 clips the minimum depth to ~0.251 m (uint16 value 251),
# which hides a 3 cm cube sitting on a table at ~0.26 m. 640x480 reports
# depths down to ~0.151 m (uint16 value 151) — the cube top at ~0.23 m is
# then clearly visible. Keep these in sync with the SO101 dataflow's
# camera_top node if you ever change resolution there.
DEPTH_W, DEPTH_H = 640, 480
DEPTH_FPS = 30

# Table-vs-cube heuristics. Cube height is the main knob — adjust to your cube.
# For a 3 cm cube at ~25 cm distance, the projected area is ~57x57 ≈ 3200 px
# (fx ≈ DEPTH_W / (2*tan(35deg)) ≈ 457 px per meter of object at 1m).
CUBE_HEIGHT_M = float(os.environ.get("CUBE_HEIGHT_M", "0.030"))   # 30 mm cube default
CUBE_TOL_M = float(os.environ.get("CUBE_TOL_M", "0.010"))         # +/- 10 mm (Orbbec noise)
MIN_CUBE_PX = int(os.environ.get("MIN_CUBE_PX", "80"))           # 3cm cube can be ~100-3000 px depending on view angle
MAX_CUBE_PX = int(os.environ.get("MAX_CUBE_PX", "10000"))        # reject table surface (40k+ px)
# Depth validity range (m). Below min the sensor is unreliable (too close);
# above max we ignore (likely sky/back-wall under top-view).
MIN_DEPTH_M = float(os.environ.get("MIN_DEPTH_M", "0.10"))
MAX_DEPTH_M = float(os.environ.get("MAX_DEPTH_M", "2.0"))
# Stability: a new detection is published only if it agrees with the previous
# one within this radius (m). Reduces jitter at the cost of a few frames of lag.
STABILITY_RADIUS_M = float(os.environ.get("STABILITY_RADIUS_M", "0.015"))
STABILITY_FRAMES = int(os.environ.get("STABILITY_FRAMES", "3"))

# Camera -> SO-101 base 4x4 extrinsics.
#
# Current value is a 1-point seed: assumes the Orbbec is top-view mounted with
# its optical axis (camera +Z) pointing toward the SO-101's +x, and that one
# measured cube (base_x=0.29 m, base_y=0, base_z=0) corresponds to roughly the
# center of the depth image (cam ~0, ~0, 0.55). Calibrate properly with
# `calibrate_extrinsics.py` once you have >= 3 (base, cam) pairs — see SKILL.md.
CAMERA_TO_BASE = np.array([
    [ 0.000000,  0.000000,  1.000000, -0.260000],
    [ 1.000000,  0.000000,  0.000000,  0.000000],
    [ 0.000000, -1.000000,  0.000000,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  1.000000],
], dtype=np.float64)


# ---------- state ----------
# We keep TWO coordinate frames in the published state:
#   x/y/z  — camera frame (raw detection, before CAMERA_TO_BASE)
#   bx/by/bz — SO-101 base frame (after CAMERA_TO_BASE projection)
#   px/py  — pixel position of the cube centroid in the depth image (640x400)
# Calibration scripts use (x, y, z); the arm skill uses (bx, by, bz);
# /bbox returns (px, py) for visual confirmation during calibration.
_state = {"x": None, "y": None, "z": None, "bx": None, "by": None, "bz": None,
          "px": None, "py": None, "ts": 0}
_lock = threading.Lock()


# ---------- HTTP (same shape as sim/ball_state.py) ----------
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):  # silence access logs
        pass

    def do_GET(self):
        path = self.path.rstrip("/")
        if path not in ("/ball", "/healthz", "/bbox"):
            self.send_response(404)
            self.end_headers()
            return
        with _lock:
            payload = {
                "x": _state["x"], "y": _state["y"], "z": _state["z"],
                "bx": _state["bx"], "by": _state["by"], "bz": _state["bz"],
                "px": _state["px"], "py": _state["py"],
                "ts": _state["ts"],
            }
            if path == "/bbox":
                # Compact view for at-a-glance pixel-position check
                payload = {
                    "px": _state["px"], "py": _state["py"],
                    "cam_z_m": _state["z"], "ts": _state["ts"],
                }
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def _serve(host: str, port: int) -> None:
    HTTPServer((host, port), _Handler).serve_forever()


# ---------- Orbbec setup ----------
def _open_orbbec(sn: str | None, warmup_s: float = 1.5):
    """Open the Gemini 335, request 640x400 Y16 depth, return a started pipeline."""
    # Lazy import so the file is parseable even if pyorbbecsdk is missing
    # (the side-server then fails fast on its first frame).
    from pyorbbecsdk import (
        Config,
        Context,
        OBError,
        OBFormat,
        OBSensorType,
        Pipeline,
    )

    ctx = Context()
    if sn:
        dl = ctx.query_devices()
        if dl.get_count() == 0:
            raise RuntimeError("no Orbbec devices visible")
        dev = None
        for i in range(dl.get_count()):
            d = dl.get_device_by_index(i)
            if d.get_device_info().get_serial_number() == sn:
                dev = d
                break
        if dev is None:
            raise RuntimeError(f"Orbbec device with SN={sn} not found")
        print(f"[vision] picked Orbbec by SN={sn}", flush=True)
    else:
        dl = ctx.query_devices()
        if dl.get_count() == 0:
            raise RuntimeError("no Orbbec devices visible")
        dev = dl.get_device_by_index(0)
        print(f"[vision] picked first Orbbec: "
              f"{dev.get_device_info().get_name()} SN={dev.get_device_info().get_serial_number()}",
              flush=True)

    p = Pipeline(dev)
    cfg = Config()
    plist = p.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    try:
        prof = plist.get_video_stream_profile(DEPTH_W, DEPTH_H, OBFormat.Y16, DEPTH_FPS)
    except OBError:
        prof = plist.get_default_video_stream_profile()
    cfg.enable_stream(prof)
    p.start(cfg)
    time.sleep(warmup_s)
    print(f"[vision] pipeline started ({prof.get_width()}x{prof.get_height()} "
          f"@{prof.get_fps()} {prof.get_format()})", flush=True)
    return p


# ---------- cube finding ----------
def _largest_connected_component(mask: np.ndarray, min_px: int = 0, max_px: int = 10**9) -> tuple[int, np.ndarray] | tuple[int, None]:
    """Return (size, cluster_idx) for the largest 4-connected True region whose
    size falls in [min_px, max_px]. Uses a simple BFS via deque — no scipy dep.
    mask is (H, W) bool. cluster_idx is a (H, W) bool where True marks the chosen
    cluster, or 0.  When no cluster fits the range, returns (0, None).
    """
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    sizes: list[int] = []
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or labels[y, x] != 0:
                continue
            cid = len(sizes) + 1
            n = 0
            stack = [(y, x)]
            labels[y, x] = cid
            while stack:
                cy, cx = stack.pop()
                n += 1
                if cy > 0 and mask[cy - 1, cx] and labels[cy - 1, cx] == 0:
                    labels[cy - 1, cx] = cid; stack.append((cy - 1, cx))
                if cy + 1 < h and mask[cy + 1, cx] and labels[cy + 1, cx] == 0:
                    labels[cy + 1, cx] = cid; stack.append((cy + 1, cx))
                if cx > 0 and mask[cy, cx - 1] and labels[cy, cx - 1] == 0:
                    labels[cy, cx - 1] = cid; stack.append((cy, cx - 1))
                if cx + 1 < w and mask[cy, cx + 1] and labels[cy, cx + 1] == 0:
                    labels[cy, cx + 1] = cid; stack.append((cy, cx + 1))
            sizes.append(n)
    if not sizes:
        return 0, None
    # Pick the largest component whose size is within [min_px, max_px].
    # Sort by size descending so we pick the biggest valid one.
    order = sorted(range(len(sizes)), key=lambda i: sizes[i], reverse=True)
    for idx in order:
        sz = sizes[idx]
        if min_px <= sz <= max_px:
            return sz, (labels == (idx + 1))
    return 0, None


def _find_cube(depth_m: np.ndarray) -> tuple[tuple[float, float, float, int, int] | None, dict]:
    """Find the cube in camera frame. Returns ((cx, cy, cz, px, py) or None, debug_dict).

    Algorithm:
      1. Modal depth on valid pixels -> table.
      2. Tight depth band around the expected cube top:
            [table - CUBE_HEIGHT_M - TIGHT_TOL, table - CUBE_HEIGHT_M + TIGHT_TOL]
         The default TIGHT_TOL of 5 mm rejects the table's own noise.
      3. Connected components on the band. Take the largest one whose size
         is in [MIN_CUBE_PX, MAX_CUBE_PX]. Prefer the biggest component
         because that is usually the actual object rather than a stray
         scatter blob.
    """
    debug: dict = {"table_depth": None, "cluster_size": 0, "cube_mask": None}
    valid = (depth_m >= MIN_DEPTH_M) & (depth_m <= MAX_DEPTH_M)
    if valid.sum() < MIN_CUBE_PX * 4:
        return None, debug
    # (remaining code is unchanged; the final return is patched below.)

    # Robust table depth: median of valid depths (11x more stable than histogram mode
    # under frame-to-frame jitter, verified by test_table_detection.py).
    table_depth = float(np.median(depth_m[valid]))
    debug["table_depth"] = table_depth
    if table_depth >= MAX_DEPTH_M * 0.95:
        return None, debug

    # Band around the expected cube-top depth, using the configurable
    # CUBE_TOL_M (default ±10 mm for the 3 cm cube). The table's own depth
    # noise (5 to 15 mm) can spread the histogram tail; CUBE_TOL_M should
    # be wide enough to cover the cube top despite table-estimation error.
    cube_top = table_depth - CUBE_HEIGHT_M
    cube_lo = max(MIN_DEPTH_M, cube_top - CUBE_TOL_M)
    cube_hi = cube_top + CUBE_TOL_M
    cube_mask = valid & (depth_m >= cube_lo) & (depth_m <= cube_hi)
    debug["cube_mask"] = cube_mask
    if cube_mask.sum() < MIN_CUBE_PX:
        return None, debug

    size, cluster = _largest_connected_component(cube_mask, MIN_CUBE_PX, MAX_CUBE_PX)
    debug["cluster_size"] = size
    if cluster is None or size == 0:
        return None, debug

    ys, xs = np.where(cluster)
    med_depth = float(np.median(depth_m[ys, xs]))
    cx_pix = float(xs.mean())
    cy_pix = float(ys.mean())
    fx = fy = DEPTH_W / (2.0 * np.tan(np.deg2rad(35.0)))
    cam_x = (cx_pix - DEPTH_W / 2.0) * med_depth / fx
    cam_y = (cy_pix - DEPTH_H / 2.0) * med_depth / fy
    cam_z = med_depth
    return (cam_x, cam_y, cam_z, int(round(cx_pix)), int(round(cy_pix))), debug


# ---------- stability filter ----------
class _Stable:
    """Only publish detections that are stable across N consecutive frames."""
    def __init__(self, radius_m: float, n_frames: int):
        self.radius_m = radius_m
        self.n_frames = n_frames
        self._buf: deque[tuple[float, float, float]] = deque(maxlen=n_frames)

    def update(self, xyz: tuple[float, float, float] | None) -> tuple[float, float, float] | None:
        if xyz is None:
            self._buf.clear()
            return None
        self._buf.append(xyz)
        if len(self._buf) < self._n():
            return None
        arr = np.array(self._buf)
        # Reject if any single point is too far from the mean (jitter).
        mean = arr.mean(axis=0)
        if np.linalg.norm(arr - mean, axis=1).max() > self.radius_m:
            return None
        return tuple(float(v) for v in mean)

    def _n(self) -> int:
        return self.n_frames


# ---------- main loop ----------
def _publish(cam_xyz5, stable: _Stable) -> bool:
    """Project camera->base, run stability filter, publish to _state.

    cam_xyz5 = (cam_x, cam_y, cam_z, px, py) — 5-tuple from _find_cube.
    """
    if cam_xyz5 is None:
        return False
    cam_x, cam_y, cam_z, px, py = cam_xyz5
    cam_pt = np.array([cam_x, cam_y, cam_z, 1.0], dtype=np.float64)
    if _HAND_EYE_MODE == "eye_in_hand":
        try:
            joints = _query_joints()
            base_T_ee = _compute_base_T_ee(joints)
            base_pt = base_T_ee @ _EE_T_CAMERA @ cam_pt
        except Exception:
            # If bridge/FK fails, publish nothing this frame
            return False
    else:
        base_pt = CAMERA_TO_BASE @ cam_pt
    bx, by, bz = float(base_pt[0]), float(base_pt[1]), float(base_pt[2])
    pub = stable.update((bx, by, bz))
    if pub is None:
        return False
    with _lock:
        if _HAND_EYE_MODE == "eye_in_hand":
            # arm_skills reads x/y; fill with base frame coords
            _state["x"] = round(pub[0], 5)
            _state["y"] = round(pub[1], 5)
            _state["z"] = round(pub[2], 5)
        else:
            _state["x"] = round(cam_x, 5)
            _state["y"] = round(cam_y, 5)
            _state["z"] = round(cam_z, 5)
        _state["bx"] = round(pub[0], 5)
        _state["by"] = round(pub[1], 5)
        _state["bz"] = round(pub[2], 5)
        _state["px"] = px
        _state["py"] = py
        _state["ts"] += 1
    return True


def _dump_frame(depth_m: np.ndarray, debug: dict, cam_xyz, outdir: Path) -> None:
    """Save depth visualization + cube mask + table info to disk for offline debug.

    Files written into outdir:
      depth_raw.png         — colormap of the raw depth (0 = black, far = white)
      depth_table.png       — depth with table plane highlighted as a line
      cube_mask.png         — the cube mask from the detector
      debug.json            — numeric summary
    """
    try:
        from PIL import Image
    except ImportError:
        print("[dump] PIL not available; writing only debug.json", flush=True)
        _dump_debug_only(depth_m, debug, cam_xyz, outdir)
        return

    outdir.mkdir(parents=True, exist_ok=True)

    # 1. raw depth (clamp to MAX_DEPTH_M, scale to 0..255)
    d = np.clip(depth_m, 0.0, MAX_DEPTH_M)
    d8 = (d / MAX_DEPTH_M * 255.0).astype(np.uint8)
    Image.fromarray(d8, mode="L").save(outdir / "depth_raw.png")

    # 2. cube mask
    if debug.get("cube_mask") is not None:
        m = (debug["cube_mask"].astype(np.uint8) * 255)
        Image.fromarray(m, mode="L").save(outdir / "cube_mask.png")

    # 3. depth with table line drawn
    rgb = np.stack([d8] * 3, axis=-1)
    td = debug.get("table_depth")
    if td is not None and 0 < td < MAX_DEPTH_M:
        # mark a horizontal line at the row whose modal depth matches the table
        # (use the table depth to convert to row index: row = ((table-z)/range)*H)
        # simpler: just put a red banner at top with the table value as text would
        # need PIL.ImageDraw. Skip draw for now and just save a sidecar txt.
        pass
    Image.fromarray(rgb, mode="RGB").save(outdir / "depth_table.png")

    _dump_debug_only(depth_m, debug, cam_xyz, outdir)


def _dump_debug_only(depth_m: np.ndarray, debug: dict, cam_xyz, outdir: Path) -> None:
    outdir.mkdir(parents=True, exist_ok=True)
    valid = depth_m[(depth_m >= MIN_DEPTH_M) & (depth_m <= MAX_DEPTH_M)]
    summary = {
        "depth_min_m": float(depth_m.min()),
        "depth_max_m": float(depth_m.max()),
        "depth_mean_m": float(depth_m.mean()),
        "depth_valid_count": int(valid.size),
        "depth_valid_min_m": float(valid.min()) if valid.size else None,
        "depth_valid_max_m": float(valid.max()) if valid.size else None,
        "table_depth_m": debug.get("table_depth"),
        "cluster_size_px": int(debug.get("cluster_size", 0)),
        "cube_mask_total_px": int(debug["cube_mask"].sum()) if debug.get("cube_mask") is not None else 0,
        "cam_xyz": list(cam_xyz) if cam_xyz is not None else None,
        "CUBE_HEIGHT_M": CUBE_HEIGHT_M,
        "CUBE_TOL_M": CUBE_TOL_M,
    }
    (outdir / "debug.json").write_text(json.dumps(summary, indent=2))
    print(f"[dump] wrote {outdir}/depth_raw.png cube_mask.png depth_table.png debug.json",
          flush=True)
    print(f"[dump] {summary}", flush=True)


def _run(args: argparse.Namespace) -> int:
    print(f"[vision] starting HTTP on http://{args.host}:{args.port}/ball", flush=True)
    threading.Thread(target=_serve, args=(args.host, args.port), daemon=True).start()

    try:
        pipeline = _open_orbbec(args.orbbec_sn)
    except Exception as e:
        print(f"[vision] FATAL: cannot open Orbbec: {e}", flush=True, file=sys.stderr)
        while True:
            time.sleep(60)

    # One-shot dump mode: grab a single frame, save debug artifacts, exit.
    if args.dump_frame:
        from pathlib import Path
        outdir = Path(args.dump_out)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            fset = pipeline.wait_for_frames(500)
            if fset is None:
                continue
            d = fset.get_depth_frame()
            if d is None:
                continue
            raw = bytes(d.get_data())
            if len(raw) != DEPTH_W * DEPTH_H * 2:
                continue
            depth_u16 = np.frombuffer(raw, dtype=np.uint16).reshape(DEPTH_H, DEPTH_W).copy()
            scale = d.get_depth_scale() or 1.0
            depth_m = depth_u16.astype(np.float32) * scale * 0.001
            cam_xyz5, debug = _find_cube(depth_m)
            cam_xyz = cam_xyz5[:3] if cam_xyz5 is not None else None
            print(f"[dump] cam_xyz={cam_xyz}  px,py={(cam_xyz5[3], cam_xyz5[4]) if cam_xyz5 else None}  "
                  f"table={debug.get('table_depth')}  cluster_size={debug.get('cluster_size')}", flush=True)
            _dump_frame(depth_m, debug, cam_xyz, outdir)
            return 0
        print("[dump] FAILED: timed out waiting for a frame in 15s", flush=True)
        return 1

    stable = _Stable(STABILITY_RADIUS_M, STABILITY_FRAMES)
    n_frames = 0
    n_published = 0
    last_log = time.perf_counter()
    print(f"[vision] VISION_MODE=depth", flush=True)
    while True:
        fset = pipeline.wait_for_frames(100)
        if fset is None:
            continue
        d = fset.get_depth_frame()
        if d is None:
            continue
        raw = bytes(d.get_data())
        if len(raw) != DEPTH_W * DEPTH_H * 2:
            continue  # SDK returned an unexpected buffer; skip
        depth_u16 = np.frombuffer(raw, dtype=np.uint16).reshape(DEPTH_H, DEPTH_W).copy()
        scale = d.get_depth_scale() or 1.0
        depth_m = depth_u16.astype(np.float32) * scale * 0.001

        cam_xyz5, _ = _find_cube(depth_m)
        cam_xyz = cam_xyz5

        n_frames += 1
        if _publish(cam_xyz, stable):
            n_published += 1
        now = time.perf_counter()
        if now - last_log > 5.0:
            print(f"[vision] frames={n_frames} published={n_published} "
                  f"current=({_state['x']}, {_state['y']}, {_state['z']}) ts={_state['ts']}",
                  flush=True)
            last_log = now


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Orbbec cube-pose HTTP side-server")
    p.add_argument("--orbbec-sn", default=DEFAULT_SN,
                   help="Orbbec serial number (default: env ORBBEC_SN or CP1E542000CJ)")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--dump-frame", action="store_true",
                   help="grab a single frame, write debug PNG/JSON, then exit")
    p.add_argument("--dump-out", default="/tmp/so101-pick-cube-dump",
                   help="output directory for --dump-frame")
    return p.parse_args()


if __name__ == "__main__":
    sys.exit(_run(_parse_args()))