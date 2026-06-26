#!/usr/bin/env python3
"""Hand-eye calibration helper for so101-pick-cube.

Goal
----
Compute a 4x4 camera->base extrinsics matrix (CAMERA_TO_BASE) to paste into
`vision_service.py` from a small number of (cube_in_base, cube_in_camera)
correspondences.

Workflow
--------
You do NOT need to measure anything in the camera frame — the camera will
tell you where the cube is in its own frame. You just need to know where
the cube is in the SO-101 base frame (measured with a ruler / tape).

The naive recipe is:
  1. Put a cube at a known (x, y, z) in base frame.
  2. Run vision_service and read GET /ball — that's (x_cam, y_cam, z_cam).
  3. We now have one correspondence (p_base, p_cam).
  4. Repeat for 3 distinct (x, y) positions.
  5. We solve the rigid transform (R, t) such that p_base = R @ p_cam + t.

This script does step 5: it reads a JSON file of correspondences and writes
a 4x4 matrix to stdout (or --out <path>).

Why not 4 points?
-----------------
A rigid transform has 6 DOF. 3 non-collinear points give 9 equations and
uniquely determine (R, t). 4 points is the over-determined "proper" way to
do it (Kabsch / Arun's algorithm). We use 3 because that matches the
manual-measurement cadence ("put the cube in 3 corners, type 3 numbers,
done").

If you have >= 4 points, we automatically use Kabsch (SVD) for the
least-squares best fit. 3 points solves exactly.

Coordinate frame assumptions
----------------------------
  - base frame:  +x is the SO-101's "forward" (arm reach direction),
                 +y is the arm's left (looking from above, behind the
                 arm), +z is up. origin is the SO-101 base center on
                 the table (the screw/bolt hole in the bottom plate).
  - camera frame: pinhole model with the optical axis along +Z.
                  Orbbec Gemini 335 depth FOV ~70 deg horizontal.

Usage
-----
  # 1. Start the vision service (without CAMERA_TO_BASE patched yet —
  #    the identity will report camera-frame values, which is fine).
  nohup /usr/bin/python3.10 vision_service.py --port 8779 \
      > /tmp/so101-pick-cube-vision.log 2>&1 &

  # 2. Put the cube at a known base position (e.g. x=0.29, y=0, z=0.0).
  # 3. Wait for /ball to stabilize, then read it.
  curl -s http://127.0.0.1:8779/ball
  #   -> {"x": -0.024, "y": 0.038, "z": 0.582, "ts": 7}

  # 4. Append that to a JSON correspondences file (see --example below).

  # 5. After 3+ points:
  /usr/bin/python3.10 calibrate_extrinsics.py --correspondences points.json

Format of --correspondences
---------------------------
A JSON file with this shape:
  {
    "points": [
      {"base": [0.29, 0.00, 0.00], "cam": [-0.024, 0.038, 0.582]},
      {"base": [0.20, 0.10, 0.00], "cam": [-0.040, 0.012, 0.580]},
      {"base": [0.20,-0.10, 0.00], "cam": [ 0.010,-0.020, 0.581]}
    ]
  }
Each point is a (x, y, z) tuple in meters.

Output
------
A numpy-style 4x4 matrix printed as Python list-of-lists. Copy-paste into
vision_service.py's CAMERA_TO_BASE constant.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def _solve_rigid(cam_pts: np.ndarray, base_pts: np.ndarray) -> np.ndarray:
    """Solve p_base = R @ p_cam + t via Kabsch (SVD) for >=3 points.

    cam_pts: (N, 3)  — points measured in the camera frame
    base_pts: (N, 3) — the same physical points measured in base frame

    Returns 4x4 homogeneous matrix.
    """
    assert cam_pts.shape == base_pts.shape
    assert cam_pts.shape[0] >= 3, "need at least 3 non-collinear points"

    cam_c = cam_pts.mean(axis=0)
    base_c = base_pts.mean(axis=0)
    cam_d = cam_pts - cam_c
    base_d = base_pts - base_c

    # Kabsch
    H = cam_d.T @ base_d
    U, S, Vt = np.linalg.svd(H)
    # Ensure a proper rotation (no reflection)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = base_c - R @ cam_c

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _print_matrix(T: np.ndarray) -> str:
    rows = []
    for row in T:
        rows.append("[" + ", ".join(f"{v: .6f}" for v in row) + "]")
    return "[\n  " + ",\n  ".join(rows) + ",\n]"


def _example_points() -> dict:
    """Return the working example from the SKILL.md 'First-run calibration' section."""
    return {
        "points": [
            # (base_x_m, base_y_m, base_z_m) <- measured from arm base
            # (cam_x_m,  cam_y_m,  cam_z_m)  <- what /ball reported
            {"base": [0.29, 0.00, 0.00], "cam": [-0.024,  0.038, 0.582]},
            {"base": [0.20, 0.10, 0.00], "cam": [-0.040,  0.012, 0.580]},
            {"base": [0.20,-0.10, 0.00], "cam": [ 0.010, -0.020, 0.581]},
        ]
    }


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--correspondences", type=Path,
                   help="JSON file with {'points': [{'base':[..], 'cam':[..]}, ...]}")
    p.add_argument("--example", action="store_true",
                   help="print an example correspondences JSON to stdout and exit")
    p.add_argument("--out", type=Path,
                   help="write the 4x4 to this file (default: stdout)")
    return p.parse_args()


def main() -> int:
    args = _parse_args()

    if args.example:
        print(json.dumps(_example_points(), indent=2))
        return 0

    if args.correspondences is None:
        print("error: pass --correspondences <file> or --example", file=sys.stderr)
        return 2

    data = json.loads(args.correspondences.read_text())
    raw = data["points"]
    if len(raw) < 3:
        print(f"error: need at least 3 points, got {len(raw)}", file=sys.stderr)
        return 2

    cam_pts = np.array([p["cam"]  for p in raw], dtype=np.float64)
    base_pts = np.array([p["base"] for p in raw], dtype=np.float64)
    T = _solve_rigid(cam_pts, base_pts)

    # Sanity-check the residual
    pred = (T[:3, :3] @ cam_pts.T).T + T[:3, 3]
    err = np.linalg.norm(pred - base_pts, axis=1)
    rms = float(np.sqrt(np.mean(err ** 2)))

    rendered = _print_matrix(T)
    out = (
        f"# solved from {len(raw)} point(s); RMS residual = {rms*1000:.2f} mm\n"
        f"# paste into vision_service.py:\n"
        f"CAMERA_TO_BASE = np.array({rendered}, dtype=np.float64)\n"
    )
    if args.out:
        args.out.write_text(out)
        print(f"wrote {args.out} (RMS={rms*1000:.2f} mm)")
    else:
        print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
