#!/usr/bin/env python3
"""Fully automated hand-eye calibration using calibrate_hand_eye.py internals."""
import json, math, os, sys, time, urllib.request, subprocess
from pathlib import Path

import numpy as np

BRIDGE = "http://127.0.0.1:8768"
MODEL_NAME = os.environ.get("MODEL_NAME",
    "/home/dora/dorobot2/third_party/robodriver/descriptions/mjcf/so101/so101_new_calib.xml")
PAIRS_FILE = "/tmp/so101_hand_eye_pairs_auto.json"
EE_T_CAMERA_OUT = os.environ.get("EE_T_CAMERA",
    "/home/dora/.octos/skills/so101-pick-cube/ee_T_camera.npy")
ORBBEC_SN = os.environ.get("ORBBEC_SN", "CP1E542000CJ")
SCRIPT_DIR = Path(__file__).resolve().parent
PATTERN_COLS = 9
PATTERN_ROWS = 6
SQUARE_SIZE = 0.025

# 5 poses with angular diversity
POSES_DEG = [
    ("pose1_home",     [-10.6,  -3.3,  15.8,  69.5,  -0.1]),
    ("pose2_extend",   [-10.6,  20.0,  35.0,  80.0,  -0.1]),
    ("pose3_retract",  [-10.6,   5.0,  -5.0,  50.0,  -0.1]),
    ("pose4_right",    [-30.0,  10.0,  25.0,  65.0, -15.0]),
    ("pose5_left",     [ 15.0,  10.0,  25.0,  65.0,  15.0]),
]

def bridge_call(method, path, args=None, timeout=120):
    data = json.dumps({"args": args or {}}).encode()
    req = urllib.request.Request(
        f"{BRIDGE}{path}", data=data,
        headers={"Content-Type": "application/json"}, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())

def move_to_named(name="home"):
    return bridge_call("POST", "/tools/vendor.moveit.arm.move_to_named", {"name": name})

def move_to_joints(joints_deg):
    joints_rad = [d * math.pi / 180.0 for d in joints_deg]
    return bridge_call("POST", "/tools/vendor.moveit.arm.move_to_joint_state",
                       {"joints": joints_rad})

def get_joints_rad():
    return [float(v) for v in bridge_call("POST", "/tools/get_state")["joint_positions"]]

# ---- Use calibrate_hand_eye internals ----
sys.path.insert(0, str(SCRIPT_DIR))
from calibrate_hand_eye import (
    _open_orbbec_color, _grab_color_frame, _detect_checkerboard,
    compute_base_T_ee
)

def main():
    print("=" * 60)
    print("Automated Hand-Eye Calibration (5 poses)")
    print("=" * 60)

    # Open camera once for all poses
    print("\n[1] Opening Orbbec color stream...")
    pipeline, camera_matrix, dist_coeffs = _open_orbbec_color(ORBBEC_SN, warmup_s=2.0)
    color_format = None  # _grab_color_frame handles this
    print("  OK")

    # Move to home
    print("\n[2] Moving arm to home...")
    move_to_named("home")
    time.sleep(2)

    # Collect pairs
    print(f"\n[3] Collecting {len(POSES_DEG)} pose pairs...")
    pairs = []
    for idx, (name, joints_deg) in enumerate(POSES_DEG):
        print(f"\n  --- Pose {idx+1}/{len(POSES_DEG)}: {name} ---")
        print(f"  Target joints (deg): {[f'{d:.0f}' for d in joints_deg]}")

        # Move arm
        move_to_joints(joints_deg)
        time.sleep(4)  # settle

        # Read actual joints
        try:
            joints_rad = get_joints_rad()
            actual_deg = [j * 180 / math.pi for j in joints_rad]
            print(f"  Actual joints (deg): {[f'{d:.1f}' for d in actual_deg]}")
        except Exception as e:
            print(f"  ERROR reading joints: {e}")
            continue

        # Compute base_T_ee
        base_T_ee = compute_base_T_ee(joints_rad)

        # Detect checkerboard (try several frames)
        R_ct, t_ct = None, None
        for attempt in range(15):
            time.sleep(0.3)
            bgr = _grab_color_frame(pipeline, color_format)
            if bgr is None:
                continue
            result = _detect_checkerboard(bgr, (PATTERN_COLS, PATTERN_ROWS),
                                          SQUARE_SIZE, camera_matrix, dist_coeffs)
            if result is not None:
                R_ct, t_ct = result
                print(f"  Checkerboard detected (attempt {attempt+1})")
                break
        else:
            print(f"  WARNING: checkerboard not detected, skipping pose")
            continue

        pairs.append({
            "name": name,
            "joints": [round(j, 6) for j in joints_rad],
            "base_T_ee": base_T_ee.tolist(),
            "cam_R_target": R_ct.tolist(),
            "cam_t_target": t_ct.tolist(),
        })
        print(f"  Saved ({len(pairs)}/{len(POSES_DEG)})")

    pipeline.stop()
    print(f"\n  Collected {len(pairs)}/{len(POSES_DEG)} pairs")

    if len(pairs) < 3:
        print(f"ERROR: need at least 3 pairs, got {len(pairs)}")
        sys.exit(1)

    # Save pairs
    pairs_simple = [{
        "joints": p["joints"],
        "cam_R_target": p["cam_R_target"],
        "cam_t_target": p["cam_t_target"],
    } for p in pairs]
    with open(PAIRS_FILE, "w") as f:
        json.dump(pairs_simple, f, indent=2)
    print(f"Pairs saved to {PAIRS_FILE}")

    # Solve
    print(f"\n[4] Solving AX=XB with {len(pairs)} pairs...")
    result = subprocess.run(
        ["/home/dora/miniconda3/envs/pico/bin/python3",
         str(SCRIPT_DIR / "calibrate_hand_eye.py"), "solve",
         "--model", MODEL_NAME,
         "--pairs", PAIRS_FILE,
         "--out", EE_T_CAMERA_OUT],
        capture_output=True, text=True, timeout=60,
        env={**os.environ,
             "LD_LIBRARY_PATH": "/home/dora/SDK/pyorbbecsdk/install/lib:/home/dora/miniconda3/envs/pico/lib",
             "PYTHONPATH": "/home/dora/SDK/pyorbbecsdk/install/lib:/home/dora/miniconda3/envs/pico/lib/python3.10/site-packages",
             })
    print(result.stdout)
    if result.returncode != 0:
        print("STDERR:", result.stderr)
        sys.exit(1)

    print(f"Done! ee_T_camera saved to {EE_T_CAMERA_OUT}")
    print(f"export EE_T_CAMERA_PATH={EE_T_CAMERA_OUT}")
    print(f"export HAND_EYE_MODE=eye_in_hand")

    # Back to home
    print("\nMoving back to home...")
    move_to_named("home")
    print("Done.")

if __name__ == "__main__":
    main()
