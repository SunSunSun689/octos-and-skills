#!/usr/bin/env python3
"""One-shot cube finder via depth image + manual pixel selection.

Grabs one depth frame from Orbbec Gemini 335, saves a JET visualization,
reads cube pixel coords from user, computes base frame (x,y,z) via
eye_in_hand FK, and prints the pick_cube_at command.

Usage:
  # Interactive: shows depth image, you enter pixel coords
  python find_cube.py --orbbec-sn CP1E542000CJ

  # Non-interactive: provide pixel coords directly
  python find_cube.py --px 320 --py 200
"""
from __future__ import annotations

import argparse, json, math, os, sys, time, urllib.request
import numpy as np


def _get_joints_from_bridge():
    """Read current joint angles from minimal_bridge (:8768)."""
    body = json.dumps({"args": {}}).encode()
    req = urllib.request.Request(
        "http://127.0.0.1:8768/tools/get_state", data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as r:
        return [float(v) for v in json.loads(r.read().decode())["joint_positions"]]


def _fk_base_T_ee(joints, model_path):
    """MuJoCo FK: joint angles → 4x4 base_T_ee."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    d.qpos[0:5] = joints
    mujoco.mj_forward(m, d)
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    pos = d.site_xpos[sid].copy()
    R = d.site_xmat[sid].reshape(3, 3).copy()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--orbbec-sn", default="CP1E542000CJ")
    p.add_argument("--ee-t-camera", default=os.environ.get("EE_T_CAMERA_PATH",
        "/home/dora/.octos/skills/so101-pick-cube/ee_T_camera.npy"))
    p.add_argument("--model", default=os.environ.get("MODEL_NAME",
        "/home/dora/dorobot2/third_party/robodriver/descriptions/mjcf/so101/so101_new_calib.xml"))
    p.add_argument("--px", type=int, default=None)
    p.add_argument("--py", type=int, default=None)
    args = p.parse_args()

    from pyorbbecsdk import Config, Context, OBFormat, OBSensorType, Pipeline

    ctx = Context()
    dl = ctx.query_devices()
    dev = None
    if args.orbbec_sn:
        for i in range(dl.get_count()):
            d = dl.get_device_by_index(i)
            if d.get_device_info().get_serial_number() == args.orbbec_sn:
                dev = d; break
    if dev is None:
        dev = dl.get_device_by_index(0)
    print(f"Device: {dev.get_device_info().get_name()} SN={dev.get_device_info().get_serial_number()}")

    pipeline = Pipeline(dev)
    cfg = Config()
    plist = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    prof = plist.get_video_stream_profile(640, 400, OBFormat.Y16, 30)
    cfg.enable_stream(prof)
    pipeline.start(cfg)
    time.sleep(1.5)

    # Drain stale frames
    for _ in range(5):
        pipeline.wait_for_frames(100)

    # Grab a frame
    fset = pipeline.wait_for_frames(2000)
    if fset is None:
        print("ERROR: no frames"); pipeline.stop(); return 1
    dframe = fset.get_depth_frame()
    if dframe is None:
        print("ERROR: no depth frame"); pipeline.stop(); return 1

    dw, dh = dframe.get_width(), dframe.get_height()
    scale = dframe.get_depth_scale() or 1.0
    raw = bytes(dframe.get_data())
    depth_u16 = np.frombuffer(raw, dtype=np.uint16).reshape(dh, dw).copy()
    depth_m = depth_u16.astype(np.float32) * scale * 0.001

    # Get intrinsics
    cam_param = pipeline.get_camera_param()
    intr = cam_param.depth_intrinsic
    fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
    print(f"Depth: {dw}x{dh}  fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    pipeline.stop()

    # Save visualization
    import cv2
    valid = np.clip(depth_m, 0.05, 2.0)
    vis = (valid / 2.0 * 255).astype(np.uint8)
    vis_color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    out_path = "/tmp/find_cube_depth.jpg"
    cv2.imwrite(out_path, vis_color)
    print(f"Depth visualization saved: {out_path} ({dw}x{dh})")
    print(f"Depth range: {depth_m[depth_m>0.01].min():.3f}m ~ {depth_m[depth_m>0.01].max():.3f}m")

    # Get pixel coords
    px, py = args.px, args.py
    if px is None or py is None:
        print(f"\nOpen {out_path} and find the cube center pixel.")
        raw = input("Enter px py: ").strip()
        px, py = map(int, raw.split())

    px = max(0, min(dw-1, px))
    py = max(0, min(dh-1, py))

    # Median depth in 5x5 window around selected pixel
    window = depth_m[max(0,py-2):py+3, max(0,px-2):px+3]
    med_depth = float(np.median(window[window > 0.01]))
    if med_depth <= 0.01:
        print(f"ERROR: invalid depth at ({px},{py}): {med_depth:.3f}m"); return 1

    # Camera frame coords (pinhole)
    cam_x = (px - cx) * med_depth / fx
    cam_y = (py - cy) * med_depth / fy
    cam_z = med_depth

    # Base frame via eye_in_hand FK
    joints = _get_joints_from_bridge()
    ee_T_camera = np.load(args.ee_t_camera)
    base_T_ee = _fk_base_T_ee(joints, args.model)
    base_pt = base_T_ee @ ee_T_camera @ np.array([cam_x, cam_y, cam_z, 1.0])
    bx, by, bz = float(base_pt[0]), float(base_pt[1]), float(base_pt[2])

    print(f"\n{'='*50}")
    print(f"  Camera frame: x={cam_x:.4f} y={cam_y:.4f} z={cam_z:.4f}")
    print(f"  Base frame:   x={bx:.4f} y={by:.4f} z={bz:.4f}")
    print(f"  Joints: {[f'{j:.2f}' for j in joints]}")
    print(f"{'='*50}")
    print(f"\n  arm_skills.pick_at({bx:.3f}, {by:.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
