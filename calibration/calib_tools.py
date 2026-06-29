"""Camera intrinsics and hand-eye calibration tools.

Standard two-step pipeline:
  1. Camera intrinsics: chessboard -> cv2.calibrateCamera()
  2. Hand-eye: fixed chessboard, N arm poses -> cv2.calibrateHandEye() (AX=XB)

Adapted from octos-nano. Uses arm_skills MuJoCo FK instead of kinematics.py.
"""

import cv2
import numpy as np
import json
import os
import glob
import math
import sys

CALIB_DIR = os.path.dirname(os.path.abspath(__file__))
SKILL_DIR = os.path.dirname(CALIB_DIR)
sys.path.insert(0, SKILL_DIR)

IMAGES_DIR = os.path.join(CALIB_DIR, "chessboard_images")
HAND_EYE_DATA = os.path.join(CALIB_DIR, "hand_eye_data.json")
INTRINSICS_FILE = os.path.join(CALIB_DIR, "camera_intrinsics.json")
HAND_EYE_FILE = os.path.join(CALIB_DIR, "hand_eye.json")

CHESSBOARD = (9, 6)


def _obj_points(square_size_mm):
    w, h = CHESSBOARD
    objp = np.zeros((w * h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
    objp *= square_size_mm / 1000.0
    return objp


def _load_intrinsics():
    if not os.path.exists(INTRINSICS_FILE):
        return None
    with open(INTRINSICS_FILE) as f:
        data = json.load(f)
    if not data or "camera_matrix" not in data:
        return None
    return data


def detect_chessboard(frame_bgr, square_size_mm):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)
    if not ret:
        return False, None, None, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    K = _load_intrinsics()
    if K is not None:
        mtx = np.array(K["camera_matrix"])
        dist = np.array(K["dist_coeffs"])
    else:
        h, w = frame_bgr.shape[:2]
        mtx = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], dtype=np.float32)
        dist = np.zeros((1, 5), dtype=np.float32)

    objp = _obj_points(square_size_mm)
    retval, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
    return True, corners, rvec, tvec


def capture_chessboard_image(frame_bgr, square_size_mm):
    success, corners, rvec, tvec = detect_chessboard(frame_bgr, square_size_mm)
    if not success:
        return None

    os.makedirs(IMAGES_DIR, exist_ok=True)
    idx = len(glob.glob(os.path.join(IMAGES_DIR, "img_*.png")))
    path = os.path.join(IMAGES_DIR, f"img_{idx:03d}.png")
    cv2.imwrite(path, frame_bgr)

    return {"index": idx, "path": path, "total_images": idx + 1,
            "corners_found": len(corners)}


def calibrate_camera_intrinsics(square_size_mm):
    if not os.path.exists(IMAGES_DIR):
        raise FileNotFoundError(f"No chessboard images directory: {IMAGES_DIR}")

    image_paths = sorted(glob.glob(os.path.join(IMAGES_DIR, "img_*.png")))
    if len(image_paths) < 5:
        raise ValueError(f"Need at least 5 chessboard images, have {len(image_paths)}")

    objp = _obj_points(square_size_mm)
    obj_points, img_points = [], []
    img_size = None

    for path in image_paths:
        frame = cv2.imread(path)
        if img_size is None:
            img_size = frame.shape[:2][::-1]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)
        if not ret:
            continue
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners)

    if len(obj_points) < 5:
        raise ValueError(f"Only {len(obj_points)} images had detectable chessboards")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None)

    total_error = 0
    for i in range(len(obj_points)):
        projected, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(img_points[i], projected, cv2.NORM_L2) / len(projected)
        total_error += error
    mean_error = total_error / len(obj_points)

    result = {
        "camera_matrix": mtx.tolist(), "dist_coeffs": dist.tolist(),
        "image_size": list(img_size),
        "reprojection_error_px": round(float(mean_error), 4),
        "num_images_used": len(obj_points), "square_size_mm": square_size_mm,
        "chessboard": list(CHESSBOARD),
    }
    with open(INTRINSICS_FILE, "w") as f:
        json.dump(result, f, indent=2)
    return result


def collect_hand_eye_pose(frame_bgr, joints_dict, square_size_mm):
    success, corners, rvec, tvec = detect_chessboard(frame_bgr, square_size_mm)
    if not success:
        return None

    R_target2cam, _ = cv2.Rodrigues(rvec)
    t_target2cam = tvec.flatten()

    data = []
    if os.path.exists(HAND_EYE_DATA):
        with open(HAND_EYE_DATA) as f:
            data = json.load(f)

    pose = {
        "joints": {k: round(v, 2) for k, v in joints_dict.items()},
        "R_target2cam": R_target2cam.tolist(),
        "t_target2cam": t_target2cam.tolist(),
        "corners_count": len(corners),
    }
    data.append(pose)
    with open(HAND_EYE_DATA, "w") as f:
        json.dump(data, f, indent=2)

    return {"pose_index": len(data) - 1, "total_poses": len(data)}


def _rotation_to_rpy(R):
    """Convert 3x3 rotation matrix to roll-pitch-yaw (xyz extrinsic) in radians."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    return [roll, pitch, yaw]


def solve_hand_eye(method="park"):
    from arm_skills import forward_kinematics, _rpy_to_rot

    if not os.path.exists(HAND_EYE_DATA):
        raise FileNotFoundError(f"No hand-eye data file: {HAND_EYE_DATA}")
    with open(HAND_EYE_DATA) as f:
        data = json.load(f)

    if len(data) < 6:
        raise ValueError(f"Need at least 6 hand-eye poses, have {len(data)}")

    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []

    keys = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    for pose in data:
        j5 = [pose["joints"][k] for k in keys]
        fk = forward_kinematics(j5, with_gripper=False)
        T = fk["transform"]
        R_gripper2base.append(T[:3, :3])
        t_gripper2base.append(T[:3, 3].reshape(3, 1))
        R_target2cam.append(np.array(pose["R_target2cam"]))
        t_target2cam.append(np.array(pose["t_target2cam"]).reshape(3, 1))

    methods = {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    if method not in methods:
        raise ValueError(f"Unknown method '{method}'. Available: {list(methods.keys())}")

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
        method=methods[method])

    T_cam_in_wrist = np.eye(4)
    T_cam_in_wrist[:3, :3] = R_cam2gripper
    T_cam_in_wrist[:3, 3] = t_cam2gripper.flatten()

    # Verify consistency
    target_positions = []
    for i in range(len(data)):
        T_wb = np.eye(4); T_wb[:3, :3] = R_gripper2base[i]
        T_wb[:3, 3] = t_gripper2base[i].flatten()
        T_tc = np.eye(4); T_tc[:3, :3] = R_target2cam[i]
        T_tc[:3, 3] = t_target2cam[i].flatten()
        target_positions.append((T_wb @ T_cam_in_wrist @ T_tc)[:3, 3])

    target_positions = np.array(target_positions)
    mean_pos = target_positions.mean(axis=0)
    max_dev = np.max(np.linalg.norm(target_positions - mean_pos, axis=1))

    # Extract RPY from T_cam_in_wrist
    rpy = _rotation_to_rpy(T_cam_in_wrist[:3, :3])

    K = _load_intrinsics()
    if K is not None:
        mtx = np.array(K["camera_matrix"])
        cam_info = {"fx": float(mtx[0, 0]), "fy": float(mtx[1, 1]),
                    "cx": float(mtx[0, 2]), "cy": float(mtx[1, 2]),
                    "width": K["image_size"][0], "height": K["image_size"][1]}
    else:
        cam_info = {"fx": 486, "fy": 468, "cx": 320, "cy": 288, "width": 640, "height": 480}

    result = {
        "T_cam_in_wrist": {
            "rpy": [round(x, 4) for x in rpy],
            "translation": [round(float(x), 4) for x in T_cam_in_wrist[:3, 3]],
        },
        "camera": cam_info,
        "Z_table": round(float(mean_pos[2]), 4),
        "method": method,
        "num_poses": len(data),
    }
    with open(HAND_EYE_FILE, "w") as f:
        json.dump(result, f, indent=2)

    verification = {
        "target_mean_xyz": [round(float(x), 4) for x in mean_pos],
        "target_std_xyz": [round(float(x), 4) for x in target_positions.std(axis=0)],
        "max_deviation_m": round(float(max_dev), 4),
        "max_deviation_mm": round(float(max_dev * 1000), 2),
    }
    return result, verification
