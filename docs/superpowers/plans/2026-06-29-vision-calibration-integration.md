# Vision + Calibration Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate USB camera vision (HSV block detection) and hand-eye calibration (chessboard intrinsics + AX=XB solve) from octos-nano into so101-pick-cube, with motion execution via existing MuJoCo IK + dora bridge.

**Architecture:** Copy vision/ and calibration/ modules from octos-nano, adapting camera.py for Linux OpenCV, calib_tools.py to use arm_skills MuJoCo FK instead of octos-nano's kinematics.py, and porting grasp_block core algorithm to use arm_skills._solve() + arm_skills._move() instead of direct serial bus control. Add 6 new tools to main dispatcher and manifest.json.

**Tech Stack:** Python 3, OpenCV 4.11, numpy 2.2, mujoco 3.10, dora bridge HTTP

## Global Constraints

- All paths self-contained within skill directory
- Camera: Orbbec Gemini 335 as RGB camera via OpenCV VideoCapture
- FK: MuJoCo-based (arm_skills.py), not URDF/numpy
- Motion: dora bridge HTTP (:8768), not direct serial
- No new pip dependencies — OpenCV, numpy, mujoco already installed
- Version bump: manifest.json 0.4.0 → 0.5.0

---

### Task 1: Create directory structure and copy static assets

**Files:**
- Create: `vision/__init__.py`
- Create: `calibration/__init__.py`
- Create: `calibration/chessboard_9x6.png` (copy)
- Create: `calibration/camera_intrinsics.json` (template seed)
- Create: `calibration/hand_eye.json` (empty seed)

**Interfaces:**
- Produces: directory structure ready for later tasks

- [ ] **Step 1: Create directories**

```bash
mkdir -p vision calibration calibration/chessboard_images
touch vision/__init__.py calibration/__init__.py
```

- [ ] **Step 2: Copy chessboard image from octos-nano**

```bash
cp /home/dora/.octos/skills/skills/octos-nano/bridge/calibration/chessboard_9x6.png \
   /home/dora/.octos/skills/so101-pick-cube/calibration/chessboard_9x6.png
```

- [ ] **Step 3: Create camera_intrinsics.json seed**

Write to `calibration/camera_intrinsics.json`:

```json
{
  "camera_matrix": [[486.2, 0.0, 320.5], [0.0, 468.4, 287.5], [0.0, 0.0, 1.0]],
  "dist_coeffs": [[0.005, -0.018, 0.003, -0.003, 0.001]],
  "image_size": [640, 480],
  "reprojection_error_px": 0.03,
  "num_images_used": 13,
  "square_size_mm": 25.0,
  "chessboard": [9, 6]
}
```

These are seed values copied from octos-nano; they will be overwritten when the user runs `calibrate_camera`.

- [ ] **Step 4: Create hand_eye.json empty seed**

```json
{}
```

Write to `calibration/hand_eye.json`. The `_load_hand_eye()` equivalent returns None for empty files.

- [ ] **Step 5: Verify files exist**

```bash
ls vision/__init__.py calibration/__init__.py calibration/chessboard_9x6.png calibration/camera_intrinsics.json calibration/hand_eye.json
```

- [ ] **Step 6: Commit**

```bash
git add vision/ calibration/
git commit -m "feat: add vision/ and calibration/ directory structure with static assets

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 2: Create vision/block_detector.py (copy from octos-nano)

**Files:**
- Create: `vision/block_detector.py`

**Interfaces:**
- Produces: `detect_blocks(frame_bgr, target_colors=None) -> list[dict]`, `annotate_frame(frame_bgr, blocks) -> np.ndarray`, `COLOR_RANGES: dict`

- [ ] **Step 1: Write the file**

```python
"""Color-based block detector using HSV thresholding and contour detection."""

import cv2
import numpy as np

COLOR_RANGES = {
    "red":    [(np.array([0, 100, 100]), np.array([10, 255, 255])),
               (np.array([170, 100, 100]), np.array([180, 255, 255]))],
    "green":  [(np.array([40, 80, 60]), np.array([80, 255, 255]))],
    "blue":   [(np.array([100, 120, 60]), np.array([130, 255, 255]))],
    "yellow": [(np.array([20, 100, 100]), np.array([35, 255, 255]))],
    "black":  [(np.array([0, 0, 0]), np.array([180, 255, 50]))],
}

MIN_CONTOUR_AREA = 500


def detect_blocks(frame_bgr, target_colors=None):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    colors = target_colors or list(COLOR_RANGES.keys())
    blocks = []

    for color in colors:
        ranges = COLOR_RANGES.get(color, [])
        mask = None
        for lower, upper in ranges:
            part = cv2.inRange(hsv, lower, upper)
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        if mask is None:
            continue

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            blocks.append({"color": color, "cx": cx, "cy": cy, "width": w, "height": h})

    return blocks


def annotate_frame(frame_bgr, blocks):
    annotated = frame_bgr.copy()
    cmap = {"red": (0, 0, 255), "green": (0, 255, 0),
            "blue": (255, 0, 0), "yellow": (0, 255, 255),
            "black": (128, 128, 128)}
    for b in blocks:
        c = cmap.get(b["color"], (255, 255, 255))
        x, y = b["cx"] - b["width"] // 2, b["cy"] - b["height"] // 2
        w, h = b["width"], b["height"]
        cv2.rectangle(annotated, (x, y), (x + w, y + h), c, 2)
        cv2.circle(annotated, (b["cx"], b["cy"]), 4, c, -1)
        cv2.putText(annotated, f"{b['color']} ({b['cx']},{b['cy']})",
                    (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    return annotated
```

- [ ] **Step 2: Verify import**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && python3 -c "from vision.block_detector import detect_blocks, annotate_frame, COLOR_RANGES; print('OK:', len(COLOR_RANGES), 'colors')"
```

Expected: `OK: 5 colors`

- [ ] **Step 3: Commit**

```bash
git add vision/block_detector.py
git commit -m "feat: add vision/block_detector.py — HSV color block detection

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 3: Add forward_kinematics() and _rpy_to_rot() to arm_skills.py

**Files:**
- Modify: `arm_skills.py`

**Interfaces:**
- Produces: `arm_skills.forward_kinematics(joint_angles_deg, with_gripper=False) -> dict`, `arm_skills._rpy_to_rot(rpy) -> np.ndarray`
- Consumes: existing `_m`, `_d`, `_site`, `ARM_QPOS` from arm_skills

- [ ] **Step 1: Add _rpy_to_rot() helper**

Insert after the `_call` function (line 98, before `_move`):

```python
def _rpy_to_rot(rpy):
    """Convert roll-pitch-yaw (radians) to 3x3 rotation matrix (no scipy needed)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])
```

- [ ] **Step 2: Add forward_kinematics() function**

Insert after `_rpy_to_rot()`, before `_move()`:

```python
def forward_kinematics(joint_angles_deg, with_gripper=False):
    """FK using the loaded MuJoCo model.

    Args:
        joint_angles_deg: 5 joint angles [pan, lift, elbow, flex, roll] in degrees.
        with_gripper: If True, use pinch site (tool tip). If False, use gripper_link (wrist).

    Returns:
        dict with position (x,y,z), rotation (3x3), and transform (4x4).
    """
    q = np.array(joint_angles_deg[:5], dtype=float)
    _d.qpos[:] = 0
    _d.qpos[ARM_QPOS] = q
    mujoco.mj_forward(_m, _d)

    if with_gripper:
        pos = _d.site_xpos[_site].copy()
        rot = _d.site_xmat[_site].reshape(3, 3).copy()
    else:
        wrist_id = mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_BODY, "gripper_link")
        pos = _d.xpos[wrist_id].copy()
        rot = _d.xmat[wrist_id].reshape(3, 3).copy()

    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = pos

    return {"position": (float(pos[0]), float(pos[1]), float(pos[2])),
            "rotation": rot, "transform": T}
```

- [ ] **Step 3: Verify FK returns valid data**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && \
MODEL_NAME=mjcf/so101/so101_new_calib.xml ROBOT_MANIFEST=manifests/so101-hw.json \
python3 -c "
import os, sys
os.environ['MODEL_NAME'] = 'mjcf/so101/so101_new_calib.xml'
os.environ['ROBOT_MANIFEST'] = 'manifests/so101-hw.json'
sys.path.insert(0, '.')
import arm_skills
fk = arm_skills.forward_kinematics([0, 0, 0, 0, 0])
print('position:', fk['position'])
print('transform shape:', fk['transform'].shape)
"
```

Expected: 3-float position tuple, transform shape (4, 4)

- [ ] **Step 4: Commit**

```bash
git add arm_skills.py
git commit -m "feat: add forward_kinematics() and _rpy_to_rot() to arm_skills

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 4: Create vision/camera.py (Linux OpenCV adaptation)

**Files:**
- Create: `vision/camera.py`

**Interfaces:**
- Produces: `Camera` class with `read() -> np.ndarray`, `read_jpeg_base64() -> str`, `close()`

- [ ] **Step 1: Write the file**

```python
"""USB Camera capture via OpenCV VideoCapture (Linux V4L2).

Uses Orbbec Gemini 335 as plain RGB camera. No pyorbbecsdk needed.
Device can be overridden via CAMERA_DEVICE env var (default /dev/video0).
"""

import os
import cv2
import numpy as np
import base64

CAMERA_DEVICE = os.environ.get("CAMERA_DEVICE", "/dev/video0")


class Camera:
    def __init__(self, device=CAMERA_DEVICE, width=640, height=480):
        self.device = device
        self.width = width
        self.height = height
        self._cap = None

    def _ensure_open(self):
        if self._cap is not None:
            return
        for dev in [self.device, 0, 1, 2]:
            cap = cv2.VideoCapture(dev)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            if cap.isOpened():
                self._cap = cap
                self.device = dev
                return
            cap.release()
        raise RuntimeError(
            f"Cannot open camera. Tried {self.device}, indices 0-2. "
            f"Set CAMERA_DEVICE env var or check USB connection."
        )

    def read(self):
        """Capture a single BGR frame. Returns (H, W, 3) numpy array."""
        self._ensure_open()
        ret, frame = self._cap.read()
        if not ret or frame is None:
            raise RuntimeError("Camera read returned no frame. Check USB connection.")
        return frame

    def read_jpeg_base64(self):
        """Capture and return as base64 JPEG string."""
        frame = self.read()
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            raise RuntimeError("Failed to encode frame as JPEG")
        return base64.b64encode(jpeg.tobytes()).decode("utf-8")

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __del__(self):
        self.close()
```

- [ ] **Step 2: Verify import**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && python3 -c "from vision.camera import Camera; print('Camera class imported OK')"
```

Expected: `Camera class imported OK`

- [ ] **Step 3: Commit**

```bash
git add vision/camera.py
git commit -m "feat: add vision/camera.py — OpenCV VideoCapture for Linux

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 5: Create calibration/calib_tools.py (adapted from octos-nano)

**Files:**
- Create: `calibration/calib_tools.py`

**Interfaces:**
- Produces: `capture_chessboard_image()`, `calibrate_camera_intrinsics()`, `collect_hand_eye_pose()`, `solve_hand_eye()`
- Consumes: `arm_skills.forward_kinematics`, `arm_skills._rpy_to_rot`

Adapted from octos-nano `bridge/calibration/calib_tools.py`. Key changes: CALIB_DIR paths, FK import from arm_skills instead of kinematics, numpy-based RPY→matrix instead of scipy.

- [ ] **Step 1: Write the file**

See the full code below. Write to `calibration/calib_tools.py`:

```python
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
```

- [ ] **Step 2: Verify import**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && \
MODEL_NAME=mjcf/so101/so101_new_calib.xml ROBOT_MANIFEST=manifests/so101-hw.json \
python3 -c "from calibration.calib_tools import CHESSBOARD, detect_chessboard; print('calib_tools OK, chessboard:', CHESSBOARD)"
```

Expected: `calib_tools OK, chessboard: (9, 6)`

- [ ] **Step 3: Commit**

```bash
git add calibration/calib_tools.py
git commit -m "feat: add calibration/calib_tools.py — camera + hand-eye calibration

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 6: Add detect_blocks + capture_chessboard tools to main and manifest.json

**Files:**
- Modify: `main`
- Modify: `manifest.json`

These are camera-only tools — no bridge dependency. They validate that vision/ and calibration/ modules work with main dispatcher.

- [ ] **Step 1: Add tool handlers to main**

Open `main`. Add after the `elif tool == "set_gripper":` block (after line 149):

```python
        elif tool == "detect_blocks":
            from vision.camera import Camera
            from vision.block_detector import detect_blocks
            target_colors = args.get("colors", None)
            cam = Camera()
            try:
                frame = cam.read()
                blocks = detect_blocks(frame, target_colors)
                _emit(json.dumps({"blocks": blocks, "count": len(blocks)}), True)
            finally:
                cam.close()

        elif tool == "capture_chessboard":
            from vision.camera import Camera
            from calibration.calib_tools import capture_chessboard_image
            sq_mm = args.get("square_size_mm", 25)
            cam = Camera()
            try:
                frame = cam.read()
                result = capture_chessboard_image(frame, sq_mm)
            finally:
                cam.close()
            if result is None:
                _emit("No chessboard detected. Make sure the 9x6 chessboard is fully visible.", False)
            _emit(json.dumps(result), True)
```

- [ ] **Step 2: Add tool definitions to manifest.json**

Open `manifest.json`. Change `"version": "0.4.0"` to `"version": "0.5.0"`. Add after `place_cube_at` tool entry:

```json
    {
      "name": "detect_blocks",
      "description": "Capture a frame from the RGB camera and detect colored blocks (red, green, blue, yellow, black) using HSV thresholding. Returns each block's color and pixel position. Optionally filter by color.",
      "input_schema": {
        "type": "object",
        "properties": {
          "colors": {
            "type": "array",
            "items": {"type": "string", "enum": ["red", "green", "blue", "yellow", "black"]},
            "description": "Optional list of colors to detect. If omitted, detects all colors."
          }
        }
      }
    },
    {
      "name": "capture_chessboard",
      "description": "Detect a 9x6 chessboard in the current camera frame and save the image for camera intrinsics calibration. Run this 5+ times with the chessboard at different positions/angles, then run calibrate_camera.",
      "input_schema": {
        "type": "object",
        "properties": {
          "square_size_mm": {
            "type": "number",
            "description": "Chessboard square size in millimeters. Default 25."
          }
        }
      }
    },
```

- [ ] **Step 3: Verify tool dispatch**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && echo '{"colors": ["yellow"]}' | python3 main detect_blocks 2>&1
```

Expected: Either blocks JSON (if camera connected) or clear "Cannot open camera" error. NOT "unknown tool".

- [ ] **Step 4: Commit**

```bash
git add main manifest.json
git commit -m "feat: add detect_blocks + capture_chessboard tools

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 7: Add calibrate_camera + hand_eye_collect + hand_eye_solve tools

**Files:**
- Modify: `main`
- Modify: `manifest.json`
- Modify: `arm_skills.py` (`_move()` saves last joint angles to temp file)

`hand_eye_collect` needs current joint angles. In octos-nano these come from direct serial. In so101-pick-cube, every `_move()` call saves commanded angles to `/tmp/so101_last_joints.json`; `hand_eye_collect` reads that file.

- [ ] **Step 1: Save last joint angles in arm_skills._move()**

Replace the existing `_move()` function in `arm_skills.py`:

```python
def _move(q):
    result = _call(MOVE, joints=[float(v) for v in q], control_source="octos")
    try:
        import json as _json
        with open("/tmp/so101_last_joints.json", "w") as _f:
            _json.dump({"shoulder_pan": float(q[0]), "shoulder_lift": float(q[1]),
                        "elbow_flex": float(q[2]), "wrist_flex": float(q[3]),
                        "wrist_roll": float(q[4])}, _f)
    except OSError:
        pass
    return result
```

- [ ] **Step 2: Add tool handlers to main**

Add after `capture_chessboard` handler:

```python
        elif tool == "calibrate_camera":
            from calibration.calib_tools import calibrate_camera_intrinsics
            sq_mm = args.get("square_size_mm", 25)
            try:
                result = calibrate_camera_intrinsics(sq_mm)
            except (FileNotFoundError, ValueError) as e:
                _emit(str(e), False)
            fx = result["camera_matrix"][0][0]
            fy = result["camera_matrix"][1][1]
            cx = result["camera_matrix"][0][2]
            cy = result["camera_matrix"][1][2]
            err = result["reprojection_error_px"]
            _emit(json.dumps({
                "message": f"Camera calibrated: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}, reproj_err={err:.3f}px",
                "intrinsics": result,
            }), True)

        elif tool == "hand_eye_collect":
            from vision.camera import Camera
            from calibration.calib_tools import collect_hand_eye_pose
            sq_mm = args.get("square_size_mm", 25)
            joints_file = "/tmp/so101_last_joints.json"
            if not os.path.exists(joints_file):
                _emit("No joint angles available. Move the arm first with move_to or pick_cube_at.", False)
            with open(joints_file) as f:
                joints = json.load(f)
            cam = Camera()
            try:
                frame = cam.read()
                result = collect_hand_eye_pose(frame, joints, sq_mm)
            finally:
                cam.close()
            if result is None:
                _emit("No chessboard detected. Chessboard must be fixed on table and visible from current arm pose.", False)
            _emit(json.dumps({
                "message": f"Hand-eye pose {result['pose_index'] + 1}/{result['total_poses']} recorded. Total: {result['total_poses']}",
                **result,
                "joints": {k: round(v, 2) for k, v in joints.items()},
            }), True)

        elif tool == "hand_eye_solve":
            from calibration.calib_tools import solve_hand_eye
            method = args.get("method", "park")
            try:
                result, verification = solve_hand_eye(method)
            except (FileNotFoundError, ValueError) as e:
                _emit(str(e), False)
            _emit(json.dumps({
                "message": (
                    f"Hand-eye solved ({method}, {result['num_poses']} poses). "
                    f"Z_table={result['Z_table']:.4f}m. "
                    f"Target consistency: max_dev={verification['max_deviation_mm']:.1f}mm"
                ),
                "calibration": result,
                "verification": verification,
            }), True)
```

- [ ] **Step 3: Add to manifest.json**

Add after `capture_chessboard` entry:

```json
    {
      "name": "calibrate_camera",
      "description": "Run camera intrinsics calibration on all saved chessboard images (captured via capture_chessboard). Requires at least 5 images. Produces camera_intrinsics.json.",
      "input_schema": {
        "type": "object",
        "properties": {
          "square_size_mm": {"type": "number", "description": "Square size in mm. Default 25."}
        }
      }
    },
    {
      "name": "hand_eye_collect",
      "description": "Record one hand-eye calibration pose. Chessboard must be fixed on table. Arm must be at the desired pose (use move_to first). Reads current joint angles, captures frame, detects chessboard, saves pose pair.",
      "input_schema": {
        "type": "object",
        "properties": {
          "square_size_mm": {"type": "number", "description": "Square size in mm. Default 25."}
        }
      }
    },
    {
      "name": "hand_eye_solve",
      "description": "Solve AX=XB hand-eye calibration using collected poses. Requires 6+ poses. Saves hand_eye.json. Methods: tsai, park, horaud, andreff, daniilidis (default: park).",
      "input_schema": {
        "type": "object",
        "properties": {
          "method": {"type": "string", "enum": ["tsai", "park", "horaud", "andreff", "daniilidis"]}
        }
      }
    },
```

- [ ] **Step 4: Verify dispatch**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && echo '{}' | python3 main calibrate_camera 2>&1
```

Expected: Error about no images directory (expected), NOT "unknown tool".

- [ ] **Step 5: Commit**

```bash
git add main manifest.json arm_skills.py
git commit -m "feat: add calibrate_camera + hand_eye_collect + hand_eye_solve tools

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 8: Add grasp_block tool

**Files:**
- Modify: `main`
- Modify: `manifest.json`

Ports core algorithm from octos-nano `handle_grasp_block` (so101_bridge.py:356-538), replacing direct serial bus calls with arm_skills bridge calls.

- [ ] **Step 1: Add grasp_block handler**

Add after `hand_eye_solve` handler in `main`:

```python
        elif tool == "grasp_block":
            import time as _time
            import numpy as _np
            from vision.camera import Camera
            from vision.block_detector import detect_blocks

            color = str(args.get("color", "yellow"))
            he_file = os.path.join(skill_dir, "calibration", "hand_eye.json")
            if not os.path.exists(he_file):
                _emit("No hand-eye calibration. Run hand_eye_collect (6+ poses) then hand_eye_solve first.", False)
            with open(he_file) as f:
                calib = json.load(f)
            if not calib or "T_cam_in_wrist" not in calib:
                _emit("hand_eye.json is empty. Run hand_eye_solve first.", False)

            T_cam_rpy = calib["T_cam_in_wrist"]["rpy"]
            T_cam_trans = calib["T_cam_in_wrist"]["translation"]
            fx, fy = calib["camera"]["fx"], calib["camera"]["fy"]
            cx0, cy0 = calib["camera"]["cx"], calib["camera"]["cy"]
            Z_TABLE = calib.get("Z_table", -0.125)

            R_cam = arm_skills._rpy_to_rot(T_cam_rpy)
            T_cam_in_wrist = _np.eye(4)
            T_cam_in_wrist[:3, :3] = R_cam
            T_cam_in_wrist[:3, 3] = T_cam_trans

            # 1. Home + open gripper
            arm_skills._call(arm_skills.NAMED, name="home", control_source="octos")
            arm_skills._call(arm_skills.GRIP, width=arm_skills.GRIP_OPEN_W)
            _time.sleep(1.5)

            # 2. Detect block
            cam = Camera()
            try:
                frame = cam.read()
            except Exception:
                cam.close()
                _emit("Camera read failed. Check USB connection.", False)
            blocks = detect_blocks(frame, [color])
            if not blocks:
                cam.close()
                _emit(f"No {color} block detected from search position", False)

            block = max(blocks, key=lambda b: b["width"] * b["height"])
            cx, cy = block["cx"], block["cy"]

            # 3. Pixel -> world via ray-plane intersection
            jf = "/tmp/so101_last_joints.json"
            if not os.path.exists(jf):
                cam.close()
                _emit("No joint angles available. Arm may not have moved yet.", False)
            with open(jf) as f:
                cur_joints = json.load(f)
            j5 = [cur_joints[k] for k in ["shoulder_pan", "shoulder_lift",
                    "elbow_flex", "wrist_flex", "wrist_roll"]]
            fk = arm_skills.forward_kinematics(j5, with_gripper=False)
            T_base_to_cam = fk["transform"] @ T_cam_in_wrist

            dx, dy = (cx - cx0) / fx, (cy - cy0) / fy
            a = T_base_to_cam[2, 0] * dx + T_base_to_cam[2, 1] * dy + T_base_to_cam[2, 2]
            b = Z_TABLE - T_base_to_cam[2, 3]

            if abs(a) < 1e-6:
                cam.close()
                _emit("Camera ray parallel to table plane — cannot compute block position", False)
            lam = b / a
            if lam <= 0:
                cam.close()
                _emit(f"Block behind camera (lambda={lam:.2f}) — check calibration", False)

            if cx > cx0:
                Z_EFF = Z_TABLE - 0.020
                b_r = Z_EFF - T_base_to_cam[2, 3]
                lam_r = b_r / a
                if lam_r <= 0:
                    cam.close()
                    _emit(f"Block behind camera (right, lambda={lam_r:.2f})", False)
                P_base = T_base_to_cam @ _np.array([lam_r*dx, lam_r*dy, lam_r, 1.0])
                X_block, Y_block = P_base[0] - 0.005, P_base[1] + 0.085
            else:
                Z_EFF = Z_TABLE
                P_base = T_base_to_cam @ _np.array([lam*dx, lam*dy, lam, 1.0])
                X_block, Y_block = P_base[0] - 0.025, P_base[1] + 0.015

            # 4. IK to block
            target = (float(X_block), float(Y_block), float(Z_EFF) + 0.02)
            ik = arm_skills._solve(list(target), arm_skills.HOME.copy())
            if ik is None:
                cam.close()
                _emit(f"IK failed for target {target}", False)

            # 5. Move + grasp
            arm_skills._move(ik)
            _time.sleep(3.0)
            arm_skills._call(arm_skills.GRIP, width=arm_skills.GRIP_CLOSE_W)
            _time.sleep(3.0)

            # 6. Lift
            lift_ik = arm_skills._solve(
                [float(X_block), float(Y_block), float(Z_EFF) + 0.07], ik)
            if lift_ik is not None:
                arm_skills._move(lift_ik)
                _time.sleep(2.0)

            arm_skills._call(arm_skills.NAMED, name="home", control_source="octos")
            _time.sleep(3.0)

            # 7. Detect drop box and release
            drop_frame = cam.read()
            drop_blocks = detect_blocks(drop_frame, ["black"])
            cam.close()

            dropped = False
            if drop_blocks:
                bb = max(drop_blocks, key=lambda b: b["width"] * b["height"])
                with open(jf) as f:
                    dj = json.load(f)
                j5d = [dj[k] for k in ["shoulder_pan", "shoulder_lift",
                        "elbow_flex", "wrist_flex", "wrist_roll"]]
                fk_d = arm_skills.forward_kinematics(j5d, with_gripper=False)
                T_bc_d = fk_d["transform"] @ T_cam_in_wrist
                dx_d = (bb["cx"] - cx0) / fx
                dy_d = (bb["cy"] - cy0) / fy
                Z_DROP = -0.02
                a_d = T_bc_d[2,0]*dx_d + T_bc_d[2,1]*dy_d + T_bc_d[2,2]
                lam_d = (Z_DROP - T_bc_d[2,3]) / a_d
                P_d = T_bc_d @ _np.array([lam_d*dx_d, lam_d*dy_d, lam_d, 1.0])
                drop_ik = arm_skills._solve(
                    [float(P_d[0]), float(P_d[1]), Z_DROP], j5d)
                if drop_ik is not None:
                    arm_skills._move(drop_ik)
                    _time.sleep(2.0)
                    arm_skills._call(arm_skills.GRIP, width=arm_skills.GRIP_OPEN_W)
                    _time.sleep(1.5)
                    dropped = True

            arm_skills._call(arm_skills.NAMED, name="home", control_source="octos")
            _emit(json.dumps({
                "grasped": True, "color": color, "method": "hand_eye_ik",
                "block_world": [round(float(X_block), 4), round(float(Y_block), 4),
                                round(float(Z_EFF), 4)],
                "target_joints": [round(float(x), 2) for x in ik],
                "dropped": dropped,
            }), True)
```

- [ ] **Step 2: Add to manifest.json**

Add after `hand_eye_solve`:

```json
    {
      "name": "grasp_block",
      "description": "Autonomously pick up a colored block using hand-eye calibration. Sequence: home -> detect block via camera -> compute world position (ray-plane intersection) -> IK solve -> grasp -> lift -> detect black drop box -> release. Requires completed hand-eye calibration.",
      "input_schema": {
        "type": "object",
        "properties": {
          "color": {"type": "string", "enum": ["red", "green", "blue", "yellow", "black"],
                    "description": "Block color to grasp. Default: yellow."}
        }
      }
    }
```

- [ ] **Step 3: Verify dispatch**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && echo '{"color":"yellow"}' | python3 main grasp_block 2>&1
```

Expected: "No hand-eye calibration" error (expected). NOT "unknown tool" or import errors.

- [ ] **Step 4: Commit**

```bash
git add main manifest.json
git commit -m "feat: add grasp_block — autonomous vision-guided pick-and-place

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 9: End-to-end verification

**Files:** (none modified)

- [ ] **Step 1: Existing tools regression**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && echo '' | python3 main get_dropoff_position
```

Expected: `{"output": "x=0.250, y=0.000", "success": true}` (SO-101)

- [ ] **Step 2: All new tools recognized**

```bash
cd /home/dora/.octos/skills/so101-pick-cube
for tool in detect_blocks capture_chessboard calibrate_camera hand_eye_collect hand_eye_solve grasp_block; do
  echo "--- $tool ---"
  echo '{}' | python3 main "$tool" 2>&1 | head -1
done
```

Expected: All 6 respond (with expected errors like "No camera" or "No calibration"), none say "unknown tool".

- [ ] **Step 3: manifest.json count**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && python3 -c "import json; m=json.load(open('manifest.json')); print(f'{len(m[\"tools\"])} tools, version {m[\"version\"]}')"
```

Expected: `10 tools, version 0.5.0` (4 existing + 6 new)

- [ ] **Step 4: Full import test**

```bash
cd /home/dora/.octos/skills/so101-pick-cube && \
MODEL_NAME=mjcf/so101/so101_new_calib.xml ROBOT_MANIFEST=manifests/so101-hw.json \
python3 -c "
import os, sys
os.environ['MODEL_NAME'] = 'mjcf/so101/so101_new_calib.xml'
os.environ['ROBOT_MANIFEST'] = 'manifests/so101-hw.json'
sys.path.insert(0, '.')
import arm_skills
from vision.camera import Camera
from vision.block_detector import detect_blocks, annotate_frame
from calibration.calib_tools import (capture_chessboard_image, calibrate_camera_intrinsics,
    collect_hand_eye_pose, solve_hand_eye, CHESSBOARD)
fk = arm_skills.forward_kinematics([0, 0, 0, 0, 0])
print(f'All imports OK. FK position: {fk[\"position\"]}')
print('READY')
"
```

Expected: `All imports OK... READY`

- [ ] **Step 5: Commit if changes**

```bash
git status
# If clean: done. If changes from verification fixes:
git add -A && git commit -m "chore: verification fixes after integration

Co-Authored-By: Claude <noreply@anthropic.com>"
```
