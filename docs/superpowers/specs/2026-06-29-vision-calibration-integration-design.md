# Design: Integrate Vision + Hand-Eye Calibration from octos-nano into so101-pick-cube

Date: 2026-06-29
Status: approved

## Problem

so101-pick-cube operates in "no camera" mode — cube (x, y, z) coordinates must be provided by the caller. It lacks object detection, camera-based calibration, and autonomous grasp-from-pixel capability. The old Orbbec depth-camera pipeline (vision_service.py, find_cube.py) requires pyorbbecsdk 1.10.22 and is not maintained.

octos-nano (sibling skill at `skills/so101-arm`) has a mature, working vision + hand-eye calibration pipeline using RGB camera, HSV color block detection, chessboard-based camera intrinsics calibration, and OpenCV AX=XB hand-eye solving. But it controls the arm via direct serial port, incompatible with so101-pick-cube's dora bridge architecture.

## Goal

Copy octos-nano's vision and calibration code into so101-pick-cube, adapting the execution layer to use so101-pick-cube's existing MuJoCo IK + dora bridge HTTP infrastructure. The result is a single self-contained skill with both camera-based autonomy and bridge-based motion control.

## Architecture

```
octos/LLM
  │
  ▼
main (tool dispatcher)
  ├── Existing tools: get_dropoff_position, pick_cube_at, place_cube_at, move_to, set_gripper
  └── New tools: detect_blocks, calibrate_camera, hand_eye_collect, hand_eye_solve, grasp_block
       │
       ├── vision/camera.py          ← from octos-nano, Linux OpenCV adaptation
       ├── vision/block_detector.py  ← from octos-nano, unchanged
       ├── calibration/calib_tools.py← from octos-nano, FK import → arm_skills
       │
       └── arm_skills.py (existing + new FK function)
            ├── _solve()              MuJoCo IK        (existing)
            ├── _move()               bridge HTTP      (existing)
            ├── _call()               bridge HTTP      (existing)
            └── forward_kinematics()  MuJoCo FK        (NEW)
```

Core principle: **bring vision + calibration logic from octos-nano, wire to so101-pick-cube's motion engine.**

## Hardware

- Camera: Orbbec Gemini 335 used as plain RGB camera via OpenCV VideoCapture (V4L2 on Linux)
- No pyorbbecsdk dependency — depth data not needed for HSV block detection or chessboard calibration

## Files to add

| File | Source | Changes |
|------|--------|---------|
| `vision/__init__.py` | New | Empty |
| `vision/camera.py` | octos-nano `vision/camera.py` | Replace ffmpeg AVFoundation with OpenCV VideoCapture for Linux |
| `vision/block_detector.py` | octos-nano `vision/block_detector.py` | None (pure OpenCV, cross-platform) |
| `calibration/__init__.py` | New | Empty |
| `calibration/calib_tools.py` | octos-nano `bridge/calibration/calib_tools.py` | Fix CALIB_DIR paths; change FK import to arm_skills.forward_kinematics |
| `calibration/chessboard_9x6.png` | octos-nano `bridge/calibration/chessboard_9x6.png` | None (binary copy) |
| `calibration/camera_intrinsics.json` | octos-nano `bridge/calibration/camera_intrinsics.json` | Copy as template |
| `calibration/hand_eye.json` | octos-nano `bridge/calibration/hand_eye.json` | Copy as template (empty/seed) |

## Files to modify

### arm_skills.py

Add one public function (~20 lines):

```python
def forward_kinematics(joint_angles_deg, with_gripper=False):
    """FK using the loaded MuJoCo model.
    Returns: {"position": (x,y,z), "rotation": 3x3, "transform": 4x4}
    """
```

Uses the already-loaded `_m`, `_d`, `_site` — no new model loading. Returns same dict shape as octos-nano's kinematics.py so calib_tools.py can consume it without logic changes.

### main

Add 6 new `elif tool == "..."` branches (capture_chessboard, detect_blocks, calibrate_camera, hand_eye_collect, hand_eye_solve, grasp_block). Vision and calibration modules are imported lazily (only when the tool is invoked) to avoid loading camera/model when not needed:

```python
elif tool == "detect_blocks":
    from vision.camera import Camera
    from vision.block_detector import detect_blocks
    ...

elif tool == "grasp_block":
    from vision.camera import Camera
    from vision.block_detector import detect_blocks
    # Core logic ported from octos-nano so101_bridge.py:356-538
    # But execution uses arm_skills._solve() + arm_skills._move()
    ...
```

### manifest.json

Add 6 new tool definitions. Bump version to 0.5.0.

## New tools

### capture_chessboard

Detect 9×6 chessboard in frame, save image for intrinsics calibration. User moves chessboard to different positions/angles, calls this repeatedly (≥5 times), then runs calibrate_camera.

- Input: `{"square_size_mm": 25}`
- Output: `{"index": 0, "total_images": 1, "corners_found": 54}`
- No bridge/arm dependency — camera only

### detect_blocks

Capture frame, run HSV color block detection.

- Input: `{"colors": ["yellow"]}` (optional filter)
- Output: `{"blocks": [{"color":"yellow","cx":320,"cy":240,"width":80,"height":80}], "count":1}`
- No bridge/arm dependency — camera only

### calibrate_camera

Run OpenCV calibrateCamera on saved chessboard images.

- Input: `{"square_size_mm": 25}`
- Output: `{"message":"Camera calibrated: fx=486.2...","intrinsics":{...}}`
- Requires: ≥5 chessboard images captured via `capture_chessboard` (the capture_chessboard tool itself runs offline — user moves chessboard, runs it manually)

### hand_eye_collect

At current arm pose: read joints via bridge, detect fixed chessboard via solvePnP, save pose pair.

- Input: `{"square_size_mm": 25}`
- Output: pose index + total count
- Chessboard must be fixed on table; arm moved to different poses between calls

### hand_eye_solve

Solve AX=XB on all collected poses.

- Input: `{"method": "park"}` (tsai/park/horaud/andreff/daniilidis)
- Output: `{"calibration": {...T_cam_in_wrist...}, "verification": {...max_deviation_mm...}}`
- Requires: ≥6 hand_eye_collect poses
- Saves hand_eye.json

### grasp_block

Full autonomous pick: detect → pixel→world → IK → grasp → place.

- Input: `{"color": "yellow"}`
- Output: grasp result with block world position and joint angles
- Pipeline:
  1. home + open gripper (bridge)
  2. capture frame + detect_blocks(color)
  3. pixel → world via ray-plane intersection (FK + T_cam_in_wrist from hand_eye.json)
  4. IK solve via arm_skills._solve()
  5. move to block, close gripper, lift, home (bridge)
  6. detect black drop box, compute its world position, IK to it, release
- Requires: valid hand_eye.json (run calibration first)

## grasp_block core algorithm (ported from octos-nano)

```
1. Detect block at pixel (cx, cy)
2. Read current joints → FK → T_base_to_wrist
3. T_base_to_cam = T_base_to_wrist @ T_cam_in_wrist
4. Camera ray: dx=(cx-cx0)/fx, dy=(cy-cy0)/fy
5. Intersect ray with Z=Z_table plane → (X_block, Y_block, Z_table)
6. Left/right correction (table tilt — preserved from octos-nano)
7. IK: arm_skills._solve([X_block, Y_block, Z+0.02], HOME)
8. Execute via arm_skills._move()
```

## Error handling

- No hand_eye.json → tell user to run calibration first
- Block not detected → surface error with color name
- Ray parallel to table (a≈0) → fail with explanation
- Block behind camera (λ≤0) → fail, suggest recalibration
- IK no solution → surface target coordinates, suggest moving block closer

## Verification

1. **Import test**: `python3 -c "from vision.block_detector import detect_blocks; from calibration import calib_tools"` — no import errors
2. **FK test**: `python3 -c "import arm_skills; print(arm_skills.forward_kinematics([0,0,0,0,0]))"` — returns valid transform
3. **Camera test**: `python3 -c "from vision.camera import Camera; c = Camera(); print(c.read().shape)"` — (480, 640, 3)
4. **Regression**: `main get_dropoff_position` — existing tools still work
5. **End-to-end** (hardware): calibrate_camera → hand_eye_collect ×6 → hand_eye_solve → grasp_block
