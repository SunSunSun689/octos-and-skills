---
name: so101-pick-cube
description: SO-101 and Adora (1.15× scaled SO-101) pick-and-place of a cube on real hardware. Cube (x, y, z) coordinates in base frame are provided by the caller — no camera dependency. Reuses the SPEC-VENDOR-NODE-V1 bridge at :8768 and arm_skills.py IK.
version: 0.3.0
author: dorarobotics
robot_type: so101, adora
required_safety_tier: safe_motion
hardware_requirements: so-arm101, feetech-sts3215, lerobot, dora-moveit2
preflight:
  - label: check arm serial bus
    command: bash -c 'test -e "${SO101_PORT:-/dev/ttyACM0}" || (echo "FATAL: no arm bus at ${SO101_PORT:-/dev/ttyACM0}" && exit 1)'
    timeout_secs: 5
    critical: true
init:
  - label: start dora hardware bridge
    command: |
      BRIDGE_DIR="/home/dora/.octos/skills/skills/octos-dora-bridge"
      cd "$BRIDGE_DIR"
      # Start dora daemon if not running
      dora list > /dev/null 2>&1 || dora up
      # Start dataflow (idempotent if already running)
      dora start so101-hw-bridge-resolved.yaml --detach 2>/dev/null || true
      # Wait for bridge to become ready
      for i in $(seq 1 30); do
        curl -fsS -m 2 http://127.0.0.1:8768/healthz > /dev/null 2>&1 && break
        sleep 1
      done
    timeout_secs: 60
    critical: true
  - label: move arm to home
    command: |
      curl -fsS -X POST http://127.0.0.1:8768/tools/vendor.moveit.arm.move_to_named \
        -H "Content-Type: application/json" \
        -d '{"args":{"name":"home"}}' \
        -m 30
    timeout_secs: 45
    critical: true
ready_check:
  - label: bridge HTTP responds
    command: curl -fsS -m 2 http://127.0.0.1:8768/healthz
    timeout_secs: 3
    retries: 10
    critical: true
shutdown:
  - label: stop dora dataflow
    command: dora stop 2>/dev/null || true
    timeout_secs: 15
    critical: false
emergency_shutdown:
  - label: estop via bridge then stop dora
    command: |
      curl -fsS -X POST http://127.0.0.1:8768/tools/robot.estop \
        -H "Content-Type: application/json" \
        -d '{"args":{"reason":"emergency"}}' \
        -m 5 2>/dev/null || true
      dora stop 2>/dev/null || true
      fi
    timeout_secs: 10
    critical: true
---

# SO-101 / Adora cube pick-and-place (real hardware, no camera)

This skill controls an SO-101 or Adora (1.15× scaled SO-101) arm to pick up a cube
and place it on a target plate. **Cube (x, y, z) coordinates are provided by the
caller** (LLM or upstream system) — there is no camera/vision dependency.

The arm control reuses the same SPEC bridge as the sim `so101` skill:

- The bridge listens on `http://127.0.0.1:8768` and exposes the SPEC-VENDOR-NODE-V1
  verbs (`vendor.moveit.arm.move_to_joint_state`, `vendor.moveit.arm.move_to_named`,
  `vendor.moveit.arm.gripper.set`).
- `arm_skills.py` does on-demand MuJoCo IK against the arm-specific `MODEL_NAME` and
  drives the bridge over HTTP.

## Robot selection

Use `--robot` to switch between supported arms:

| Flag | MODEL_NAME | ROBOT_MANIFEST |
|------|-----------|----------------|
| `--robot so101` (default) | `mjcf/so101/so101_new_calib.xml` | `manifests/so101-hw.json` |
| `--robot adora` | `mjcf/adora/adora.xml` | `manifests/adora-hw.json` |

```bash
# SO-101 (default)
echo '{"x":0.29,"y":0,"z":0.02}' | python3 main pick_cube_at

# Adora (1.15× workspace)
echo '{"x":0.33,"y":0,"z":0.02}' | python3 main --robot adora pick_cube_at
```

Or override via environment variables for custom configs:
```bash
MODEL_NAME=/path/to/custom.xml ROBOT_MANIFEST=/path/to/custom.json python3 main pick_cube_at
```

## Adora vs SO-101

Adora is a 1.15× physically scaled SO-101 — same kinematic chain (5-DOF, STS3215
servos, same joint names and ranges). The only differences are:

- MuJoCo XML: all body pos ×1.15 (larger reach)
- Manifest: height/grasp/place/lift parameters ×1.15
- Joint angles, servo params, gripper config: identical

Adora's workspace is approximately 15% larger in each dimension.

## Vision source code (preserved but not used)

The following files are kept as reference but are NOT launched or called by this skill:

- `vision_service.py` — Orbbec Gemini 335 RGB-D camera driver (HTTP :8779)
- `find_cube.py` — cube detection from depth frames
- `calibrate_extrinsics.py` / `calibrate_hand_eye.py` / `auto_calibrate.py` — hand-eye calibration tools
- `calibrate.sh` / `start_vision.sh` — convenience launchers
- `ee_T_camera.npy` / `ee_T_camera_4pairs.npy` / `ee_T_camera_backup.npy` — calibration results

If you need vision-based cube detection, run `vision_service.py` manually (requires
pyorbbecsdk 1.10.22 + PYTHONPATH + LD_LIBRARY_PATH setup). See the git history of
SKILL.md for the full vision workflow documentation.

## Tools

| Tool | Input | Output |
|------|-------|--------|
| `get_dropoff_position` | *(none)* | `x=0.250, y=0.000` (plate position from manifest) |
| `pick_cube_at` | `{"x": 0.15, "y": 0.05, "z": 0.02}` | picks cube at given base-frame coords |
| `place_cube_at` | `{"x": 0.25, "y": 0.0}` | places held cube at target |

## Environment

| Env var | Default | Purpose |
|---|---|---|
| `SKILL_PACK` | `<skill-dir>` | where `arm_skills.py` and `manifest.py` live |
| `MODEL_NAME` | `<skill-dir>/mjcf/so101/so101_new_calib.xml` | MuJoCo XML for IK (Adora: `mjcf/adora/adora.xml`) |
| `ROBOT_MANIFEST` | `<skill-dir>/manifests/so101-hw.json` | arm_skills per-robot tuning (Adora: `manifests/adora-hw.json`) |
| `ARM_BRIDGE_URL` | `http://127.0.0.1:8768` | SPEC bridge |

## Motion verbs

Same surface as the sim `so101` skill — the bridge and arm_skills don't change:

- `vendor.moveit.arm.move_to_joint_state` — `{"joints": [j1..j5]}` (radians).
  Joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.
- `vendor.moveit.arm.move_to_named` — `{"name": "home"|"safe"|"zero"|"up"}`.
- `vendor.moveit.arm.gripper.set` — `{"width": 0.03}` (meters; 0 = closed).
- `vendor.moveit.arm.gripper.open` / `vendor.moveit.arm.gripper.close`.

The SO-101 is 5-DOF; the pick-and-place skill handles orientation automatically
(down-only grasp IK).

## Common operations

- **Dropoff position:** `main get_dropoff_position` → `"x=0.250, y=0.000"` (Adora: `"x=0.288, y=0.000"`).
- **Pick (SO-101):** `main pick_cube_at` with `{"x": 0.142, "y": 0.038, "z": 0.025}` on stdin.
- **Pick (Adora):** `main --robot adora pick_cube_at` with `{"x": 0.163, "y": 0.044, "z": 0.029}` on stdin.
- **Place (SO-101):** `main place_cube_at` with `{"x": 0.250, "y": 0.000}` on stdin.
- **Place (Adora):** `main --robot adora place_cube_at` with `{"x": 0.288, "y": 0.000}` on stdin.
- **Home:** `POST /tools/vendor.moveit.arm.move_to_named` `{"args":{"name":"home"}}`.
- **Emergency stop:** `POST /tools/robot.estop` — disables servo torque on the bus.

## Error codes

| Code | Meaning | What to do |
|---|---|---|
| `VENDOR_ERROR` | planning/execution failure (IK no-solution, etc.) | surface `msg`; try another pose |
| `CONTROLLER_BUSY` | another caller holds the motion slot | `robot.release_control` then retry |
| `BRIDGE_TIMEOUT` | no `cmd_response` within timeout | raise `CMD_TIMEOUT_S` |
| `BRIDGE_DOWN` | bridge lost its dora session | operator-side recovery |
