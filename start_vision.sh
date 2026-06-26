#!/usr/bin/env bash
set -e
export LD_LIBRARY_PATH=/home/dora/SDK/pyorbbecsdk/install/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=/home/dora/SDK/pyorbbecsdk/install/lib:${PYTHONPATH}
export HAND_EYE_MODE=eye_in_hand
export EE_T_CAMERA_PATH=/home/dora/.octos/skills/so101-pick-cube/ee_T_camera.npy
export MODEL_NAME=/home/dora/dorobot2/third_party/robodriver/descriptions/mjcf/so101/so101_new_calib.xml
export CUBE_HEIGHT_M=0.030
export CUBE_TOL_M=0.025
export MIN_CUBE_PX=100
export MAX_CUBE_PX=10000
export MIN_DEPTH_M=0.03
exec /home/dora/miniconda3/envs/pico/bin/python3 \
    /home/dora/.octos/skills/so101-pick-cube/vision_service.py \
    --orbbec-sn CP1E542000CJ --port 8779
