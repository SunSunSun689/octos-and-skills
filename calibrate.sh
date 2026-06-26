#!/usr/bin/env bash
# SO101 + Gemini 335 眼在手上标定脚本
# 棋盘格 9×6 内角点, 每格 25mm
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Python with mujoco + opencv + pyorbbecsdk
# mujoco is in the conda pico env
PYTHON="${PYTHON:-/home/dora/miniconda3/envs/pico/bin/python3}"

# Orbbec SDK shared libraries
export LD_LIBRARY_PATH="/home/dora/SDK/pyorbbecsdk/install/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/home/dora/SDK/pyorbbecsdk/install/lib:${PYTHONPATH:-}"
MODEL_NAME="${MODEL_NAME:-/home/dora/dorobot2/third_party/robodriver/descriptions/mjcf/so101/so101_new_calib.xml}"
ORBBEC_SN="${ORBBEC_SN:-CP1E542000CJ}"
PAIRS_FILE="${PAIRS_FILE:-/tmp/so101_hand_eye_pairs.json}"
EE_T_CAMERA="${EE_T_CAMERA:-${SCRIPT_DIR}/ee_T_camera.npy}"
BRIDGE_URL="${BRIDGE_URL:-http://127.0.0.1:8768}"

CMD="${1:-help}"

case "$CMD" in
    collect)
        echo "=== 采集标定数据 ==="
        echo "棋盘格固定在桌面不动，移动机械臂到不同姿态"
        echo "每次按 ENTER 记录一个姿态，至少 3 个"
        echo ""
        $PYTHON "${SCRIPT_DIR}/calibrate_hand_eye.py" collect \
            --square-size 0.025 \
            --pattern-cols 9 \
            --pattern-rows 6 \
            --orbbec-sn "$ORBBEC_SN" \
            --bridge "$BRIDGE_URL" \
            --model "$MODEL_NAME" \
            --pairs "$PAIRS_FILE"
        ;;
    solve)
        echo "=== 求解 ee_T_camera ==="
        $PYTHON "${SCRIPT_DIR}/calibrate_hand_eye.py" solve \
            --model "$MODEL_NAME" \
            --pairs "$PAIRS_FILE" \
            --out "$EE_T_CAMERA"
        ;;
    check)
        echo "=== 检查标定质量 ==="
        $PYTHON "${SCRIPT_DIR}/calibrate_hand_eye.py" check \
            --model "$MODEL_NAME" \
            --ee-t-camera "$EE_T_CAMERA" \
            --pairs "$PAIRS_FILE"
        ;;
    all)
        echo "=== 一键采集+求解+检查 ==="
        bash "$0" collect
        bash "$0" solve
        bash "$0" check
        echo ""
        echo "=== 完成 ==="
        echo "export HAND_EYE_MODE=eye_in_hand"
        echo "export EE_T_CAMERA_PATH=${EE_T_CAMERA}"
        ;;
    help|*)
        echo "用法: calibrate.sh <命令>"
        echo ""
        echo "  collect  采集标定数据 (移动机械臂 → ENTER)"
        echo "  solve    求解 AX=XB → ee_T_camera.npy"
        echo "  check    检查标定质量"
        echo "  all      一键执行 collect → solve → check"
        echo ""
        echo "环境变量 (可选):"
        echo "  MODEL_NAME   ${MODEL_NAME}"
        echo "  ORBBEC_SN    ${ORBBEC_SN}"
        echo "  PAIRS_FILE   ${PAIRS_FILE}"
        echo "  EE_T_CAMERA  ${EE_T_CAMERA}"
        ;;
esac
