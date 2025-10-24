#!/usr/bin/env bash
# Prod setup for Jetson Xavier (TensorRT). Builds .plan via trtexec.
# Usage:
#   ./scripts/prepare_prod.sh /abs/path/yolov11m-pose.onnx [images]  # второй арг = имя входа в ONNX
#   ./scripts/prepare_prod.sh                                       # если ONNX уже лежит в models/
set -euo pipefail

WS="${WS:-$PWD}"
PKG_DIR="$WS/src/f_human_detection2"
MODELS_DIR="$PKG_DIR/f_human_detection2/models"
ONNX_SRC="${1:-}"
INPUT_NAME="${2:-images}"
ONNX_DST="$MODELS_DIR/yolov11m-pose.onnx"
PLAN_DST="$MODELS_DIR/yolov11m-pose.plan"

echo "[1/8] Source ROS 2 Humble"
set +u; source /opt/ros/humble/setup.bash; set -u

echo "[2/8] System deps (Jetson)"
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev python3-venv \
  ros-humble-cv-bridge ros-humble-vision-msgs
# JetPack packages
sudo apt-get install -y python3-pycuda || true
# libnvinfer* обычно предустановлены с JetPack; пропустим, если нет
sudo apt-get install -y libnvinfer8 libnvinfer-plugin8 python3-libnvinfer || true

echo "[3/8] Python deps"
python3 -m pip install --user --upgrade pip
python3 -m pip install --user numpy opencv-python-headless onnx
export PATH="$HOME/.local/bin:$PATH"

echo "[4/8] Models dir"
mkdir -p "$MODELS_DIR"
if [[ -n "${ONNX_SRC}" && -f "${ONNX_SRC}" ]]; then
  cp -f "${ONNX_SRC}" "${ONNX_DST}"
fi
[[ -f "${ONNX_DST}" ]] || { echo "ERROR: missing ${ONNX_DST}"; exit 2; }

echo "[5/8] Build TensorRT engine"
TRT=/usr/src/tensorrt/bin/trtexec
[[ -x "${TRT}" ]] || { echo "ERROR: ${TRT} not found"; exit 3; }
"${TRT}" \
  --onnx="${ONNX_DST}" \
  --saveEngine="${PLAN_DST}" \
  --explicitBatch \
  --fp16 \
  --workspace=4096 \
  --shapes="${INPUT_NAME}:1x3x384x640"

echo "[6/8] Verify engine"
"${TRT}" --loadEngine="${PLAN_DST}" --profilingVerbosity=detailed --separateProfileRun --dumpProfile > /dev/null

echo "[7/8] Build workspace"
cd "$WS"
rm -rf build/ install/ log/
colcon build --symlink-install --packages-select f_human_detection_msgs f_human_detection2

echo "[8/8] Sanity"
set +u; source install/setup.bash; set -u
python3 - <<'PY'
import importlib
m = importlib.import_module('f_human_detection2.pose_node')
print("OK: f_human_detection2 import (prod)")
PY

cat <<'MSG'
Run prod:
  source install/setup.bash
  ros2 launch f_human_detection2 pose_prod.launch.py f_human_detection2.backend:=tensorrt

Notes:
- Engine path used: f_human_detection2/models/yolov11m-pose.plan
- If input name differs from 'images', pass it as 2nd arg.
MSG
