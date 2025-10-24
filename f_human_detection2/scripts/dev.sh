#!/usr/bin/env bash
# Dev setup for f_human_detection2 on x86 (no TensorRT). Uses ONNXRuntime/CPU.
# Usage:
#   ./scripts/prepare_dev.sh [/abs/path/yolo11m-pose.pt]
#   ./scripts/prepare_dev.sh  # if ONNX уже готов
set -euo pipefail

WS="${WS:-$PWD}"
PKG_DIR="$WS/src/f_human_detection2"
MODELS_DIR="$PKG_DIR/f_human_detection2/models"
ONNX="$MODELS_DIR/yolov11m-pose.onnx"

echo "[1/6] Source ROS 2 Humble"
set +u; source /opt/ros/humble/setup.bash; set -u

echo "[2/6] System deps"
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev python3-venv \
  ros-humble-cv-bridge ros-humble-vision-msgs

echo "[3/6] Python deps"
python3 -m pip install --user --upgrade pip
python3 -m pip install --user numpy opencv-python-headless onnx ultralytics onnxruntime
export PATH="$HOME/.local/bin:$PATH"

echo "[4/6] Model (ONNX)"
mkdir -p "$MODELS_DIR"
PT="${1:-}"
if [[ -n "${PT}" && -f "${PT}" ]]; then
  python3 - <<PY
from ultralytics import YOLO
m = YOLO("${PT}")
m.export(format="onnx", imgsz=(384,640), opset=13, simplify=True, dynamic=False)
print("export done")
PY
  FOUND=$(find . -maxdepth 6 -name "yolo11m-pose.onnx" -o -name "*pose*.onnx" | head -n1 || true)
  if [[ -n "${FOUND}" ]]; then cp -f "${FOUND}" "${ONNX}"; fi
fi
[[ -f "${ONNX}" ]] || echo "WARN: missing ${ONNX}"

echo "[5/6] Build"
cd "$WS"
rm -rf build/ install/ log/
colcon build --symlink-install --packages-select f_human_detection_msgs f_human_detection2

echo "[6/6] Sanity"
set +u; source install/setup.bash; set -u
# форсируем backend=onnxrt для дев-сессии
yq_cmd=$(command -v yq || true)
if [[ -n "${yq_cmd}" && -f "$PKG_DIR/config/dev.yaml" ]]; then
  $yq_cmd -i '.f_human_detection2.ros__parameters.backend="onnxrt"' "$PKG_DIR/config/dev.yaml" || true
fi
python3 - <<'PY'
import importlib; import os
m = importlib.import_module('f_human_detection2.pose_node')
print("OK: f_human_detection2 import (dev)")
PY

cat <<'MSG'
Run dev:
  source install/setup.bash
  ros2 launch f_human_detection2 pose_dev.launch.py f_human_detection2.backend:=onnxrt
MSG
