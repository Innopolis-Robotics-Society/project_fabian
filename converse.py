# check_onnx.py
from pathlib import Path
import onnxruntime as ort
import numpy as np

onnx_path = Path("PMLDL/project_fabian/yolo11m-pose.onnx").resolve()  # ← подставь найденный путь
assert onnx_path.is_file(), f"ONNX not found: {onnx_path}"

avail = ort.get_available_providers()
providers = ["CUDAExecutionProvider","CPUExecutionProvider"]
providers = [p for p in providers if p in avail]
print("providers:", avail, "using:", providers)

sess = ort.InferenceSession(str(onnx_path), providers=providers or None)
name = sess.get_inputs()[0].name
y = sess.run(None, {name: np.zeros((1,3,384,640), np.float32)})
print("num_outputs:", len(y), "shapes:", [a.shape for a in y])
