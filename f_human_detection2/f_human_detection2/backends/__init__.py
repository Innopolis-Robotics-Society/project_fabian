# not empty

# Factory for selecting backend at runtime.
from .yolo11_pose_trt import Yolo11PoseTRT
from .yolo11_pose_onnxrt import Yolo11PoseONNX

# Lazy factory. Imports backend only when requested.
def make_backend(kind: str, engine_path: str, onnx_path: str, input_size):
    if kind == "tensorrt":
        from .yolo11_pose_trt import Yolo11PoseTRT
        return Yolo11PoseTRT(engine_path, input_size)
    elif kind == "onnxrt":
        from .yolo11_pose_onnxrt import Yolo11PoseONNX
        return Yolo11PoseONNX(onnx_path, input_size)
    else:
        raise ValueError(f"Unknown backend: {kind}")

