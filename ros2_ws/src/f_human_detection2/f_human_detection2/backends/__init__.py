# not empty

# Factory for selecting backend at runtime.
from .yolo11_pose_trt import Yolo11PoseTRT
from .yolo11_pose_onnxrt import Yolo11PoseONNX
__all__ = ["make_backend"]

def make_backend(kind, engine_path, onnx_path, size):
    k = str(kind).lower()
    if k == "onnxrt":
        from .yolo11_pose_onnxrt import Yolo11PoseONNXRT
        return Yolo11PoseONNXRT(onnx_path, size)
    elif k == "tensorrt":
        try:
            from .yolo11_pose_trt import Yolo11PoseTRT
        except Exception as e:
            raise ImportError(
                "TensorRT backend requires CUDA + PyCUDA. Use backend=onnxrt here."
            ) from e
        return Yolo11PoseTRT(engine_path, size)
    else:
        raise ValueError(f"unknown backend: {kind}")
