# Minimal TensorRT inference wrapper. Assumes engine outputs person-only detections with kpts.
# If your export keeps multi-class, filter 'person' in postprocess at decode().

import numpy as np
try:
    import pycuda.autoinit  # noqa: F401
    import pycuda.driver as cuda
except Exception as e:
    raise ImportError("TensorRT backend requires CUDA + PyCUDA. Use backend=onnxrt here.") from e
import tensorrt as trt
import cv2
from ..utils import letterbox

class Yolo11PoseTRT:
    def __init__(self, engine_path: str, input_size=(640,384)):
        self.logger = trt.Logger(trt.Logger.ERROR)
        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.input_w, self.input_h = input_size
        self.stream = cuda.Stream()
        # assume single input and three outputs: boxes, scores, kpts
        self.bindings = [None] * self.engine.num_bindings
        self.inputs = []
        self.outputs = []
        self.alloc_buffers()

    def alloc_buffers(self):
        for i in range(self.engine.num_bindings):
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            shape = self.engine.get_binding_shape(i)
            size = int(np.prod(shape))
            device_mem = cuda.mem_alloc(size * np.dtype(dtype).itemsize)
            self.bindings[i] = int(device_mem)
            if self.engine.binding_is_input(i):
                self.inputs.append((i, device_mem, dtype, shape))
            else:
                host_mem = cuda.pagelocked_empty(size, dtype)
                self.outputs.append((i, device_mem, host_mem, dtype, shape))

    def preprocess(self, img_bgr):
        img, r, dwdh = letterbox(img_bgr, (self.input_w, self.input_h))
        img = img[:, :, ::-1]  # BGR->RGB
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2,0,1))[None]  # 1x3xHxW
        return img, r, dwdh

    def infer(self, img_bgr):
        inp, r, dwdh = self.preprocess(img_bgr)
        d_idx, d_mem, d_type, d_shape = self.inputs[0]
        cuda.memcpy_htod_async(d_mem, inp.astype(d_type).ravel(), self.stream)
        self.context.execute_async_v2(self.bindings, self.stream.handle, None)
        # copy outputs
        out_tensors = []
        for (i, d_mem, h_mem, d_type, shape) in self.outputs:
            cuda.memcpy_dtoh_async(h_mem, d_mem, self.stream)
            out_tensors.append(np.array(h_mem).reshape(shape))
        self.stream.synchronize()
        # expected shapes: (N,4), (N,), (N,17,3)
        boxes, scores, kpts = out_tensors
        return (boxes.astype(np.float32), scores.astype(np.float32), kpts.astype(np.float32)), r, dwdh
