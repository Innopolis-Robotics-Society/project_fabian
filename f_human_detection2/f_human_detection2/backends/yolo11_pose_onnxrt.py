import numpy as np
import onnxruntime as ort
import cv2
from ..utils import letterbox

class Yolo11PoseONNX:
    def __init__(self, onnx_path: str, input_size=(640,384)):
        providers = ["CUDAExecutionProvider","CPUExecutionProvider"]
        self.sess = ort.InferenceSession(onnx_path, providers=providers)
        self.input_name = self.sess.get_inputs()[0].name
        self.out_names = [o.name for o in self.sess.get_outputs()]
        self.input_w, self.input_h = input_size

    def preprocess(self, img_bgr):
        img, r, dwdh = letterbox(img_bgr, (self.input_w, self.input_h))
        img = img[:, :, ::-1].astype(np.float32) / 255.0
        img = np.transpose(img, (2,0,1))[None]
        return img, r, dwdh

    def infer(self, img_bgr):
        inp, r, dwdh = self.preprocess(img_bgr)
        outs = self.sess.run(self.out_names, {self.input_name: inp})
        if len(outs) == 1:
            o = outs[0]            # [1,56,5040]
            o = o[0].transpose(1,0)  # -> [5040,56]
            boxes = o[:, 0:4]        # xywh
            scores = o[:, 4]         # objectness/person score
            kpts = o[:, 5:56].reshape(-1, 17, 3)  # 51 -> (17,3)
            return (boxes.astype(np.float32), scores.astype(np.float32), kpts.astype(np.float32)), r, dwdh
        else:
            boxes, scores, kpts = outs  # already split
            return (boxes.astype(np.float32), scores.astype(np.float32), kpts.astype(np.float32)), r, dwdh

