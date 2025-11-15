import numpy as np
import onnxruntime as ort
import cv2
from ..utils import letterbox

class Yolo11PoseONNXRT:
    def __init__(self, onnx_path: str, input_size=(640, 384)):
        aps = ort.get_available_providers()
        prefer = ["CUDAExecutionProvider", "AzureExecutionProvider", "CPUExecutionProvider"]
        providers = [p for p in prefer if p in aps]
        self.sess = ort.InferenceSession(onnx_path, providers=providers or None)
        self.input_name = self.sess.get_inputs()[0].name
        self.out_names = [o.name for o in self.sess.get_outputs()]
        self.input_w, self.input_h = input_size

    def preprocess(self, img_bgr):
        img, r, dwdh = letterbox(img_bgr, (self.input_w, self.input_h))
        img = img[:, :, ::-1].astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[None]
        return img, r, dwdh

    def infer(self, img_bgr):
        inp, r, dwdh = self.preprocess(img_bgr)
        outs = self.sess.run(self.out_names, {self.input_name: inp})
        if len(outs) == 1:
            o = outs[0][0].transpose(1, 0)   # [1,56,S] -> [S,56]
            boxes = o[:, :4]; scores = o[:, 4]; kpts = o[:, 5:56].reshape(-1, 17, 3)
        else:
            boxes, scores, kpts = outs
        return (boxes.astype(np.float32), scores.astype(np.float32), kpts.astype(np.float32)), r, dwdh
    def get_chosen_provider(self):
        """
        Retrieve the preferred execution provider from ONNX Runtime.

        Returns:
            str: The name of the chosen provider.
        """
        aps = ort.get_available_providers()
        print(f"Available providers: {aps}")

        prefer = ["CUDAExecutionProvider", "AzureExecutionProvider", "CPUExecutionProvider"]
        chosen_provider = None

        for provider in prefer:
            if provider in aps:
                chosen_provider = provider
                print(f"Chosen provider: {chosen_provider}")
                break
        else:
            print("No preferred providers found, defaulting to 'CPUExecutionProvider'.")
            chosen_provider = "CPUExecutionProvider"

        return chosen_provider