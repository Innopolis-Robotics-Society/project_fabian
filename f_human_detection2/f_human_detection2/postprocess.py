import numpy as np
from .utils import nms, scale_coords_kpts

COCO17_EDGES = [
    (5,7),(7,9), (6,8),(8,10), (5,6),
    (11,13),(13,15), (12,14),(14,16),
    (5,11),(6,12), (11,12)
]
COCO17_BODY_INDEXES = list(range(17))  # already body-only

def decode_yolo_pose(outputs, conf_thr=0.25, iou_thr=0.45, max_persons=20, orig_size=None, ratio=1.0, dwdh=(0,0)):
    """
    Expect outputs as:
      boxes: (N,4) xywh in letterbox space
      scores: (N,) person class score
      kpts: (N,17,3) [x,y,confidence] in letterbox space
    Return trimmed arrays mapped to original image size.
    """
    boxes, scores, kpts = outputs
    if boxes is None or len(boxes) == 0:
        return np.zeros((0,4)), np.zeros((0,)), np.zeros((0,17,3))
    m = scores >= conf_thr
    boxes = boxes[m]; scores = scores[m]; kpts = kpts[m]
    if len(boxes) == 0:
        return np.zeros((0,4)), np.zeros((0,)), np.zeros((0,17,3))
    keep = nms(boxes, scores, iou_thr, max_persons)
    boxes = boxes[keep]; scores = scores[keep]; kpts = kpts[keep]
    if orig_size is not None:
        w, h = orig_size
        kpts = scale_coords_kpts(kpts, ratio, dwdh, w, h)

        # map boxes back from letterbox to original image
        bx = boxes.copy().astype(np.float32)

        # 1) undo letterbox: scale all 4 coords
        # boxes are (cx, cy, w, h) in letterbox space
        bx[:, 0] = (bx[:, 0] - dwdh[0]) / ratio  # cx
        bx[:, 1] = (bx[:, 1] - dwdh[1]) / ratio  # cy
        bx[:, 2] = bx[:, 2] / ratio              # w
        bx[:, 3] = bx[:, 3] / ratio              # h

        # 2) convert center (cx,cy,w,h) -> top-left (x,y,w,h)
        bx[:, 0] = bx[:, 0] - bx[:, 2] / 2.0     # x_tl
        bx[:, 1] = bx[:, 1] - bx[:, 3] / 2.0     # y_tl

        boxes = bx

    return boxes, scores, kpts
