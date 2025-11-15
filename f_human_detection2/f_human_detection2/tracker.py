import numpy as np
from collections import deque


def iou_xywh(box, boxes):
    """
    Compute IoU between one box (x,y,w,h) and an array of boxes (N,4) in xywh.
    Returns array of shape (N,).
    """
    x, y, w, h = box
    xx, yy, ww, hh = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]

    x1 = np.maximum(x, xx)
    y1 = np.maximum(y, yy)
    x2 = np.minimum(x + w, xx + ww)
    y2 = np.minimum(y + h, yy + hh)

    inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    union = w * h + ww * hh - inter + 1e-9

    return inter / union
    
class SimpleByteLike:
    """
    Minimalistic tracker.
    - Assign IDs via IoU of bounding boxes across frames.
    - Keeps short-term memory.
    Good enough for prototyping.
    """
    def __init__(self, max_age=30, iou_th=0.3):
        self.tracks = {}     # id -> (bbox, age)
        self.history = {}    # id -> deque
        self.next_id = 1
        self.max_age = max_age
        self.iou_th = iou_th

    def reset(self):
        """Drop all tracks and restart IDs from 1."""
        self.tracks.clear()
        self.history.clear()
        self.next_id = 1

    def update(self, boxes):
        assigned = set()
        ids = list(self.tracks.keys())
        new_tracks = {}
        result_ids = [-1]*len(boxes)
        # compute IoU matrix
        if ids and len(boxes):
            M = np.zeros((len(ids), len(boxes)), dtype=np.float32)
            for i, tid in enumerate(ids):
                M[i] = iou_xywh(self.tracks[tid][0], boxes)
            # greedy assign
            for _ in range(min(len(ids), len(boxes))):
                i,j = np.unravel_index(np.argmax(M), M.shape)
                if M[i,j] < self.iou_th: break
                tid = ids[i]
                new_tracks[tid] = (boxes[j], 0)
                result_ids[j] = tid
                assigned.add(j)
                M[i,:] = -1; M[:,j] = -1

        # new IDs
        for j, b in enumerate(boxes):
            if j in assigned: continue
            tid = self.next_id; self.next_id += 1
            new_tracks[tid] = (b, 0); result_ids[j] = tid

        # age unassigned
        for tid,(b,age) in self.tracks.items():
            if tid not in new_tracks:
                if age+1 < self.max_age:
                    new_tracks[tid] = (b, age+1)

        self.tracks = new_tracks
        return result_ids
