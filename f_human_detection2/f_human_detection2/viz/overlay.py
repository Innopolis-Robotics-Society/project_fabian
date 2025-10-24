import cv2
import numpy as np

# Same edges as in postprocess
EDGES = [
    (5,7),(7,9),(6,8),(8,10),(5,6),
    (11,13),(13,15),(12,14),(14,16),
    (5,11),(6,12),(11,12)
]

def draw_overlay(img, boxes, kpts, ids=None):
    """
    Draw skeleton lines and ids on BGR image.
    """
    out = img.copy()
    for i in range(len(boxes)):
        if ids is not None:
            cv2.putText(out, f"{ids[i]}", (int(boxes[i,0]), int(boxes[i,1])-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(out, f"{ids[i]}", (int(boxes[i,0]), int(boxes[i,1])-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        # bbox
        x,y,w,h = boxes[i].astype(int)
        cv2.rectangle(out, (x,y), (x+w,y+h), (0,255,0), 2)
        # keypoints
        for a,b in EDGES:
            xa,ya,sa = kpts[i,a]
            xb,yb,sb = kpts[i,b]
            if sa>0 and sb>0:
                cv2.line(out, (int(xa),int(ya)), (int(xb),int(yb)), (255,0,0), 2)
        for j in range(kpts.shape[1]):
            xj,yj,sj = kpts[i,j]
            if sj>0:
                cv2.circle(out, (int(xj),int(yj)), 2, (0,0,255), -1)
    return out
