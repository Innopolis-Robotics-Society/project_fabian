import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def resolve_model_path(param_path: str, package_name: str = "f_human_detection2") -> str:
    """
    Resolve model file path.
    1) If absolute path exists -> use it.
    2) If relative -> resolve against package share 'models' directory.
    """
    if not param_path:
        return ""
    p = Path(param_path)
    if p.is_file():
        return str(p.resolve())
    share = Path(get_package_share_directory(package_name))
    candidate = share / "models" / param_path
    if candidate.is_file():
        return str(candidate.resolve())
    # also try direct file name in share/models
    candidate = share / "models" / Path(param_path).name
    return str(candidate.resolve()) if candidate.is_file() else ""

def letterbox(img, new_shape=(640, 384), color=(114, 114, 114)):
    """
    Resize with unchanged aspect ratio using padding.
    Returns: image, ratio, (dw, dh)
    """
    shape = img.shape[:2]  # (h, w)
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[1] / shape[0], new_shape[0] / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw, dh = new_shape[0] - new_unpad[0], new_shape[1] - new_unpad[1]
    dw /= 2; dh /= 2
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh-0.1)), int(round(dh+0.1))
    left, right = int(round(dw-0.1)), int(round(dw+0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, r, (dw, dh)

def scale_coords_kpts(kpts, ratio, dwdh, orig_w, orig_h):
    """
    Map keypoints back from letterboxed space to original image.
    kpts: (N,17,3)
    """
    if kpts.size == 0:
        return kpts
    k = kpts.copy()
    k[..., 0] = (k[..., 0] - dwdh[0]) / ratio
    k[..., 1] = (k[..., 1] - dwdh[1]) / ratio
    k[..., 0] = np.clip(k[..., 0], 0, orig_w - 1)
    k[..., 1] = np.clip(k[..., 1], 0, orig_h - 1)
    return k

def nms(boxes, scores, iou_th=0.45, max_dets=300):
    """
    Simple NMS in numpy. boxes: (N,4) xywh
    """
    if len(boxes) == 0:
        return []
    x1 = boxes[:,0]; y1 = boxes[:,1]; w = boxes[:,2]; h = boxes[:,3]
    x2 = x1 + w; y2 = y1 + h
    order = scores.argsort()[::-1]
    keep = []
    areas = w * h
    while order.size > 0 and len(keep) < max_dets:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        inds = np.where(iou <= iou_th)[0]
        order = order[inds + 1]
    return keep
