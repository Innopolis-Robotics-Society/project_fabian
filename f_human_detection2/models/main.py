#!/usr/bin/env python3
"""
Export a .pt model to ONNX via torch.onnx.export, with opset compatible
with your onnxruntime (1.20.0 supports up to opset 21).

Usage example:

  python3 export_pt_to_onnx.py \
      --pt yolo11n_handpose.pt \
      --onnx yolo11n_handpose_opset21.onnx \
      --imgsz 640 \
      --opset 21

By default this script FIRST tries to treat the .pt as a TorchScript model
(torch.jit.load). If that fails, it will stop and tell you to edit build_model()
to construct your model and load a state_dict manually.
"""

import argparse
import sys
from pathlib import Path

import torch
import onnxruntime as ort


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--pt", type=str, required=True,
                   help="Path to .pt model (TorchScript or checkpoint)")
    p.add_argument("--onnx", type=str, required=True,
                   help="Output .onnx path")
    p.add_argument("--imgsz", type=int, nargs="+", default=[640],
                   help="Input size. One value => square H=W; two values => H W")
    p.add_argument("--opset", type=int, default=21,
                   help="ONNX opset version (<= 21 for ORT 1.20.0)")
    p.add_argument("--dynamic", action="store_true",
                   help="Export with dynamic batch dimension")
    return p.parse_args()


def normalize_imgsz(imgsz_arg):
    if len(imgsz_arg) == 1:
        h = w = int(imgsz_arg[0])
    elif len(imgsz_arg) == 2:
        h, w = map(int, imgsz_arg)
    else:
        raise ValueError("imgsz must be 1 (square) or 2 (H W) integers")
    return h, w


def build_model_from_torchscript(pt_path: Path) -> torch.nn.Module:
    """
    Try to load the .pt as a TorchScript model.
    This requires that pt_path was saved with torch.jit.script/trace.
    """
    print(f"[INFO] Trying TorchScript load from: {pt_path}")
    model = torch.jit.load(str(pt_path), map_location="cpu")
    model.eval()
    print("[INFO] Loaded TorchScript model successfully.")
    return model


def build_model_from_state_dict(pt_path: Path) -> torch.nn.Module:
    """
    TEMPLATE for state_dict loading.
    You MUST edit this function to match your architecture if TorchScript load fails.
    """
    raise NotImplementedError(
        "TorchScript load failed. You must implement build_model_from_state_dict() "
        "to construct your model and load the state_dict from this .pt."
    )

    # Example skeleton (you must replace with your own model):
    #
    # from my_model_def import MyModel
    #
    # model = MyModel(...)            # construct your model
    # ckpt = torch.load(str(pt_path), map_location="cpu")
    #
    # # Depending on how you saved the checkpoint, you may need:
    # #   ckpt['model'] or ckpt['state_dict'] or ckpt directly.
    # state_dict = ckpt if isinstance(ckpt, dict) and 'state_dict' not in ckpt else ckpt['state_dict']
    #
    # model.load_state_dict(state_dict)
    # model.eval()
    # return model


def build_model(pt_path: Path) -> torch.nn.Module:
    """
    Try TorchScript first. If that fails, fall back to state_dict path
    (which you must implement).
    """
    try:
        return build_model_from_torchscript(pt_path)
    except Exception as e:
        print(f"[WARN] TorchScript load failed: {e}", file=sys.stderr)
        print("[WARN] Falling back to state_dict path (you must implement it).", file=sys.stderr)
        return build_model_from_state_dict(pt_path)


def export_to_onnx(model: torch.nn.Module,
                   onnx_path: Path,
                   img_h: int,
                   img_w: int,
                   opset: int,
                   dynamic: bool):
    """
    Use torch.onnx.export to save ONNX.
    """
    model.eval()
    dummy = torch.randn(1, 3, img_h, img_w, device="cpu")

    dynamic_axes = None
    if dynamic:
        dynamic_axes = {
            "images": {0: "batch"},
            "output": {0: "batch"},
        }

    print(f"[INFO] Exporting to ONNX: {onnx_path}")
    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        input_names=["images"],
        output_names=["output"],
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
    )
    print("[INFO] ONNX export finished.")


def verify_with_ort(onnx_path: Path):
    """
    Verify that the exported model loads with the current onnxruntime.
    """
    print(f"[INFO] Verifying ONNX model with onnxruntime {ort.__version__}")
    providers = ort.get_available_providers()
    print(f"[INFO] Available providers: {providers}")

    sess = ort.InferenceSession(
        str(onnx_path),
        providers=providers
    )

    inputs = sess.get_inputs()
    outputs = sess.get_outputs()

    print("[INFO] Model inputs:")
    for i in inputs:
        print(f"  name={i.name}, shape={i.shape}, type={i.type}")

    print("[INFO] Model outputs:")
    for o in outputs:
        print(f"  name={o.name}, shape={o.shape}, type={o.type}")

    print("[INFO] ONNX model successfully loaded by onnxruntime.")


def main():
    args = parse_args()

    pt_path = Path(args.pt)
    if not pt_path.exists():
        print(f"[ERROR] .pt file not found: {pt_path}", file=sys.stderr)
        sys.exit(1)

    onnx_path = Path(args.onnx)
    onnx_path.parent.mkdir(parents=True, exist_ok=True)

    img_h, img_w = normalize_imgsz(args.imgsz)
    print(f"[INFO] Using input size: {img_h}x{img_w}")
    print(f"[INFO] Target opset: {args.opset}")

    # Build model
    model = build_model(pt_path)

    # Export
    export_to_onnx(model, onnx_path, img_h, img_w, args.opset, args.dynamic)

    # Verify with your current ORT (1.20.0 with CUDA/TensorRT)
    verify_with_ort(onnx_path)


if __name__ == "__main__":
    main()
                    