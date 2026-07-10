#!/usr/bin/env python3
"""Export an Ultralytics detection model and compile it for RK3588."""

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--imgsz", type=int, default=640)
    args = parser.parse_args()

    from ultralytics import YOLO
    from rknn.api import RKNN

    model = YOLO(str(args.model))
    exported = Path(
        model.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=12,
            simplify=False,
            dynamic=False,
            batch=1,
            device="cpu",
        )
    )

    compiler = RKNN(verbose=False)
    try:
        if compiler.config(
            target_platform="rk3588",
            mean_values=[[0, 0, 0]],
            std_values=[[255, 255, 255]],
            optimization_level=3,
        ) != 0:
            raise RuntimeError("RKNN config failed")
        if compiler.load_onnx(model=str(exported)) != 0:
            raise RuntimeError("RKNN ONNX import failed")
        if compiler.build(do_quantization=False) != 0:
            raise RuntimeError("RKNN build failed")
        if compiler.export_rknn(str(args.output)) != 0:
            raise RuntimeError("RKNN export failed")
    finally:
        compiler.release()

    metadata = {
        "source": args.model.name,
        "source_sha256": sha256(args.model),
        "onnx": exported.name,
        "onnx_sha256": sha256(exported),
        "artifact": args.output.name,
        "artifact_sha256": sha256(args.output),
        "target_platform": "rk3588",
        "precision": "fp16",
        "input": {"shape": [1, args.imgsz, args.imgsz, 3], "layout": "NHWC", "dtype": "uint8", "color": "RGB"},
        "classes": model.names,
        "toolchain": {
            "ultralytics": "8.3.40",
            "torch": "2.4.1+cpu",
            "onnx": "1.16.1",
            "rknn_toolkit2": "2.3.2",
        },
    }
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
