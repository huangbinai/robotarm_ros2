#!/usr/bin/env python3
"""Read-only readiness check; optionally enumerate cameras, never start streams."""
import argparse
import hashlib
import importlib.util
import importlib
import json
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-root", type=Path, default=root / "third_party/graspnet-baseline")
    parser.add_argument("--checkpoint", type=Path, default=root / "models/graspnet/checkpoint-rs.tar")
    parser.add_argument("--yolo-model", type=Path, default=root / "tools/yolo26s-seg.pt")
    parser.add_argument("--probe-camera", action="store_true", help="enumerate Orbbec devices; never starts streaming or moves the robot")
    args = parser.parse_args()
    errors = []
    modules = {}
    for name in ("rclpy", "message_filters", "pyorbbecsdk", "ultralytics", "torch", "open3d", "graspnetAPI"):
        spec = importlib.util.find_spec(name)
        modules[name] = spec.origin if spec else None
        if spec is None:
            errors.append(f"missing Python module: {name}")
    paths = {"model_root": args.model_root, "checkpoint": args.checkpoint, "yolo_model": args.yolo_model}
    for name, path in paths.items():
        if not (path.is_dir() if name == "model_root" else path.is_file()):
            errors.append(f"missing {name}: {path}")
    if args.model_root.is_dir():
        for relative in ("models/graspnet.py", "pointnet2", "knn"):
            if not (args.model_root / relative).exists():
                errors.append(f"incomplete GraspNet repository: {relative}")
    cuda = None
    if modules["torch"]:
        try:
            import torch
            cuda = bool(torch.cuda.is_available())
            if not cuda:
                errors.append("CUDA unavailable in active PyTorch; GraspNet CUDA extensions require a compatible build")
        except Exception as exc:
            errors.append(f"PyTorch failed to load: {exc}")
    extensions = {}
    for name in ("pointnet2._ext", "knn_pytorch.knn_pytorch", "grasp_nms"):
        try:
            module = importlib.import_module(name)
            extensions[name] = module.__file__
        except Exception as exc:
            extensions[name] = None
            errors.append(f"native extension unavailable: {name}: {exc}")
    camera_count = None
    if args.probe_camera:
        try:
            sdk = importlib.import_module("pyorbbecsdk")
            context = sdk.Context()
            camera_count = context.query_devices().get_count()
            if camera_count == 0:
                errors.append("No Orbbec camera detected; connect Gemini 2 to Ubuntu for capture acceptance")
        except Exception as exc:
            errors.append(f"camera enumeration failed: {exc}")
    hashes = {}
    manifest = root / "models/MANIFEST.sha256"
    expected = {line.split()[1]: line.split()[0] for line in manifest.read_text().splitlines() if line.strip() and not line.startswith("#")}
    for name, path in (("checkpoint", args.checkpoint), ("yolo_model", args.yolo_model)):
        if not path.is_file():
            continue
        with path.open("rb") as stream:
            hashes[name] = hashlib.file_digest(stream, "sha256").hexdigest()
        try:
            relative = path.resolve().relative_to(root).as_posix()
        except ValueError:
            relative = ""
        if relative in expected and hashes[name] != expected[relative]:
            errors.append(f"checksum mismatch: {path}")
    report = dict(
        ok=not errors, python=sys.executable, modules=modules, cuda_available=cuda,
        paths={name: str(path) for name, path in paths.items()}, sha256=hashes, errors=errors,
        extensions=extensions, camera_count=camera_count,
        scope="imports/native-extension loading/paths/CUDA/known hashes; optional device enumeration, no capture or robot commands",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
