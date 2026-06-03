from __future__ import annotations

import argparse
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any
from urllib.request import Request, urlopen

import cv2
import numpy as np


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run Windows-side GraspNet baseline inference and write candidates JSON for Ubuntu ROS2."
    )
    parser.add_argument("--server-url", default="http://127.0.0.1:8081")
    parser.add_argument("--output-path", default=r"D:\tmp\graspnet_candidates.json")
    parser.add_argument("--model-root", default="")
    parser.add_argument("--checkpoint-path", default="")
    parser.add_argument("--backend-module", default="graspnet_baseline_inference")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--max-grasps", type=int, default=20)
    parser.add_argument("--poll-hz", type=float, default=2.0)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--once", action="store_true")
    return parser


def atomic_write_json(path: str | Path, payload: dict[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=str(target.parent), delete=False) as handle:
        json.dump(payload, handle, ensure_ascii=False)
        tmp_name = handle.name
    os.replace(tmp_name, target)


def write_backend_missing_payload(path: str | Path, *, reason: str) -> None:
    atomic_write_json(
        path,
        {
            "frame_id": "camera_depth_frame",
            "source": "windows_graspnet_baseline",
            "backend_configured": False,
            "candidates": [],
            "error": str(reason),
            "timestamp": time.time(),
        },
    )


def load_backend(*, model_root: str, checkpoint_path: str, backend_module: str, device: str):
    if model_root:
        root = str(Path(model_root))
        if root not in sys.path:
            sys.path.insert(0, root)
    module = importlib.import_module(backend_module)
    runner_cls = getattr(module, "GraspNetBaselineInference")
    return runner_cls(model_root=model_root, checkpoint_path=checkpoint_path, device=device)


def fetch_json(server_url: str, endpoint: str, timeout_ms: int) -> dict[str, Any]:
    url = f"{server_url.rstrip('/')}/{endpoint.lstrip('/')}"
    request = Request(url, headers={"User-Agent": "rebotarm_graspnet_bridge/0.1"})
    with urlopen(request, timeout=max(int(timeout_ms), 1) / 1000.0) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"{endpoint} returned non-object JSON")
    return payload


def fetch_image(server_url: str, endpoint: str, timeout_ms: int, flags: int) -> np.ndarray:
    url = f"{server_url.rstrip('/')}/{endpoint.lstrip('/')}"
    request = Request(url, headers={"User-Agent": "rebotarm_graspnet_bridge/0.1"})
    with urlopen(request, timeout=max(int(timeout_ms), 1) / 1000.0) as response:
        data = response.read()
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), flags)
    if image is None:
        raise RuntimeError(f"{endpoint} decode failed")
    return image


def build_graspnet_payload(
    *,
    backend,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    detections_payload: dict[str, Any],
    camera_info: dict[str, Any],
    max_grasps: int,
) -> dict[str, Any]:
    detections = list(detections_payload.get("detections", []))
    if not detections:
        return {
            "frame_id": str(camera_info.get("depth_frame_id", "camera_depth_frame")),
            "source": "windows_graspnet_baseline",
            "backend_configured": True,
            "candidates": [],
            "reason": "no detections",
            "timestamp": time.time(),
        }
    raw = backend.infer(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detections=detections,
        camera_info=camera_info,
        max_grasps=max_grasps,
    )
    candidates = []
    for item in list(raw)[: max(0, int(max_grasps))]:
        if not isinstance(item, dict):
            continue
        candidate = dict(item)
        candidate.setdefault("source", "windows_graspnet_baseline")
        candidate.setdefault("class_name", str(detections[0].get("class_name", "")))
        candidates.append(candidate)
    return {
        "frame_id": str(camera_info.get("depth_frame_id", "camera_depth_frame")),
        "source": "windows_graspnet_baseline",
        "backend_configured": True,
        "candidates": candidates,
        "timestamp": time.time(),
    }


def run_once(args, backend) -> dict[str, Any]:
    color_bgr = fetch_image(args.server_url, "/snapshot.jpg", args.timeout_ms, cv2.IMREAD_COLOR)
    depth_mm = fetch_image(args.server_url, "/depth.png", args.timeout_ms, cv2.IMREAD_UNCHANGED)
    if depth_mm.ndim == 3:
        depth_mm = depth_mm[:, :, 0]
    detections = fetch_json(args.server_url, "/detections.json", args.timeout_ms)
    camera_info = fetch_json(args.server_url, "/camera_info.json", args.timeout_ms)
    payload = build_graspnet_payload(
        backend=backend,
        color_bgr=color_bgr,
        depth_mm=depth_mm.astype(np.uint16, copy=False),
        detections_payload=detections,
        camera_info=camera_info,
        max_grasps=int(args.max_grasps),
    )
    atomic_write_json(args.output_path, payload)
    return payload


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        backend = load_backend(
            model_root=args.model_root,
            checkpoint_path=args.checkpoint_path,
            backend_module=args.backend_module,
            device=args.device,
        )
    except Exception as exc:
        write_backend_missing_payload(args.output_path, reason=f"{type(exc).__name__}: {exc}")
        raise SystemExit(
            "GraspNet backend unavailable. Create a module exposing "
            "GraspNetBaselineInference(...).infer(...) and pass --backend-module/--model-root."
        ) from exc

    period = 1.0 / max(float(args.poll_hz), 0.1)
    while True:
        try:
            payload = run_once(args, backend)
            print(f"wrote {len(payload.get('candidates', []))} grasp candidates to {args.output_path}")
        except Exception as exc:
            write_backend_missing_payload(args.output_path, reason=f"{type(exc).__name__}: {exc}")
            print(f"graspnet bridge failed: {type(exc).__name__}: {exc}")
        if args.once:
            return
        time.sleep(period)


if __name__ == "__main__":
    main()
