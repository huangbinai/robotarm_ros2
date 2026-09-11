#!/usr/bin/env python3
"""Run real YOLO + GraspNet weights on a checked-in upstream RGB-D sample.

No camera or robot connection. Reports model execution, not grasp task success.
"""
import argparse
import json
from pathlib import Path
import time


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-root", type=Path, default=root / "third_party/graspnet-baseline")
    parser.add_argument("--checkpoint", type=Path, default=root / "models/graspnet/checkpoint-rs.tar")
    parser.add_argument("--yolo-model", type=Path, default=root / "tools/yolo26s-seg.pt")
    parser.add_argument("--iterations", type=int, default=3)
    args = parser.parse_args()
    import cv2
    import numpy as np
    import scipy.io
    import torch
    from builtin_interfaces.msg import Time
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg
    from rebotarm_vision.detector.yolo_detector import YoloDetector
    from rebotarm_vision.graspnet_baseline_adapter import GraspNetBaselineBackend, predictions_to_candidate_array
    from rebotarm_vision.perception_frames import detection_payload

    torch.set_num_threads(2)
    np.random.seed(0)
    torch.manual_seed(0)
    sample = args.model_root / "doc/example_data"
    color = cv2.imread(str(sample / "color.png"))
    depth = cv2.imread(str(sample / "depth.png"), cv2.IMREAD_UNCHANGED)
    meta = scipy.io.loadmat(str(sample / "meta.mat"))
    intrinsic = meta["intrinsic_matrix"]
    factor = float(np.asarray(meta["factor_depth"]).reshape(-1)[0])
    if color is None or depth is None or factor <= 0:
        raise RuntimeError("upstream RGB-D sample missing or invalid")
    depth_mm = (depth.astype(np.float64) * (1000.0 / factor)).astype(np.uint16)
    camera_info = dict(fx=float(intrinsic[0, 0]), fy=float(intrinsic[1, 1]),
                       cx=float(intrinsic[0, 2]), cy=float(intrinsic[1, 2]), depth_scale_m=0.001,
                       workspace_min_depth_m=0.15, workspace_max_depth_m=1.2)
    detector = YoloDetector(str(args.yolo_model), "cuda:0", 0.25, 0.45, False, [])
    backend = GraspNetBaselineBackend(model_root=str(args.model_root), checkpoint_path=str(args.checkpoint))
    results = []
    for iteration in range(max(1, args.iterations)):
        torch.cuda.synchronize()
        start = time.perf_counter()
        detections = result_to_detection_array_msg(detector.infer(color), Time(sec=1), "camera_depth_frame")
        torch.cuda.synchronize()
        yolo_seconds = time.perf_counter() - start
        payloads = [detection_payload(detection) for detection in detections.detections]
        if not payloads:
            raise RuntimeError("YOLO produced no detections on the upstream sample; GraspNet was not exercised")
        start = time.perf_counter()
        predictions = backend.infer(color_bgr=color, depth_mm=depth_mm, detections=payloads,
                                    camera_info=camera_info, max_grasps=5)
        torch.cuda.synchronize()
        graspnet_seconds = time.perf_counter() - start
        target = max(payloads, key=lambda item: item["confidence"])["class_name"]
        message = predictions_to_candidate_array(predictions, frame_id="camera_depth_frame", class_name=target, max_candidates=5)
        finite = all(np.isfinite([p.score, p.width_m, *p.translation_xyz, *np.asarray(p.rotation_matrix).reshape(-1)]).all() for p in predictions)
        results.append(dict(iteration=iteration, detections=len(payloads), target=target,
                            candidates=len(message.candidates), finite=finite,
                            yolo_seconds=yolo_seconds, graspnet_seconds=graspnet_seconds))
    report = dict(ok=all(row["finite"] for row in results), gpu=torch.cuda.get_device_name(),
                  torch=torch.__version__, cuda=torch.version.cuda, source="upstream recorded RGB-D sample",
                  hardware_connected=False, results=results)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
