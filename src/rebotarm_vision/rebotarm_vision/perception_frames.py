"""Frame contracts for the local RGB-D -> YOLO -> GraspNet ROS pipeline."""
from __future__ import annotations

import math
import numpy as np


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def require_fresh(header, now_ns: int, max_age_sec: float) -> None:
    age = (int(now_ns) - stamp_ns(header.stamp)) / 1e9
    if not header.frame_id or not math.isfinite(max_age_sec) or max_age_sec <= 0:
        raise ValueError("frame and positive max_age_sec are required")
    if stamp_ns(header.stamp) <= 0 or not 0.0 <= age <= max_age_sec:
        raise ValueError(f"source frame is stale or in the future: age={age:.3f}s")


def color_image_to_array(msg) -> np.ndarray:
    if msg.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported color encoding: {msg.encoding}")
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0 or step < width * 3 or len(msg.data) != step * height:
        raise ValueError("invalid color image dimensions/stride/buffer")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
    image = rows[:, :width * 3].reshape(height, width, 3)
    return np.ascontiguousarray(image[:, :, ::-1] if msg.encoding == "rgb8" else image)


def camera_info_payload(info) -> dict:
    width, height = int(info.width), int(info.height)
    fx, fy, cx, cy = (float(info.k[i]) for i in (0, 4, 2, 5))
    if width <= 0 or height <= 0 or not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
        raise ValueError("invalid CameraInfo dimensions or intrinsics")
    if fx <= 0 or fy <= 0:
        raise ValueError("CameraInfo must contain calibrated positive focal lengths")
    if any(not math.isfinite(float(v)) or abs(float(v)) > 1e-9 for v in info.d):
        raise ValueError("RGB-D must be rectified before GraspNet point-cloud projection")
    return dict(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy, depth_scale_m=0.001)


def validate_frame_bundle(color, depth, info, detections, *, now_ns, max_age_sec) -> dict:
    messages = (color, depth, info, detections)
    stamps = {stamp_ns(message.header.stamp) for message in messages}
    frames = {message.header.frame_id for message in messages}
    if len(stamps) != 1 or len(frames) != 1:
        raise ValueError("color, aligned depth, CameraInfo and detections must share stamp and optical frame")
    require_fresh(color.header, now_ns, max_age_sec)
    if (color.width, color.height) != (depth.width, depth.height) or (
        color.width, color.height
    ) != (info.width, info.height):
        raise ValueError("aligned RGB-D and CameraInfo dimensions must match")
    return camera_info_payload(info)


def detection_payload(detection) -> dict:
    bbox = [int(getattr(detection, field)) for field in ("x_min", "y_min", "x_max", "y_max")]
    if bbox[2] <= bbox[0] or bbox[3] <= bbox[1] or not math.isfinite(float(detection.confidence)):
        raise ValueError("invalid detection bounding box/confidence")
    result = dict(class_name=str(detection.class_name), confidence=float(detection.confidence), bbox_xyxy=bbox)
    if detection.has_mask:
        polygon = list(detection.mask_polygon_xy)
        if len(polygon) < 6 or len(polygon) % 2 or not all(math.isfinite(float(v)) for v in polygon):
            raise ValueError("invalid segmentation polygon")
        result["mask_polygon_xy"] = np.asarray(polygon).reshape(-1, 2).tolist()
    return result
