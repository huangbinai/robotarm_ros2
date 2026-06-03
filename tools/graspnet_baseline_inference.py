from __future__ import annotations

from pathlib import Path
import os
import sys
from typing import Any

import cv2
import numpy as np


DEFAULT_NUM_POINT = 20000


def add_windows_dll_directories() -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return
    candidates = [
        Path(sys.prefix) / "Lib" / "site-packages" / "torch" / "lib",
        Path(os.environ.get("CUDA_PATH", "")) / "bin",
        Path(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin"),
        Path(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.1\bin"),
    ]
    for path in candidates:
        if path.is_dir():
            os.add_dll_directory(str(path))


def build_masked_cloud(
    *,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    detection: dict[str, Any],
    camera_info: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    depth = np.asarray(depth_mm)
    color = np.asarray(color_bgr)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2D image")
    if color.shape[:2] != depth.shape:
        raise ValueError("color_bgr and depth_mm must have the same image size")

    height, width = depth.shape
    mask = _detection_mask(detection, width=width, height=height)
    z = depth.astype(np.float32) * float(camera_info.get("depth_scale_m", 0.001))
    valid = mask & np.isfinite(z) & (z > 0.0)
    valid = _reject_far_background(valid, z)
    v, u = np.nonzero(valid)
    if len(u) == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

    fx = float(camera_info["fx"])
    fy = float(camera_info["fy"])
    cx = float(camera_info["cx"])
    cy = float(camera_info["cy"])
    z_values = z[v, u]
    x = (u.astype(np.float32) - cx) * z_values / fx
    y = (v.astype(np.float32) - cy) * z_values / fy
    points = np.column_stack((x, y, z_values)).astype(np.float32)
    colors = color[v, u, :3].astype(np.float32)[:, ::-1] / 255.0
    return points, colors.astype(np.float32)


def _detection_mask(detection: dict[str, Any], *, width: int, height: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = _detection_polygon(detection)
    if polygon is not None:
        points = np.rint(polygon).astype(np.int32)
        points[:, 0] = np.clip(points[:, 0], 0, width - 1)
        points[:, 1] = np.clip(points[:, 1], 0, height - 1)
        cv2.fillPoly(mask, [points.reshape(-1, 1, 2)], 1)
        return mask.astype(bool)

    x_min, y_min, x_max, y_max = _detection_bbox(detection, width=width, height=height)
    mask[y_min:y_max, x_min:x_max] = 1
    return mask.astype(bool)


def _detection_polygon(detection: dict[str, Any]) -> np.ndarray | None:
    raw = None
    mask = detection.get("mask")
    if isinstance(mask, dict):
        raw = mask.get("polygon_xy")
    if raw is None:
        raw = detection.get("mask_polygon_xy")
    if raw is None:
        return None
    arr = np.asarray(raw, dtype=np.float32).reshape(-1, 2)
    if arr.shape[0] < 3:
        return None
    return arr


def _detection_bbox(detection: dict[str, Any], *, width: int, height: int) -> tuple[int, int, int, int]:
    if "bbox_xyxy" in detection:
        raw = np.asarray(detection.get("bbox_xyxy"), dtype=np.float32).reshape(-1)
        if raw.size >= 4:
            x_min, y_min, x_max, y_max = raw[:4]
        else:
            x_min, y_min, x_max, y_max = 0, 0, width, height
    else:
        x_min = detection.get("x_min", 0)
        y_min = detection.get("y_min", 0)
        x_max = detection.get("x_max", width)
        y_max = detection.get("y_max", height)
    ix_min = max(0, min(width - 1, int(round(float(x_min)))))
    iy_min = max(0, min(height - 1, int(round(float(y_min)))))
    ix_max = max(ix_min + 1, min(width, int(round(float(x_max)))))
    iy_max = max(iy_min + 1, min(height, int(round(float(y_max)))))
    return ix_min, iy_min, ix_max, iy_max


def _reject_far_background(valid: np.ndarray, z_m: np.ndarray) -> np.ndarray:
    values = z_m[valid]
    if values.size < 32:
        return valid
    near = float(np.percentile(values, 10.0))
    max_object_depth = near + 0.35
    return valid & (z_m <= max_object_depth)


def sample_cloud(points: np.ndarray, colors: np.ndarray, *, num_point: int) -> tuple[np.ndarray, np.ndarray]:
    if len(points) == 0:
        return points, colors
    count = int(num_point)
    if len(points) >= count:
        indices = np.random.choice(len(points), count, replace=False)
    else:
        extra = np.random.choice(len(points), count - len(points), replace=True)
        indices = np.concatenate([np.arange(len(points)), extra], axis=0)
    return points[indices].astype(np.float32), colors[indices].astype(np.float32)


def graspnet_array_to_candidates(grasp_array, *, class_name: str, max_grasps: int) -> list[dict[str, Any]]:
    array = np.asarray(grasp_array, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    candidates: list[dict[str, Any]] = []
    for row in array[: max(0, int(max_grasps))]:
        if row.size < 16:
            continue
        rotation = row[4:13].reshape(3, 3)
        translation = row[13:16]
        candidates.append(
            {
                "source": "windows_graspnet_baseline",
                "class_name": str(class_name),
                "score": float(row[0]),
                "width_m": float(row[1]),
                "height_m": float(row[2]),
                "depth_m": float(row[3]),
                "rotation_matrix": rotation.astype(float).tolist(),
                "translation_xyz": translation.astype(float).tolist(),
                "object_length_m": float(row[2]),
            }
        )
    return candidates


class GraspNetBaselineInference:
    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str,
        device: str = "cuda:0",
        num_point: int = DEFAULT_NUM_POINT,
        collision_thresh: float = 0.01,
        voxel_size: float = 0.01,
    ) -> None:
        self.model_root = str(model_root)
        self.checkpoint_path = str(checkpoint_path)
        self.device = str(device)
        self.num_point = int(num_point)
        self.collision_thresh = float(collision_thresh)
        self.voxel_size = float(voxel_size)
        self._torch = None
        self._GraspGroup = None
        self._ModelFreeCollisionDetector = None
        self._pred_decode = None
        self.net = self._load_network()

    def _load_network(self):
        add_windows_dll_directories()
        root = Path(self.model_root)
        for child in ("models", "dataset", "utils"):
            path = str(root / child)
            if path not in sys.path:
                sys.path.insert(0, path)
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

        import torch
        from graspnet import GraspNet, pred_decode
        from graspnetAPI import GraspGroup

        self._torch = torch
        self._pred_decode = pred_decode
        self._GraspGroup = GraspGroup
        try:
            from collision_detector import ModelFreeCollisionDetector

            self._ModelFreeCollisionDetector = ModelFreeCollisionDetector
        except Exception:
            self._ModelFreeCollisionDetector = None

        net = GraspNet(
            input_feature_dim=0,
            num_view=300,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        state_dict = checkpoint.get("model_state_dict", checkpoint)
        net.load_state_dict(state_dict)
        net.to(self.device)
        net.eval()
        return net

    def infer(
        self,
        *,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
        detections: list[dict[str, Any]],
        camera_info: dict[str, Any],
        max_grasps: int,
    ) -> list[dict[str, Any]]:
        if not detections:
            return []
        detection = max(detections, key=lambda item: float(item.get("confidence", 0.0)))
        points, colors = build_masked_cloud(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            detection=detection,
            camera_info=camera_info,
        )
        if len(points) == 0:
            return []
        points_sampled, colors_sampled = sample_cloud(points, colors, num_point=self.num_point)
        grasp_array = self._infer_grasp_array(points_sampled, colors_sampled, full_points=points)
        return graspnet_array_to_candidates(
            grasp_array,
            class_name=str(detection.get("class_name", "")),
            max_grasps=max_grasps,
        )

    def _infer_grasp_array(self, points: np.ndarray, colors: np.ndarray, *, full_points: np.ndarray):
        torch = self._torch
        if torch is None or self._pred_decode is None or self._GraspGroup is None:
            raise RuntimeError("GraspNet backend is not loaded")
        cloud = torch.from_numpy(points[np.newaxis].astype(np.float32)).to(self.device)
        color = torch.from_numpy(colors[np.newaxis].astype(np.float32)).to(self.device)
        end_points = {"point_clouds": cloud, "cloud_colors": color}
        with torch.no_grad():
            end_points = self.net(end_points)
            decoded = self._pred_decode(end_points)
        grasp_group = self._GraspGroup(decoded[0].detach().cpu().numpy())
        if self._ModelFreeCollisionDetector is not None and self.collision_thresh > 0:
            detector = self._ModelFreeCollisionDetector(full_points, voxel_size=self.voxel_size)
            collision_mask = detector.detect(grasp_group, approach_dist=0.05, collision_thresh=self.collision_thresh)
            grasp_group = grasp_group[~collision_mask]
        try:
            grasp_group = grasp_group.nms()
        except ModuleNotFoundError:
            pass
        grasp_group = grasp_group.sort_by_score()
        return getattr(grasp_group, "grasp_group_array", np.asarray(grasp_group))
