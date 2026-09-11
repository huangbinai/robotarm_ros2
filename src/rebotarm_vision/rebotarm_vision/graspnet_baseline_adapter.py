from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
import sys
from pathlib import Path
from typing import Iterable

import numpy as np

from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray


@dataclass(frozen=True)
class GraspNetPrediction:
    score: float
    translation_xyz: tuple[float, float, float]
    rotation_matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    width_m: float
    object_length_m: float = 0.0


def predictions_to_candidate_array(
    predictions: Iterable[GraspNetPrediction],
    *,
    frame_id: str,
    class_name: str,
    max_candidates: int,
    source: str = "graspnet_baseline",
) -> GraspCandidateArray:
    array = GraspCandidateArray()
    array.header.frame_id = frame_id
    array.best_index = -1
    for prediction in list(predictions)[: max(0, int(max_candidates))]:
        candidate = GraspCandidate()
        candidate.header.frame_id = frame_id
        candidate.class_name = class_name
        candidate.confidence = float(prediction.score)
        candidate.pose.position.x = float(prediction.translation_xyz[0])
        candidate.pose.position.y = float(prediction.translation_xyz[1])
        candidate.pose.position.z = float(prediction.translation_xyz[2])
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(prediction.rotation_matrix)
        candidate.pose.orientation.x = qx
        candidate.pose.orientation.y = qy
        candidate.pose.orientation.z = qz
        candidate.pose.orientation.w = qw
        candidate.jaw_width = float(prediction.width_m)
        candidate.object_length = float(prediction.object_length_m)
        candidate.valid = True
        candidate.source = source
        array.candidates.append(candidate)
    if array.candidates:
        array.best_index = 0
    return array


class GraspNetBaselineBackend:
    """Thin optional wrapper around an installed GraspNet baseline inference module.

    The upstream GraspNet baseline repository is a research codebase, so this
    adapter uses the packaged full-scene inference wrapper and its RGB-D API.
    """

    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str = "",
        device: str = "cuda:0",
        module_name: str = "rebotarm_vision.graspnet_inference",
        num_point: int = 20000,
    ) -> None:
        self.model_root = str(model_root).strip()
        self.checkpoint_path = str(checkpoint_path).strip()
        self.device = str(device).strip()
        self.module_name = str(module_name).strip()
        self._runner = None
        if not self.model_root:
            return
        root = Path(self.model_root)
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        module = importlib.import_module(self.module_name)
        runner_cls = getattr(module, "GraspNetBaselineInference")
        self._runner = runner_cls(
            model_root=self.model_root,
            checkpoint_path=self.checkpoint_path,
            device=self.device,
            num_point=int(num_point),
        )

    @property
    def available(self) -> bool:
        return self._runner is not None

    def infer(self, *, color_bgr, depth_mm, detections, camera_info, max_grasps: int) -> list[GraspNetPrediction]:
        if self._runner is None:
            raise RuntimeError("GraspNet baseline backend is not configured")
        raw_predictions = self._runner.infer(
            color_bgr=color_bgr, depth_mm=depth_mm, detections=detections,
            camera_info=camera_info, max_grasps=max_grasps,
        )
        return [_prediction_from_raw(item) for item in raw_predictions]


def _prediction_from_raw(item) -> GraspNetPrediction:
    if isinstance(item, GraspNetPrediction):
        return item
    if isinstance(item, dict):
        rotation = item.get("rotation_matrix", item.get("rotation"))
        translation = item.get("translation_xyz", item.get("translation"))
        return GraspNetPrediction(
            score=float(item.get("score", item.get("confidence", 0.0))),
            translation_xyz=_tuple3(translation),
            rotation_matrix=_matrix3(rotation),
            width_m=float(item.get("width_m", item.get("width", 0.0))),
            object_length_m=float(item.get("object_length_m", item.get("object_length", 0.0))),
        )
    return GraspNetPrediction(
        score=float(getattr(item, "score", getattr(item, "confidence", 0.0))),
        translation_xyz=_tuple3(getattr(item, "translation_xyz", getattr(item, "translation", None))),
        rotation_matrix=_matrix3(getattr(item, "rotation_matrix", getattr(item, "rotation", None))),
        width_m=float(getattr(item, "width_m", getattr(item, "width", 0.0))),
        object_length_m=float(getattr(item, "object_length_m", getattr(item, "object_length", 0.0))),
    )


def _tuple3(values) -> tuple[float, float, float]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.size < 3:
        raise ValueError("expected at least 3 values")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _matrix3(values) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    arr = np.asarray(values, dtype=np.float64).reshape(3, 3)
    return tuple(tuple(float(v) for v in row) for row in arr)  # type: ignore[return-value]


def _rotation_matrix_to_quaternion(matrix) -> tuple[float, float, float, float]:
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (float(qx / norm), float(qy / norm), float(qz / norm), float(qw / norm))
