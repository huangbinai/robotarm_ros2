from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load_wrapper():
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "tools" / "graspnet_baseline_inference.py"
    spec = importlib.util.spec_from_file_location("graspnet_baseline_inference", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_graspnet_wrapper_converts_graspnet_array_rows_to_json_candidates():
    module = _load_wrapper()
    row = np.array(
        [
            0.91,
            0.042,
            0.02,
            0.03,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.10,
            -0.02,
            0.35,
            3.0,
        ],
        dtype=np.float32,
    )

    candidates = module.graspnet_array_to_candidates(
        np.asarray([row]),
        class_name="bottle",
        max_grasps=5,
    )

    assert candidates[0]["class_name"] == "bottle"
    assert candidates[0]["score"] == pytest.approx(0.91)
    assert candidates[0]["width_m"] == pytest.approx(0.042)
    assert candidates[0]["translation_xyz"] == pytest.approx([0.10, -0.02, 0.35])
    assert np.asarray(candidates[0]["rotation_matrix"]) == pytest.approx(np.eye(3))


def test_graspnet_wrapper_builds_masked_cloud_from_detection_bbox():
    module = _load_wrapper()
    depth_mm = np.array(
        [
            [0, 0, 0, 0],
            [0, 500, 500, 0],
            [0, 500, 500, 0],
            [0, 0, 0, 0],
        ],
        dtype=np.uint16,
    )
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    detection = {"x_min": 1, "y_min": 1, "x_max": 3, "y_max": 3, "class_name": "bottle"}
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 2.0, "cy": 2.0, "depth_scale_m": 0.001}

    points, colors = module.build_masked_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detection=detection,
        camera_info=camera_info,
    )

    assert points.shape == (4, 3)
    assert colors.shape == (4, 3)
    assert points[:, 2].tolist() == pytest.approx([0.5, 0.5, 0.5, 0.5])


def test_graspnet_wrapper_accepts_windows_yolo_bbox_xyxy():
    module = _load_wrapper()
    depth_mm = np.ones((6, 6), dtype=np.uint16) * 2000
    depth_mm[2:4, 1:3] = 500
    color_bgr = np.zeros((6, 6, 3), dtype=np.uint8)
    detection = {"bbox_xyxy": [1.0, 2.0, 3.0, 4.0], "class_name": "bottle"}
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 3.0, "cy": 3.0, "depth_scale_m": 0.001}

    points, _colors = module.build_masked_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detection=detection,
        camera_info=camera_info,
    )

    assert points.shape[0] == 4
    assert points[:, 2].tolist() == pytest.approx([0.5, 0.5, 0.5, 0.5])


def test_graspnet_wrapper_uses_segmentation_mask_before_bbox_background():
    module = _load_wrapper()
    depth_mm = np.ones((8, 8), dtype=np.uint16) * 2500
    depth_mm[2:5, 2:5] = 450
    color_bgr = np.zeros((8, 8, 3), dtype=np.uint8)
    detection = {
        "bbox_xyxy": [0.0, 0.0, 8.0, 8.0],
        "mask": {
            "polygon_xy": [
                2.0,
                2.0,
                4.0,
                2.0,
                4.0,
                4.0,
                2.0,
                4.0,
            ]
        },
        "class_name": "bottle",
    }
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 4.0, "cy": 4.0, "depth_scale_m": 0.001}

    points, _colors = module.build_masked_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detection=detection,
        camera_info=camera_info,
    )

    assert points.shape[0] > 0
    assert float(points[:, 2].max()) == pytest.approx(0.45)
