from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path

import numpy as np


def _load_bridge():
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "tools" / "windows_graspnet_baseline_bridge.py"
    spec = importlib.util.spec_from_file_location("windows_graspnet_baseline_bridge", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_windows_graspnet_bridge_parser_defaults_to_local_http_service():
    module = _load_bridge()

    args = module.build_arg_parser().parse_args([])

    assert args.server_url == "http://127.0.0.1:8081"
    assert args.output_path.endswith("graspnet_candidates.json")
    assert args.max_grasps == 20
    assert args.device == "cuda:0"
    assert args.open3d_visualize is False
    assert args.visualize_top_n == 5
    assert args.visualize_max_points == 8000
    assert args.visualize_every_n == 10


def test_windows_graspnet_bridge_parser_accepts_open3d_visualization_options():
    module = _load_bridge()

    args = module.build_arg_parser().parse_args(
        [
            "--open3d-visualize",
            "--visualize-top-n",
            "4",
            "--visualize-max-points",
            "1234",
            "--visualize-every-n",
            "6",
        ]
    )

    assert args.open3d_visualize is True
    assert args.visualize_top_n == 4
    assert args.visualize_max_points == 1234
    assert args.visualize_every_n == 6


def test_windows_graspnet_visualizer_downsamples_points_and_limits_candidates(monkeypatch):
    module = _load_bridge()
    created = {}

    class FakeUtility:
        @staticmethod
        def Vector3dVector(values):
            return list(values)

        @staticmethod
        def Vector2iVector(values):
            return list(values)

    class FakePointCloud:
        def __init__(self):
            self.points = None
            self.colors = None

    class FakeLineSet:
        def __init__(self):
            self.points = None
            self.lines = None
            self.colors = None

    class FakeTriangleMesh:
        @staticmethod
        def create_coordinate_frame(size=1.0):
            return {"kind": "frame", "size": size}

    class FakeVisualizer:
        def create_window(self, *args, **kwargs):
            created["window"] = (args, kwargs)

        def clear_geometries(self):
            created["cleared"] = True

        def add_geometry(self, geometry):
            created.setdefault("geometries", []).append(geometry)

        def poll_events(self):
            created["polled"] = True

        def update_renderer(self):
            created["updated"] = True

    class FakeOpen3D:
        class geometry:
            PointCloud = FakePointCloud
            LineSet = FakeLineSet
            TriangleMesh = FakeTriangleMesh

        class utility(FakeUtility):
            pass

        class visualization:
            Visualizer = FakeVisualizer

    monkeypatch.setattr(module.importlib, "import_module", lambda name: FakeOpen3D)

    visualizer = module.Open3DGraspVisualizer(
        top_n=1,
        max_points=2,
        window_name="test",
    )
    visualizer.update(
        color_bgr=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_mm=np.ones((2, 2), dtype=np.uint16) * 500,
        camera_info={"fx": 500.0, "fy": 500.0, "cx": 0.0, "cy": 0.0},
        candidates=[
            {
                "score": 0.9,
                "translation_xyz": [0.0, 0.0, 0.5],
                "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "width_m": 0.04,
            },
            {
                "score": 0.5,
                "translation_xyz": [0.1, 0.0, 0.5],
                "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "width_m": 0.03,
            },
        ],
    )

    point_cloud = created["geometries"][0]
    assert len(point_cloud.points) == 2
    assert len(created["geometries"]) == 3


def test_windows_graspnet_bridge_writes_backend_missing_payload(tmp_path):
    module = _load_bridge()
    output = tmp_path / "graspnet_candidates.json"

    module.write_backend_missing_payload(output, reason="module not found")

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["source"] == "windows_graspnet_baseline"
    assert payload["backend_configured"] is False
    assert payload["candidates"] == []
    assert "module not found" in payload["error"]


def test_windows_graspnet_bridge_retries_replace_when_target_is_temporarily_locked(tmp_path, monkeypatch):
    module = _load_bridge()
    output = tmp_path / "graspnet_candidates.json"
    real_replace = os.replace
    calls = {"count": 0}

    def flaky_replace(src, dst):
        calls["count"] += 1
        if calls["count"] < 3:
            raise PermissionError("target temporarily locked")
        return real_replace(src, dst)

    monkeypatch.setattr(module.os, "replace", flaky_replace)
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)

    module.atomic_write_json(output, {"candidates": [1]})

    assert calls["count"] == 3
    assert json.loads(output.read_text(encoding="utf-8")) == {"candidates": [1]}


def test_windows_graspnet_bridge_builds_json_payload_from_fake_backend():
    module = _load_bridge()

    class FakeBackend:
        def infer(self, *, color_bgr, depth_mm, detections, camera_info, max_grasps):
            assert color_bgr.shape == (4, 4, 3)
            assert depth_mm.shape == (4, 4)
            assert detections[0]["class_name"] == "bottle"
            assert camera_info["depth_frame_id"] == "camera_depth_frame"
            assert max_grasps == 2
            return [
                {
                    "class_name": "bottle",
                    "score": 0.91,
                    "translation_xyz": [0.1, 0.0, 0.3],
                    "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    "width_m": 0.04,
                    "object_length_m": 0.12,
                }
            ]

    payload = module.build_graspnet_payload(
        backend=FakeBackend(),
        color_bgr=np.zeros((4, 4, 3), dtype=np.uint8),
        depth_mm=np.ones((4, 4), dtype=np.uint16) * 500,
        detections_payload={"detections": [{"class_name": "bottle", "confidence": 0.9}]},
        camera_info={"depth_frame_id": "camera_depth_frame"},
        max_grasps=2,
    )

    assert payload["backend_configured"] is True
    assert payload["frame_id"] == "camera_depth_frame"
    assert payload["candidates"][0]["source"] == "windows_graspnet_baseline"
    assert payload["candidates"][0]["score"] == 0.91
