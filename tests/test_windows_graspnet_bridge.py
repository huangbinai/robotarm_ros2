from __future__ import annotations

import importlib.util
import json
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


def test_windows_graspnet_bridge_writes_backend_missing_payload(tmp_path):
    module = _load_bridge()
    output = tmp_path / "graspnet_candidates.json"

    module.write_backend_missing_payload(output, reason="module not found")

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["source"] == "windows_graspnet_baseline"
    assert payload["backend_configured"] is False
    assert payload["candidates"] == []
    assert "module not found" in payload["error"]


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
