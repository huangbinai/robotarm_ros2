from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_windows_mjpeg_server():
    repo_root = Path(__file__).resolve().parents[3]
    module_path = repo_root / "tools" / "windows_mjpeg_server.py"
    spec = importlib.util.spec_from_file_location("windows_mjpeg_server", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_orbbec_grasp_defaults_are_fixed_but_classes_remain_overridable():
    module = _load_windows_mjpeg_server()

    parser = module.build_arg_parser()
    args = parser.parse_args(
        [
            "--classes",
            "bottle,cup",
            "--allowed-classes",
            "bottle,cup",
        ]
    )

    assert args.width == 1280
    assert args.height == 720
    assert args.fps == 30
    assert args.depth_width == 1280
    assert args.depth_height == 720
    assert args.depth_fps == 30
    assert args.conf_threshold == 0.25
    assert args.iou_threshold == 0.45
    assert args.detection_fps == 15.0
    assert args.jpeg_quality == 80
    assert args.classes == "bottle,cup"
    assert args.allowed_classes == "bottle,cup"
