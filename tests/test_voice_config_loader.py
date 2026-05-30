from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import load_voice_control_config


def test_load_default_config_contains_v1_named_poses():
    config_root = SRC / "config"

    config = load_voice_control_config(config_root)

    assert set(config.named_poses) == {"home", "camera_pose", "place_left"}
    assert config.safety_limits["execution_mode"] == "dry_run"


def test_intents_include_stop_highest_priority():
    config_root = SRC / "config"

    config = load_voice_control_config(config_root)

    stop = config.intents["stop_motion"]
    assert stop["priority"] == "highest"
    assert "急停" in stop["patterns"]
