from __future__ import annotations

from dataclasses import dataclass

from rebotarm_dashboard.panel_config import build_panel_config
from rebotarm_teach.teach_replay_parameters import TEACH_REPLAY_PARAMETER_DEFAULTS


@dataclass(frozen=True)
class _Parameter:
    value: object


def test_builds_browser_payload_with_typed_values() -> None:
    values = dict(TEACH_REPLAY_PARAMETER_DEFAULTS)
    values.update(
        {
            "record_path": "records/demo.jsonl",
            "replay_speed": 0.75,
            "web_execute_enabled": True,
            "web_execute_max_delta_rad": 1.5,
            "web_execute_max_joint_speed_rad_s": 1.25,
            "web_execute_min_duration": 1.0,
            "web_execute_max_duration": 8.0,
            "web_keyboard_default_step_rad": 0.02,
            "web_keyboard_min_step_rad": 0.005,
            "web_keyboard_max_step_rad": 0.10,
            "web_keyboard_default_duration": 0.2,
            "web_keyboard_min_duration": 0.1,
            "web_keyboard_max_duration": 2.0,
            "web_keyboard_default_speed_rad_s": 0.5,
            "web_gripper_max_effort": 0.3,
            "web_gripper_max_effort_limit": 1.5,
            "use_hardware": True,
            "panel_mode": "control",
            "execution_mode": "dry_run",
        }
    )

    payload = build_panel_config(
        get_parameter=lambda name: _Parameter(values[name]),
        has_parameter=lambda name: name in values,
        joint_names=("joint1", "joint2"),
        joint_limits={"joint1": (-1, 1), "joint2": (-2, 2)},
        joint_velocity_limits={"joint1": 3, "joint2": 1.8},
        gripper_limits=(0, 0.085),
    )

    assert payload["joint_names"] == ["joint1", "joint2"]
    assert payload["joint_limits"]["joint1"] == [-1.0, 1.0]
    assert payload["joint_velocity_limits"]["joint2"] == 1.8
    assert payload["gripper_limits"] == [0.0, 0.085]
    assert payload["web_execute"]["enabled"] is True
    assert payload["teach"]["record_path"] == "records/demo.jsonl"
    assert payload["teach"]["replay_speed"] == 0.75
    assert payload["teach"]["max_replay_velocity_rad_s_by_joint"] == [
        3.0,
        3.0,
        3.0,
        1.8,
        1.8,
        1.8,
    ]
    assert payload["teach"]["use_hardware"] is True


def test_missing_use_hardware_parameter_defaults_to_false() -> None:
    values = dict(TEACH_REPLAY_PARAMETER_DEFAULTS)
    values.update(
        {
            "record_path": "records/demo.jsonl",
            "replay_speed": 1.0,
            "web_execute_enabled": False,
            "web_execute_max_delta_rad": 1.5,
            "web_execute_max_joint_speed_rad_s": 1.5,
            "web_execute_min_duration": 1.0,
            "web_execute_max_duration": 8.0,
            "web_keyboard_default_step_rad": 0.02,
            "web_keyboard_min_step_rad": 0.005,
            "web_keyboard_max_step_rad": 0.10,
            "web_keyboard_default_duration": 0.2,
            "web_keyboard_min_duration": 0.1,
            "web_keyboard_max_duration": 2.0,
            "web_keyboard_default_speed_rad_s": 0.5,
            "web_gripper_max_effort": 0.3,
            "web_gripper_max_effort_limit": 1.5,
            "panel_mode": "check",
            "execution_mode": "dry_run",
        }
    )

    payload = build_panel_config(
        get_parameter=lambda name: _Parameter(values[name]),
        has_parameter=lambda _name: False,
        joint_names=(),
        joint_limits={},
        joint_velocity_limits={},
        gripper_limits=(0.0, 0.085),
    )

    assert payload["teach"]["use_hardware"] is False
