from __future__ import annotations

from dataclasses import dataclass

import pytest

from rebotarm_vision.gripper_policy import GripperCommand
from rebotarm_vision.visual_grasp_parameter_adapter import (
    VisualGraspParameterAdapter,
)


@dataclass(frozen=True)
class _Parameter:
    value: object


def _adapter(**overrides) -> VisualGraspParameterAdapter:
    values = {
        "auto_gripper_width": True,
        "auto_gripper_effort": True,
        "open_position_m": 0.085,
        "close_position_m": 0.025,
        "close_max_effort": 0.4,
        "open_clearance_m": 0.001,
        "close_margin_m": 0.012,
        "min_open_position_m": 0.035,
        "max_open_position_m": 0.085,
        "min_close_position_m": 0.006,
        "max_close_position_m": 0.08,
        "min_gripper_effort": 0.22,
        "max_gripper_effort": 0.60,
        "max_allowed_grasp_width_m": 0.085,
        "safe_retreat_enabled": True,
        "dynamic_retreat_enabled": True,
        "safe_retreat_min_lift_z_m": 0.12,
        "safe_retreat_distance_m": 0.06,
        "safe_retreat_axis_xyz": [-1.0, 0.0, 0.5],
        "open_before_approach": False,
        "lift_z_m": 0.04,
        "min_grasp_z_m": 0.0,
        "safe_home_after_grasp": False,
        "auto_retry_enabled": True,
        "auto_retry_max_attempts": 3,
        "safe_retreat_before_retry": True,
        "approach_visual_servo_max_step_m": 0.02,
        "approach_visual_servo_position_tolerance_m": 0.008,
        "place_after_grasp_enabled": True,
        "place_position_xyz": [0.20, -0.20, 0.25],
        "place_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "place_open_position_m": 0.08,
        "place_open_max_effort": 0.25,
        "place_retreat_z_m": 0.06,
        "fixed_grasp_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "base_approach_axis_xyz": [1.0, 0.0, 0.0],
        "base_pregrasp_distance_m": 0.08,
    }
    values.update(overrides)
    return VisualGraspParameterAdapter(lambda name: _Parameter(values[name]))


def test_builds_gripper_and_sequence_configs() -> None:
    adapter = _adapter()
    command = GripperCommand(
        open_width_m=0.06,
        close_width_m=0.04,
        max_effort=0.35,
    )

    gripper = adapter.gripper_policy()
    sequence = adapter.sequence(
        detected_jaw_width_m=0.05,
        gripper_command=command,
    )

    assert gripper.max_open_width_m == pytest.approx(0.085)
    assert gripper.max_allowed_width_m == pytest.approx(0.085)
    assert sequence.detected_jaw_width_m == pytest.approx(0.05)
    assert sequence.gripper_command is command
    assert sequence.retreat_policy.retreat_axis_xyz == (-1.0, 0.0, 0.5)
    assert sequence.retreat_policy.min_lift_z_m == pytest.approx(0.12)


def test_builds_retry_recovery_servo_place_and_pose_configs() -> None:
    adapter = _adapter()

    assert adapter.retry().max_attempts == 3
    assert adapter.recovery().safe_retreat_before_retry is True
    assert adapter.visual_servo().position_tolerance_m == pytest.approx(0.008)
    assert adapter.place().place_position_xyz == (0.20, -0.20, 0.25)

    pose = adapter.base_axis_pose(
        tcp_offset_xyz=(-0.04, 0.0, 0.0),
        target_base_offset_xyz=(0.01, 0.02, 0.03),
        grasp_z_offset_m=0.005,
    )
    assert pose.approach_axis_xyz == (1.0, 0.0, 0.0)
    assert pose.tcp_offset_xyz == (-0.04, 0.0, 0.0)
    assert pose.target_base_offset_xyz == (0.01, 0.02, 0.03)
    assert pose.grasp_z_offset_m == pytest.approx(0.005)


def test_tuple_parameter_length_is_validated() -> None:
    adapter = _adapter(place_orientation_xyzw=[0.0, 0.0, 1.0])

    with pytest.raises(
        ValueError,
        match="place_orientation_xyzw must contain exactly 4 values",
    ):
        adapter.place()
