from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_build_sequence_moves_pregrasp_grasp_closes_then_lifts():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.30, -0.10, 0.25),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.30, -0.10, 0.17),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(
        close_position_m=0.025,
        close_max_effort=0.3,
        lift_z_m=0.08,
    )

    stages = build_visual_grasp_sequence(pregrasp, grasp, config)

    assert [stage.kind for stage in stages] == [
        "move",
        "move",
        "gripper",
        "move",
    ]
    assert [stage.name for stage in stages] == [
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
        "lift",
    ]
    assert stages[0].pose == pregrasp
    assert stages[1].pose == grasp
    assert stages[2].gripper_position_m == pytest.approx(0.025)
    assert stages[2].gripper_max_effort == pytest.approx(0.3)
    assert stages[3].pose.position == pytest.approx((0.30, -0.10, 0.25))


def test_build_sequence_can_open_before_approach():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pose = PoseTarget(
        position=(0.1, 0.2, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(open_before_approach=True)

    stages = build_visual_grasp_sequence(pose, pose, config)

    assert [stage.name for stage in stages] == [
        "open_gripper",
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
        "lift",
    ]
    assert stages[0].kind == "gripper"
    assert stages[0].gripper_position_m == pytest.approx(config.open_position_m)


def test_build_sequence_rejects_too_low_grasp():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.2, 0.0, 0.2),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.2, 0.0, 0.08),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(min_grasp_z_m=0.12)

    with pytest.raises(ValueError, match="below minimum"):
        build_visual_grasp_sequence(pregrasp, grasp, config)



def test_build_sequence_uses_jaw_width_for_open_close():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.30, -0.10, 0.25),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.30, -0.10, 0.17),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(
        open_before_approach=True,
        auto_gripper_width=True,
        detected_jaw_width_m=0.04,
        open_clearance_m=0.02,
        close_margin_m=0.012,
        min_open_position_m=0.035,
        max_open_position_m=0.09,
        min_close_position_m=0.006,
        max_close_position_m=0.08,
    )

    stages = build_visual_grasp_sequence(pregrasp, grasp, config)

    assert [stage.name for stage in stages] == [
        "open_gripper",
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
        "lift",
    ]
    assert stages[0].gripper_position_m == pytest.approx(0.06)
    assert stages[3].gripper_position_m == pytest.approx(0.028)


def test_auto_gripper_width_clamps_to_safe_range():
    from rebotarm_vision.visual_grasp_sequence import VisualGraspSequenceConfig, resolve_gripper_widths

    config = VisualGraspSequenceConfig(
        auto_gripper_width=True,
        detected_jaw_width_m=0.20,
        open_clearance_m=0.02,
        close_margin_m=0.012,
        max_open_position_m=0.09,
        max_close_position_m=0.08,
    )

    open_width, close_width = resolve_gripper_widths(config)

    assert open_width == pytest.approx(0.09)
    assert close_width == pytest.approx(0.08)


def test_gripper_policy_adapts_width_and_effort_to_object_size():
    from rebotarm_vision.gripper_policy import GripperPolicyConfig, resolve_gripper_command

    narrow = resolve_gripper_command(
        jaw_width_m=0.028,
        object_length_m=0.08,
        class_name="bottle",
        config=GripperPolicyConfig(auto_width=True, auto_effort=True),
    )
    wide = resolve_gripper_command(
        jaw_width_m=0.060,
        object_length_m=0.20,
        class_name="bottle",
        config=GripperPolicyConfig(auto_width=True, auto_effort=True),
    )

    assert narrow.allowed
    assert wide.allowed
    assert narrow.open_width_m < wide.open_width_m
    assert narrow.close_width_m < wide.close_width_m
    assert narrow.max_effort < wide.max_effort
    assert wide.max_effort == pytest.approx(0.48)


def test_gripper_policy_rejects_objects_outside_gripper_range():
    from rebotarm_vision.gripper_policy import GripperPolicyConfig, resolve_gripper_command

    command = resolve_gripper_command(
        jaw_width_m=0.20,
        object_length_m=0.25,
        class_name="box",
        config=GripperPolicyConfig(max_open_width_m=0.09),
    )

    assert not command.allowed
    assert "too wide" in command.reason


def test_sequence_adds_safe_retreat_after_lift_before_safe_home():
    from rebotarm_vision.gripper_policy import GripperCommand
    from rebotarm_vision.retreat_policy import RetreatPolicyConfig
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.22, -0.05, 0.20),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.30, -0.05, 0.13),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(
        gripper_command=GripperCommand(
            open_width_m=0.065,
            close_width_m=0.035,
            max_effort=0.42,
            allowed=True,
            reason="ok",
        ),
        retreat_policy=RetreatPolicyConfig(
            enabled=True,
            min_lift_z_m=0.24,
            retreat_distance_m=0.06,
            retreat_axis_xyz=(-1.0, 0.0, 0.0),
        ),
        include_safe_home=True,
    )

    stages = build_visual_grasp_sequence(pregrasp, grasp, config)

    assert [stage.name for stage in stages] == [
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
        "lift",
        "safe_retreat",
        "safe_home",
    ]
    assert stages[2].gripper_position_m == pytest.approx(0.035)
    assert stages[2].gripper_max_effort == pytest.approx(0.42)
    assert stages[3].pose.position[2] == pytest.approx(0.24)
    assert stages[4].pose.position == pytest.approx((0.24, -0.05, 0.24))
    assert stages[5].kind == "safe_home"
