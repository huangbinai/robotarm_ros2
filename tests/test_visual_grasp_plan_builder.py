from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

import pytest

from rebotarm_vision.gripper_policy import GripperPolicyConfig
from rebotarm_vision.place_task_policy import PlaceTaskConfig
from rebotarm_vision.retreat_policy import RetreatPolicyConfig
from rebotarm_vision.visual_grasp_plan_builder import (
    VisualGraspPlanBuilder,
    VisualGraspTargetConfig,
)
from rebotarm_vision.visual_grasp_sequence import VisualGraspSequenceConfig


def _pose(x: float, y: float, z: float):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


class _Parameters:
    def gripper_policy(self) -> GripperPolicyConfig:
        return GripperPolicyConfig(
            auto_width=False,
            auto_effort=False,
            default_open_width_m=0.085,
            default_close_width_m=0.025,
            default_max_effort=0.4,
        )

    def sequence(self, *, detected_jaw_width_m, gripper_command):
        return VisualGraspSequenceConfig(
            detected_jaw_width_m=detected_jaw_width_m,
            gripper_command=gripper_command,
            lift_z_m=0.04,
            retreat_policy=RetreatPolicyConfig(enabled=False),
        )

    def place(self) -> PlaceTaskConfig:
        return PlaceTaskConfig(enabled=False)

    def base_axis_pose(self, **_kwargs):
        raise AssertionError("base-axis policy should not be used")


def _builder(*, pose_policy: str = "visual_pose") -> VisualGraspPlanBuilder:
    return VisualGraspPlanBuilder(
        parameters=_Parameters(),
        target_config=VisualGraspTargetConfig(
            tcp_offset_xyz=(0.0, 0.0, 0.0),
            target_base_offset_xyz=(0.01, -0.02, 0.03),
            grasp_base_z_offset_m=0.04,
        ),
        pose_policy=lambda: pose_policy,
        transform_plan_pose=lambda _plan, pose: deepcopy(pose),
    )


def _plan(*, source: str = "camera"):
    return SimpleNamespace(
        source=source,
        jaw_width=0.05,
        candidate=SimpleNamespace(
            jaw_width=0.04,
            object_length=0.12,
            class_name="box",
        ),
        pregrasp_pose=_pose(0.20, 0.10, 0.30),
        grasp_pose=_pose(0.25, 0.10, 0.20),
    )


def test_plan_builder_applies_legacy_offsets_and_builds_sequence() -> None:
    stages = _builder().build_sequence(_plan())

    assert [stage.name for stage in stages] == [
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
        "lift",
    ]
    assert stages[0].pose.position == pytest.approx((0.21, 0.08, 0.33))
    assert stages[1].pose.position == pytest.approx((0.26, 0.08, 0.27))
    assert stages[2].detected_jaw_width_m == pytest.approx(0.05)


def test_filtered_plan_preserves_filtered_targets_without_offsets() -> None:
    plan = _plan(source="candidate_ik_filter")

    pregrasp, grasp = _builder(pose_policy="unsupported").build_motion_targets(plan)

    assert pregrasp.position == pytest.approx((0.20, 0.10, 0.30))
    assert grasp.position == pytest.approx((0.25, 0.10, 0.20))


def test_plan_builder_rejects_unknown_pose_policy() -> None:
    with pytest.raises(ValueError, match="unsupported pose_policy: unknown"):
        _builder(pose_policy="unknown").build_motion_targets(_plan())


def test_post_grasp_builder_preserves_visual_ready_rules() -> None:
    stages = _builder().append_post_grasp_stages(
        [],
        return_visual_ready_enabled=True,
        place_after_grasp_enabled=False,
    )

    assert [stage.name for stage in stages] == [
        "plan_visual_ready",
        "return_visual_ready",
    ]
