from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .grasp_preview_sender_node import apply_tcp_offset_to_pose
from .gripper_policy import resolve_gripper_command
from .place_task_policy import build_place_stages
from .visual_grasp_pose_policy import build_base_axis_grasp_targets
from .visual_grasp_sequence import (
    PoseTarget,
    VisualGraspStage,
    append_visual_ready_return_stages,
    build_visual_grasp_sequence,
)


@dataclass(frozen=True)
class VisualGraspTargetConfig:
    tcp_offset_xyz: tuple[float, float, float]
    target_base_offset_xyz: tuple[float, float, float]
    grasp_base_z_offset_m: float


def pose_to_target(pose: Any) -> PoseTarget:
    return PoseTarget(
        position=(
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ),
        orientation=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


class VisualGraspPlanBuilder:
    """Translate one perception plan into policy-owned execution stages."""

    def __init__(
        self,
        *,
        parameters: Any,
        target_config: VisualGraspTargetConfig,
        pose_policy: Callable[[], str],
        transform_plan_pose: Callable[[Any, Any], Any],
    ) -> None:
        self._parameters = parameters
        self._target_config = target_config
        self._pose_policy = pose_policy
        self._transform_plan_pose = transform_plan_pose

    def build_sequence(self, plan: Any) -> list[VisualGraspStage]:
        pregrasp, grasp = self.build_motion_targets(plan)
        detected_width = self.detected_jaw_width(plan)
        gripper_command = resolve_gripper_command(
            jaw_width_m=detected_width,
            object_length_m=self.detected_object_length(plan),
            class_name=str(getattr(plan.candidate, "class_name", "") or ""),
            config=self._parameters.gripper_policy(),
        )
        config = self._parameters.sequence(
            detected_jaw_width_m=detected_width,
            gripper_command=gripper_command,
        )
        return build_visual_grasp_sequence(pregrasp, grasp, config)

    def append_place_stages(
        self,
        stages: list[VisualGraspStage],
    ) -> list[VisualGraspStage]:
        return stages + build_place_stages(self._parameters.place())

    def append_post_grasp_stages(
        self,
        stages: list[VisualGraspStage],
        *,
        return_visual_ready_enabled: bool,
        place_after_grasp_enabled: bool,
    ) -> list[VisualGraspStage]:
        with_place = self.append_place_stages(stages)
        return append_visual_ready_return_stages(
            with_place,
            enabled=return_visual_ready_enabled,
            place_after_grasp_enabled=place_after_grasp_enabled,
        )

    def build_motion_targets(self, plan: Any) -> tuple[PoseTarget, PoseTarget]:
        if str(getattr(plan, "source", "")).strip() == "candidate_ik_filter":
            return self._build_filtered_motion_targets(plan)
        policy = self._pose_policy().strip().lower()
        if policy in ("visual_pose", "source_pose", "legacy"):
            return (
                self._convert_plan_pose(plan, plan.pregrasp_pose, 0.0),
                self._convert_plan_pose(
                    plan,
                    plan.grasp_pose,
                    self._target_config.grasp_base_z_offset_m,
                ),
            )
        if policy != "base_axis":
            raise ValueError(f"unsupported pose_policy: {policy}")
        grasp_pose = self._transform_plan_pose(plan, plan.grasp_pose)
        return build_base_axis_grasp_targets(
            grasp_position_xyz=(
                float(grasp_pose.position.x),
                float(grasp_pose.position.y),
                float(grasp_pose.position.z),
            ),
            config=self._parameters.base_axis_pose(
                tcp_offset_xyz=self._target_config.tcp_offset_xyz,
                target_base_offset_xyz=self._target_config.target_base_offset_xyz,
                grasp_z_offset_m=self._target_config.grasp_base_z_offset_m,
            ),
        )

    @staticmethod
    def detected_jaw_width(plan: Any) -> float:
        plan_width = float(getattr(plan, "jaw_width", 0.0) or 0.0)
        candidate_width = float(
            getattr(plan.candidate, "jaw_width", 0.0) or 0.0
        )
        return plan_width if plan_width > 0.0 else candidate_width

    @staticmethod
    def detected_object_length(plan: Any) -> float:
        return float(getattr(plan.candidate, "object_length", 0.0) or 0.0)

    def _build_filtered_motion_targets(
        self,
        plan: Any,
    ) -> tuple[PoseTarget, PoseTarget]:
        return (
            pose_to_target(
                self._transform_plan_pose(plan, plan.pregrasp_pose)
            ),
            pose_to_target(self._transform_plan_pose(plan, plan.grasp_pose)),
        )

    def _convert_plan_pose(
        self,
        plan: Any,
        pose: Any,
        z_offset_m: float,
    ) -> PoseTarget:
        converted = self._transform_plan_pose(plan, pose)
        converted = apply_tcp_offset_to_pose(
            converted,
            self._target_config.tcp_offset_xyz,
        )
        base_offset = self._target_config.target_base_offset_xyz
        converted.position.x = round(float(converted.position.x) + base_offset[0], 6)
        converted.position.y = round(float(converted.position.y) + base_offset[1], 6)
        converted.position.z = round(
            float(converted.position.z) + base_offset[2] + z_offset_m,
            6,
        )
        return pose_to_target(converted)
