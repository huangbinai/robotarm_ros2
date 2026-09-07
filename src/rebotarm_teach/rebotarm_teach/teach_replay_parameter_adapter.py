from __future__ import annotations

from collections.abc import Callable
from typing import Any

from rebotarm_motion.collision_precheck import CollisionPrecheckConfig
from rebotarm_motion.replay_runtime_monitor import ReplayRuntimeMonitorConfig
from rebotarm_motion.teach_replay_start_align_precheck import (
    MoveItStartAlignPrecheckConfig,
)
from rebotarm_motion.teach_replay_start_alignment import MoveItStartAlignmentConfig

from .teach_replay_coordinator import TeachReplayLimits
from .teach_replay_trajectory_builder import TeachReplayTrajectoryConfig
from .teach_replay_workflow import TeachReplayPreparationConfig


class TeachReplayParameterAdapter:
    """Translate ROS parameter values into teach replay domain configs."""

    def __init__(self, get_parameter: Callable[[str], Any]) -> None:
        self._get_parameter = get_parameter

    def _value(self, name: str) -> Any:
        return self._get_parameter(name).value

    def velocity_limits(
        self,
        joint_names: tuple[str, ...],
    ) -> float | tuple[float, ...]:
        scalar_limit = float(self._value("max_replay_velocity_rad_s"))
        values = self._value("max_replay_velocity_rad_s_by_joint")
        if isinstance(values, (list, tuple)) and len(values) == len(joint_names):
            return tuple(float(value) for value in values)
        return scalar_limit

    def preparation(
        self,
        joint_names: tuple[str, ...],
        *,
        replay_speed: float,
    ) -> TeachReplayPreparationConfig:
        return TeachReplayPreparationConfig(
            smoothing_enabled=bool(self._value("smoothing_enabled")),
            smoothing_window=int(self._value("smoothing_window")),
            filter_enabled=bool(self._value("filter_enabled")),
            filter_cutoff_hz=float(self._value("filter_cutoff_hz")),
            filter_sample_rate_hz=float(self._value("filter_sample_rate_hz")),
            resample_enabled=bool(self._value("resample_enabled")),
            resample_rate_hz=float(self._value("resample_rate_hz")),
            replay_speed=float(replay_speed),
            max_velocity_rad_s=self.velocity_limits(joint_names),
            max_acceleration_rad_s2=float(
                self._value("max_replay_acceleration_rad_s2")
            ),
            max_jerk_rad_s3=float(self._value("max_replay_jerk_rad_s3")),
            time_parameterization_method=str(
                self._value("time_parameterization_method")
            ),
            large_motion_span_rad=float(self._value("large_motion_span_rad")),
            large_motion_total_rad=float(self._value("large_motion_total_rad")),
            large_motion_max_speed=float(self._value("large_motion_max_speed")),
        )

    def moveit_precheck(self) -> MoveItStartAlignPrecheckConfig:
        return MoveItStartAlignPrecheckConfig(
            enabled=bool(self._value("use_moveit_start_align")),
            service=str(self._value("moveit_planning_service")),
            skip_threshold=float(self._value("moveit_start_skip_threshold")),
            joint_goal_tolerance=float(self._value("moveit_joint_goal_tolerance")),
            velocity_scaling=float(self._value("moveit_velocity_scaling")),
            acceleration_scaling=float(self._value("moveit_acceleration_scaling")),
        )

    def collision(
        self,
        *,
        default_joint_positions: tuple[tuple[str, float], ...] = (),
    ) -> CollisionPrecheckConfig:
        return CollisionPrecheckConfig(
            enabled=bool(self._value("collision_check_enabled")),
            service=str(self._value("collision_check_service")),
            group_name=str(self._value("collision_group_name")),
            max_samples=max(int(self._value("collision_check_max_samples")), 1),
            timeout_sec=max(
                float(self._value("collision_check_timeout_sec")),
                0.1,
            ),
            default_joint_positions=default_joint_positions,
        )

    def trajectory(
        self,
        joint_names: tuple[str, ...],
    ) -> TeachReplayTrajectoryConfig:
        return TeachReplayTrajectoryConfig(
            use_moveit_start_align=bool(self._value("use_moveit_start_align")),
            start_hold_sec=float(self._value("start_hold_sec")),
            soft_start_duration=float(self._value("soft_start_duration")),
            soft_start_steps=int(self._value("soft_start_steps")),
            first_hold_sec=float(self._value("first_hold_sec")),
            yellow_max_speed=float(self._value("yellow_max_speed")),
            initial_replay_delay_sec=float(self._value("initial_replay_delay_sec")),
            max_velocity_rad_s=self.velocity_limits(joint_names),
            max_acceleration_rad_s2=float(
                self._value("max_replay_acceleration_rad_s2")
            ),
            max_jerk_rad_s3=float(self._value("max_replay_jerk_rad_s3")),
        )

    def alignment(self) -> MoveItStartAlignmentConfig:
        return MoveItStartAlignmentConfig(
            start_hold_sec=float(self._value("start_hold_sec")),
            first_hold_sec=float(self._value("first_hold_sec")),
            skip_threshold=float(self._value("moveit_start_skip_threshold")),
            joint_goal_tolerance=float(self._value("moveit_joint_goal_tolerance")),
            velocity_scaling=float(self._value("moveit_velocity_scaling")),
            acceleration_scaling=float(self._value("moveit_acceleration_scaling")),
        )

    def limits(self) -> TeachReplayLimits:
        return TeachReplayLimits(
            max_prepared_jump_rad=float(self._value("max_prepared_jump_rad")),
            max_replay_acceleration_rad_s2=float(
                self._value("max_replay_acceleration_rad_s2")
            ),
            max_replay_jerk_rad_s3=float(self._value("max_replay_jerk_rad_s3")),
        )

    def runtime_monitor(self) -> ReplayRuntimeMonitorConfig:
        return ReplayRuntimeMonitorConfig(
            enabled=bool(self._value("replay_monitor_enabled")),
            start_grace_sec=float(self._value("replay_monitor_start_grace_sec")),
            violation_grace_sec=float(
                self._value("replay_monitor_violation_grace_sec")
            ),
            max_tracking_error_rad=float(self._value("max_tracking_error_rad")),
            max_live_velocity_rad_s=float(self._value("max_live_velocity_rad_s")),
        )
