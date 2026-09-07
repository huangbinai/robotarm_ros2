from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .grasp_retry_policy import RetryPolicyConfig
from .gripper_policy import GripperCommand, GripperPolicyConfig
from .place_task_policy import PlaceTaskConfig
from .retreat_policy import RetreatPolicyConfig
from .trajectory_recovery_policy import RecoveryConfig
from .visual_grasp_pose_policy import BaseAxisGraspPolicyConfig
from .visual_grasp_sequence import VisualGraspSequenceConfig
from .visual_servo_policy import VisualServoApproachConfig


class VisualGraspParameterAdapter:
    """Translate ROS parameter values into visual grasp policy configs."""

    def __init__(self, get_parameter: Callable[[str], Any]) -> None:
        self._get_parameter = get_parameter

    def _value(self, name: str) -> Any:
        return self._get_parameter(name).value

    def tuple_n(self, name: str, expected_len: int) -> tuple[float, ...]:
        values = list(self._value(name))
        if len(values) != expected_len:
            raise ValueError(f"{name} must contain exactly {expected_len} values")
        return tuple(float(value) for value in values)

    def tuple3(self, name: str) -> tuple[float, float, float]:
        values = self.tuple_n(name, 3)
        return (values[0], values[1], values[2])

    def gripper_policy(self) -> GripperPolicyConfig:
        return GripperPolicyConfig(
            auto_width=bool(self._value("auto_gripper_width")),
            auto_effort=bool(self._value("auto_gripper_effort")),
            default_open_width_m=float(self._value("open_position_m")),
            default_close_width_m=float(self._value("close_position_m")),
            default_max_effort=float(self._value("close_max_effort")),
            open_clearance_m=float(self._value("open_clearance_m")),
            close_margin_m=float(self._value("close_margin_m")),
            min_open_width_m=float(self._value("min_open_position_m")),
            max_open_width_m=float(self._value("max_open_position_m")),
            min_close_width_m=float(self._value("min_close_position_m")),
            max_close_width_m=float(self._value("max_close_position_m")),
            min_effort=float(self._value("min_gripper_effort")),
            max_effort=float(self._value("max_gripper_effort")),
            max_allowed_width_m=float(self._value("max_allowed_grasp_width_m")),
        )

    def retreat_policy(self) -> RetreatPolicyConfig:
        return RetreatPolicyConfig(
            enabled=bool(self._value("safe_retreat_enabled")),
            dynamic_retreat_enabled=bool(self._value("dynamic_retreat_enabled")),
            min_lift_z_m=float(self._value("safe_retreat_min_lift_z_m")),
            retreat_distance_m=float(self._value("safe_retreat_distance_m")),
            retreat_axis_xyz=self.tuple3("safe_retreat_axis_xyz"),
        )

    def sequence(
        self,
        *,
        detected_jaw_width_m: float,
        gripper_command: GripperCommand,
    ) -> VisualGraspSequenceConfig:
        return VisualGraspSequenceConfig(
            open_before_approach=bool(self._value("open_before_approach")),
            open_position_m=float(self._value("open_position_m")),
            close_position_m=float(self._value("close_position_m")),
            close_max_effort=float(self._value("close_max_effort")),
            lift_z_m=float(self._value("lift_z_m")),
            min_grasp_z_m=float(self._value("min_grasp_z_m")),
            auto_gripper_width=bool(self._value("auto_gripper_width")),
            detected_jaw_width_m=float(detected_jaw_width_m),
            open_clearance_m=float(self._value("open_clearance_m")),
            close_margin_m=float(self._value("close_margin_m")),
            min_open_position_m=float(self._value("min_open_position_m")),
            max_open_position_m=float(self._value("max_open_position_m")),
            min_close_position_m=float(self._value("min_close_position_m")),
            max_close_position_m=float(self._value("max_close_position_m")),
            gripper_command=gripper_command,
            retreat_policy=self.retreat_policy(),
            include_safe_home=bool(self._value("safe_home_after_grasp")),
        )

    def retry(self) -> RetryPolicyConfig:
        return RetryPolicyConfig(
            enabled=bool(self._value("auto_retry_enabled")),
            max_attempts=int(self._value("auto_retry_max_attempts")),
        )

    def recovery(self) -> RecoveryConfig:
        return RecoveryConfig(
            auto_retry_enabled=bool(self._value("auto_retry_enabled")),
            safe_retreat_before_retry=bool(
                self._value("safe_retreat_before_retry")
            ),
        )

    def visual_servo(self) -> VisualServoApproachConfig:
        return VisualServoApproachConfig(
            max_step_m=float(self._value("approach_visual_servo_max_step_m")),
            position_tolerance_m=float(
                self._value("approach_visual_servo_position_tolerance_m")
            ),
        )

    def place(self) -> PlaceTaskConfig:
        return PlaceTaskConfig(
            enabled=bool(self._value("place_after_grasp_enabled")),
            place_position_xyz=self.tuple3("place_position_xyz"),
            place_orientation_xyzw=self.tuple_n("place_orientation_xyzw", 4),
            open_position_m=float(self._value("place_open_position_m")),
            open_max_effort=float(self._value("place_open_max_effort")),
            retreat_z_m=float(self._value("place_retreat_z_m")),
        )

    def base_axis_pose(
        self,
        *,
        tcp_offset_xyz: tuple[float, float, float],
        target_base_offset_xyz: tuple[float, float, float],
        grasp_z_offset_m: float,
    ) -> BaseAxisGraspPolicyConfig:
        return BaseAxisGraspPolicyConfig(
            fixed_orientation_xyzw=self.tuple_n(
                "fixed_grasp_orientation_xyzw",
                4,
            ),
            approach_axis_xyz=self.tuple3("base_approach_axis_xyz"),
            pregrasp_distance_m=float(self._value("base_pregrasp_distance_m")),
            tcp_offset_xyz=tcp_offset_xyz,
            target_base_offset_xyz=target_base_offset_xyz,
            grasp_z_offset_m=float(grasp_z_offset_m),
        )
