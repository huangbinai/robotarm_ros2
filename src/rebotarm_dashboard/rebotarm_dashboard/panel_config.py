from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Any


def build_panel_config(
    *,
    get_parameter: Callable[[str], Any],
    has_parameter: Callable[[str], bool],
    joint_names: Sequence[str],
    joint_limits: Mapping[str, tuple[float, float]],
    joint_velocity_limits: Mapping[str, float],
    gripper_limits: tuple[float, float],
) -> dict:
    """Build the browser-facing dashboard configuration payload."""

    def value(name: str) -> Any:
        return get_parameter(name).value

    return {
        "joint_names": list(joint_names),
        "joint_limits": {
            name: [float(lower), float(upper)]
            for name, (lower, upper) in joint_limits.items()
        },
        "joint_velocity_limits": {
            name: float(limit) for name, limit in joint_velocity_limits.items()
        },
        "gripper_limits": [float(gripper_limits[0]), float(gripper_limits[1])],
        "web_execute": {
            "enabled": bool(value("web_execute_enabled")),
            "max_delta_rad": float(value("web_execute_max_delta_rad")),
            "max_joint_speed_rad_s": float(
                value("web_execute_max_joint_speed_rad_s")
            ),
            "min_duration": float(value("web_execute_min_duration")),
            "max_duration": float(value("web_execute_max_duration")),
        },
        "web_keyboard": {
            "step_rad": float(value("web_keyboard_default_step_rad")),
            "min_step_rad": float(value("web_keyboard_min_step_rad")),
            "max_step_rad": float(value("web_keyboard_max_step_rad")),
            "duration": float(value("web_keyboard_default_duration")),
            "min_duration": float(value("web_keyboard_min_duration")),
            "max_duration": float(value("web_keyboard_max_duration")),
            "max_joint_speed_rad_s": float(
                value("web_keyboard_default_speed_rad_s")
            ),
        },
        "web_gripper": {
            "max_effort": float(value("web_gripper_max_effort")),
            "max_effort_limit": float(value("web_gripper_max_effort_limit")),
        },
        "teach": {
            "record_path": str(value("record_path")),
            "direct_threshold": float(value("direct_threshold")),
            "align_threshold": float(value("align_threshold")),
            "align_duration": float(value("align_duration")),
            "align_duration_auto": bool(value("align_duration_auto")),
            "align_target_speed_rad_s": float(value("align_target_speed_rad_s")),
            "align_min_duration": float(value("align_min_duration")),
            "align_max_duration": float(value("align_max_duration")),
            "align_steps": int(value("align_steps")),
            "replay_speed": float(value("replay_speed")),
            "green_jump_rad": float(value("green_jump_rad")),
            "yellow_jump_rad": float(value("yellow_jump_rad")),
            "yellow_max_speed": float(value("yellow_max_speed")),
            "max_replay_velocity_rad_s": float(value("max_replay_velocity_rad_s")),
            "max_replay_velocity_rad_s_by_joint": [
                float(item) for item in value("max_replay_velocity_rad_s_by_joint")
            ],
            "max_replay_acceleration_rad_s2": float(
                value("max_replay_acceleration_rad_s2")
            ),
            "max_replay_jerk_rad_s3": float(value("max_replay_jerk_rad_s3")),
            "large_motion_span_rad": float(value("large_motion_span_rad")),
            "large_motion_total_rad": float(value("large_motion_total_rad")),
            "large_motion_max_speed": float(value("large_motion_max_speed")),
            "start_hold_sec": float(value("start_hold_sec")),
            "soft_start_duration": float(value("soft_start_duration")),
            "soft_start_steps": int(value("soft_start_steps")),
            "first_hold_sec": float(value("first_hold_sec")),
            "final_hold_sec": float(value("final_hold_sec")),
            "use_moveit_start_align": bool(value("use_moveit_start_align")),
            "moveit_start_skip_threshold": float(
                value("moveit_start_skip_threshold")
            ),
            "collision_check_enabled": bool(value("collision_check_enabled")),
            "collision_check_max_samples": int(value("collision_check_max_samples")),
            "smoothing_enabled": bool(value("smoothing_enabled")),
            "smoothing_window": int(value("smoothing_window")),
            "filter_enabled": bool(value("filter_enabled")),
            "filter_cutoff_hz": float(value("filter_cutoff_hz")),
            "filter_sample_rate_hz": float(value("filter_sample_rate_hz")),
            "resample_enabled": bool(value("resample_enabled")),
            "resample_rate_hz": float(value("resample_rate_hz")),
            "time_parameterization_method": str(
                value("time_parameterization_method")
            ),
            "max_prepared_jump_rad": float(value("max_prepared_jump_rad")),
            "use_hardware": bool(value("use_hardware"))
            if has_parameter("use_hardware")
            else False,
        },
        "panel_mode": str(value("panel_mode")),
        "execution_mode": str(value("execution_mode")),
    }
