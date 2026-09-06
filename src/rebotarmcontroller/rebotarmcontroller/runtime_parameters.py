from __future__ import annotations

import math
import re


_NAMESPACE_SEGMENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_MAX_MOVE_TO_POSE_DURATION_SEC = 300.0


def finite_rate_hz(
    name: str,
    value: object,
    *,
    minimum: float = 1.0,
    maximum: float = 500.0,
) -> float:
    rate = float(value)
    if not math.isfinite(rate) or not minimum <= rate <= maximum:
        raise ValueError(
            f"{name} must be finite and within [{minimum:g}, {maximum:g}] Hz"
        )
    return rate


def arm_namespace(value: object) -> str:
    namespace = str(value).strip().strip("/")
    segments = namespace.split("/") if namespace else []
    if not segments or any(not _NAMESPACE_SEGMENT.fullmatch(part) for part in segments):
        raise ValueError(
            "arm_namespace must contain non-empty ROS name segments using only "
            "letters, digits, and underscores; each segment must start with a letter "
            "or underscore"
        )
    return namespace


def command_arbitration(value: object) -> str:
    mode = str(value).strip().lower()
    if mode not in ("reject", "preempt"):
        raise ValueError("cmd_arbitration must be 'reject' or 'preempt'")
    return mode


def validate_move_to_pose_goal(goal) -> float:
    duration = float(goal.duration)
    if not math.isfinite(duration) or not (
        0.01 <= duration <= _MAX_MOVE_TO_POSE_DURATION_SEC
    ):
        raise ValueError(
            "move_to_pose duration must be finite and within [0.01, 300] seconds"
        )

    pose = goal.target_pose
    pose_values = (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    if not all(math.isfinite(value) for value in pose_values):
        raise ValueError("move_to_pose target pose must contain only finite values")
    quaternion_norm = math.sqrt(sum(value * value for value in pose_values[3:]))
    if quaternion_norm <= 1e-6:
        raise ValueError("move_to_pose target orientation quaternion must be non-zero")
    return duration
