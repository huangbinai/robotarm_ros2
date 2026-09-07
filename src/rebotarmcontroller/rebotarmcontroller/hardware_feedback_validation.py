from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any


def validated_gripper_feedback_values(
    state: Any,
    *,
    open_angle_rad: float,
    coordinate_tolerance_rad: float,
    closed_feedback_tolerance_rad: float,
) -> tuple[float, float, float, int]:
    if state is None:
        raise RuntimeError("gripper feedback unavailable")
    values = (float(state.pos), float(state.vel), float(state.torq))
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("gripper feedback contains non-finite values")
    position, velocity, torque = values
    if not (
        open_angle_rad - coordinate_tolerance_rad
        <= position
        <= closed_feedback_tolerance_rad
    ):
        raise RuntimeError(
            f"gripper coordinate invalid: {position:.6f} rad outside feedback range "
            f"[{open_angle_rad:.6f}, {closed_feedback_tolerance_rad:.6f}]"
        )
    return position, velocity, torque, int(state.status_code)


def validate_feedback_state(
    label: str,
    state: Any,
    *,
    joint_position_limits_rad: Mapping[str, tuple[float, float]],
    gripper_open_angle_rad: float,
    gripper_coordinate_tolerance_rad: float,
    gripper_closed_feedback_tolerance_rad: float,
) -> None:
    if label == "gripper":
        validated_gripper_feedback_values(
            state,
            open_angle_rad=gripper_open_angle_rad,
            coordinate_tolerance_rad=gripper_coordinate_tolerance_rad,
            closed_feedback_tolerance_rad=gripper_closed_feedback_tolerance_rad,
        )
        return
    if state is None:
        raise RuntimeError(f"{label} feedback unavailable")
    values = (float(state.pos), float(state.vel), float(state.torq))
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"{label} feedback contains non-finite values")
    limits = joint_position_limits_rad.get(label)
    if limits is None:
        raise RuntimeError(f"no feedback limit configured for {label}")
    if not limits[0] <= values[0] <= limits[1]:
        raise RuntimeError(
            f"{label} position {values[0]:.6f} rad outside feedback range "
            f"[{limits[0]:.6f}, {limits[1]:.6f}]"
        )
