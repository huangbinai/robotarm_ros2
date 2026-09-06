from __future__ import annotations

import math


def is_gripper_contact_sample(
    *,
    opening_m: float,
    closure_m: float,
    velocity_rad_s: float,
    torque_nm: float,
    min_opening_m: float,
    min_closure_m: float,
    max_velocity_rad_s: float,
    min_torque_nm: float,
) -> bool:
    values = (
        opening_m,
        closure_m,
        velocity_rad_s,
        torque_nm,
        min_opening_m,
        min_closure_m,
        max_velocity_rad_s,
        min_torque_nm,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return (
        float(opening_m) > float(min_opening_m)
        and float(closure_m) >= float(min_closure_m)
        and abs(float(velocity_rad_s)) <= float(max_velocity_rad_s)
        and abs(float(torque_nm)) >= float(min_torque_nm)
    )


def active_gripper_failure_reason(
    *,
    command_error: str | None,
    status_code: int,
) -> str | None:
    if command_error:
        return str(command_error)
    status = int(status_code)
    if status != 1:
        return f"gripper feedback status_code={status}, expected 1"
    return None
