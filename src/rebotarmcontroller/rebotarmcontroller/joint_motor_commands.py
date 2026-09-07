from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class JointMotorCommand:
    mode: int
    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    torque_nm: float
    velocity_limit_rad_s: float


def resolve_joint_motor_command(
    joint_name: str,
    command: Any,
    *,
    feedback_state: Any,
    config: Any,
    position_limits_rad: tuple[float, float],
) -> JointMotorCommand:
    position = (
        float(command.pos)
        if command.use_pos
        else float(feedback_state.pos if feedback_state is not None else 0.0)
    )
    velocity = (
        float(command.vel)
        if command.use_vel
        else float(feedback_state.vel if feedback_state is not None else 0.0)
    )
    kp = float(command.kp) if command.use_kp else float(config.kp)
    kd = float(command.kd) if command.use_kd else float(config.kd)
    torque = float(command.tau) if command.use_tau else 0.0
    velocity_limit = (
        float(command.vlim) if command.use_vlim else float(config.vlim)
    )

    values = {
        "pos": position,
        "vel": velocity,
        "kp": kp,
        "kd": kd,
        "tau": torque,
        "vlim": velocity_limit,
    }
    invalid = [name for name, value in values.items() if not np.isfinite(value)]
    if invalid:
        raise ValueError(
            "joint motor command contains non-finite " + ", ".join(invalid)
        )
    if kp < 0.0 or kd < 0.0 or velocity_limit <= 0.0:
        raise ValueError(
            "joint motor kp/kd must be non-negative and vlim must be positive"
        )
    lower, upper = position_limits_rad
    if command.use_pos and not lower <= position <= upper:
        raise ValueError(
            f"{joint_name} position command {position:.6f} outside "
            f"[{lower:.6f}, {upper:.6f}]"
        )
    return JointMotorCommand(
        mode=int(command.mode),
        position_rad=position,
        velocity_rad_s=velocity,
        kp=kp,
        kd=kd,
        torque_nm=torque,
        velocity_limit_rad_s=velocity_limit,
    )


def dispatch_joint_motor_command(
    motor: Any,
    command: JointMotorCommand,
    *,
    joint_name: str,
) -> None:
    if command.mode == 0:
        motor.send_mit(
            command.position_rad,
            command.velocity_rad_s,
            command.kp,
            command.kd,
            command.torque_nm,
        )
        return
    if command.mode == 1:
        motor.send_pos_vel(
            command.position_rad,
            command.velocity_limit_rad_s,
        )
        return
    if command.mode == 2:
        if not hasattr(motor, "send_vel"):
            raise RuntimeError(f"{joint_name} does not support send_vel")
        motor.send_vel(command.velocity_rad_s)
        return
    raise ValueError(f"unsupported JointMotorCmd mode: {command.mode}")
