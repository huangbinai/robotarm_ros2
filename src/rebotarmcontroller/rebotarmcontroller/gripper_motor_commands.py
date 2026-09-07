from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class GripperMotorCommand:
    mode: int
    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    torque_nm: float
    velocity_limit_rad_s: float


def resolve_gripper_motor_command(
    command: Any,
    *,
    feedback_state: Any,
    config: Any,
    open_soft_limit_rad: float,
    torque_limit_nm: float,
) -> GripperMotorCommand:
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

    values = (position, velocity, kp, kd, torque, velocity_limit)
    if not all(np.isfinite(value) for value in values):
        raise ValueError("gripper motor command values must be finite")
    if kp < 0.0 or kd < 0.0 or velocity_limit <= 0.0:
        raise ValueError(
            "gripper kp/kd must be non-negative and vlim must be positive"
        )
    if command.use_pos and not open_soft_limit_rad <= position <= 0.0:
        raise ValueError("gripper raw position command outside calibrated range")
    if command.use_tau and abs(torque) > torque_limit_nm:
        raise ValueError(
            f"gripper torque command exceeds {torque_limit_nm:g} N.m"
        )
    return GripperMotorCommand(
        mode=int(command.mode),
        position_rad=position,
        velocity_rad_s=velocity,
        kp=kp,
        kd=kd,
        torque_nm=torque,
        velocity_limit_rad_s=velocity_limit,
    )


def dispatch_gripper_motor_command(motor: Any, command: GripperMotorCommand) -> None:
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
            raise RuntimeError("gripper does not support send_vel")
        motor.send_vel(command.velocity_rad_s)
        return
    raise ValueError(f"unsupported JointMotorCmd mode: {command.mode}")


def send_safe_gripper_mit(
    motor: Any,
    *,
    position_rad: float,
    velocity_rad_s: float,
    kp: float,
    kd: float,
    torque_feedforward_nm: float,
    torque_limit_nm: float,
    current_position_rad: float,
    current_velocity_rad_s: float,
    open_soft_limit_rad: float,
    maximum_torque_nm: float,
) -> None:
    position_command = float(np.clip(position_rad, open_soft_limit_rad, 0.0))
    position_term = kp * (position_command - current_position_rad) + kd * (
        -current_velocity_rad_s
    )
    limit = float(np.clip(abs(torque_limit_nm), 0.05, maximum_torque_nm))
    safe_feedforward = (
        float(
            np.clip(
                position_term + torque_feedforward_nm,
                -limit,
                limit,
            )
        )
        - position_term
    )
    try:
        motor.send_mit(
            position_command,
            velocity_rad_s,
            kp,
            kd,
            safe_feedforward,
        )
    except Exception as exc:
        raise RuntimeError(f"gripper MIT command failed: {exc}") from exc
