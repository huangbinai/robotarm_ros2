from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ModeTransitionConfig:
    enabled: bool = True
    allow_velocity_mode: bool = False
    enter_ramp_duration_sec: float = 0.35
    enter_max_velocity_rad_s: float = 0.05
    exit_damping_duration_sec: float = 0.15
    exit_blend_duration_sec: float = 0.35
    exit_max_lock_velocity_rad_s: float = 0.05
    exit_velocity_wait_timeout_sec: float = 1.0
    gravity_kp: float = 7.0
    gravity_kd: float = 0.8
    hold_kp: float = 12.0
    hold_kd: float = 1.2
    pos_vel_settle_duration_sec: float = 0.15
    max_position_jump_rad: float = 0.02
    feedback_timeout_sec: float = 0.10
    transition_timeout_sec: float = 2.0


@dataclass(frozen=True)
class FeedbackSample:
    positions: np.ndarray
    velocities: np.ndarray
    age_sec: float = 0.0

    def __post_init__(self) -> None:
        positions = np.asarray(self.positions, dtype=np.float64).reshape(-1)
        velocities = np.asarray(self.velocities, dtype=np.float64).reshape(-1)
        if positions.shape != velocities.shape:
            raise ValueError("feedback position and velocity sizes differ")
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "velocities", velocities)


def smoothstep(value: float) -> float:
    s = min(max(float(value), 0.0), 1.0)
    return 3.0 * s * s - 2.0 * s * s * s


def blend_scalar(start: float, end: float, progress: float) -> float:
    alpha = smoothstep(progress)
    return float(start) + alpha * (float(end) - float(start))


def blend_enter_tau(gravity_tau: np.ndarray, progress: float) -> np.ndarray:
    return np.asarray(gravity_tau, dtype=np.float64) * smoothstep(progress)


def blend_exit_tau(gravity_tau: np.ndarray, progress: float) -> np.ndarray:
    return np.asarray(gravity_tau, dtype=np.float64) * (1.0 - smoothstep(progress))


def validate_mode_transition(
    source_mode: str,
    target_mode: str,
    config: ModeTransitionConfig,
) -> None:
    source = str(source_mode).strip().lower()
    target = str(target_mode).strip().lower()
    supported = {"mit", "pos_vel", "vel"}
    if source not in supported or target not in supported:
        raise ValueError(f"unsupported mode transition: {source} -> {target}")
    if "vel" in (source, target) and not config.allow_velocity_mode:
        raise ValueError("VEL mode is disabled")
    if {source, target} == {"mit", "vel"}:
        raise ValueError("direct MIT and VEL transition is forbidden")


def validate_feedback(
    sample: FeedbackSample,
    config: ModeTransitionConfig,
    *,
    max_velocity_rad_s: float | None = None,
) -> None:
    if not np.all(np.isfinite(sample.positions)):
        raise ValueError("feedback positions are not finite")
    if not np.all(np.isfinite(sample.velocities)):
        raise ValueError("feedback velocities are not finite")
    if float(sample.age_sec) > float(config.feedback_timeout_sec):
        raise ValueError("feedback is stale")
    if max_velocity_rad_s is not None and sample.velocities.size:
        maximum = float(np.max(np.abs(sample.velocities)))
        if maximum > float(max_velocity_rad_s):
            raise ValueError(
                f"feedback velocity {maximum:.4f} rad/s exceeds "
                f"{float(max_velocity_rad_s):.4f} rad/s"
            )
