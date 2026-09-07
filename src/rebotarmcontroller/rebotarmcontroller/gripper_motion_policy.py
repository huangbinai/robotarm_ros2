from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GripperMotionPolicyConfig:
    arrive_tolerance_rad: float
    position_max_speed_rad_s: float
    move_kp: float
    move_kd: float
    grasp_close_kp: float
    grasp_close_kd: float
    grasp_hold_kp: float
    grasp_hold_kd: float
    default_torque_limit_nm: float


@dataclass(frozen=True)
class GripperMotionSnapshot:
    mode: str
    active: bool
    position_rad: float
    target_rad: float
    goal_rad: float
    target_effort_nm: float
    close_force_nm: float
    hold_force_nm: float
    hold_angle_rad: float
    hold_deadline: float | None
    target_deadline: float | None
    last_tick: float | None
    neutral_pending: tuple[float, str, bool] | None


@dataclass(frozen=True)
class GripperTickDecision:
    action: str
    position_rad: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque_ff_nm: float = 0.0
    torque_limit_nm: float = 0.0
    next_target_rad: float | None = None
    reason: str = ""
    marks_success: bool = False

    @property
    def requires_feedback(self) -> bool:
        return self.action not in ("idle", "neutral")


def decide_gripper_tick(
    snapshot: GripperMotionSnapshot,
    *,
    now: float,
    config: GripperMotionPolicyConfig,
) -> GripperTickDecision:
    current_time = float(now)
    if not math.isfinite(current_time):
        raise ValueError("gripper tick time must be finite")

    pending = snapshot.neutral_pending
    if pending is not None:
        angle, reason, marks_success = pending
        return GripperTickDecision(
            action="neutral",
            position_rad=float(angle),
            torque_limit_nm=0.05,
            reason=str(reason),
            marks_success=bool(marks_success),
        )
    if not snapshot.active:
        return GripperTickDecision(action="idle")
    if snapshot.mode == "grasp_closing":
        return GripperTickDecision(
            action="mit",
            position_rad=0.0,
            kp=config.grasp_close_kp,
            kd=config.grasp_close_kd,
            torque_ff_nm=snapshot.close_force_nm,
            torque_limit_nm=config.default_torque_limit_nm,
        )
    if snapshot.mode == "grasp_holding":
        if (
            snapshot.hold_deadline is not None
            and current_time >= snapshot.hold_deadline
        ):
            return GripperTickDecision(
                action="cancel",
                reason="grasp release: hold timeout",
            )
        return GripperTickDecision(
            action="mit",
            position_rad=snapshot.hold_angle_rad,
            kp=config.grasp_hold_kp,
            kd=config.grasp_hold_kd,
            torque_ff_nm=snapshot.hold_force_nm,
            torque_limit_nm=config.default_torque_limit_nm,
        )
    if snapshot.mode != "position":
        return GripperTickDecision(action="idle")
    if abs(snapshot.position_rad - snapshot.goal_rad) < config.arrive_tolerance_rad:
        return GripperTickDecision(
            action="queue_neutral",
            position_rad=snapshot.position_rad,
            reason="position target reached",
            marks_success=True,
        )

    elapsed = max(current_time - (snapshot.last_tick or current_time), 0.0)
    max_step = config.position_max_speed_rad_s * elapsed
    remaining = snapshot.goal_rad - snapshot.target_rad
    target = snapshot.target_rad
    if abs(remaining) <= max_step:
        target = snapshot.goal_rad
    elif max_step > 0.0:
        target += math.copysign(max_step, remaining)
    if (
        snapshot.target_deadline is not None
        and current_time >= snapshot.target_deadline
    ):
        return GripperTickDecision(
            action="cancel",
            next_target_rad=target,
            reason="gripper dynamic position timeout",
        )
    torque_ff = snapshot.target_effort_nm if abs(target) < 1.0e-6 else 0.0
    return GripperTickDecision(
        action="mit",
        position_rad=target,
        kp=config.move_kp,
        kd=config.move_kd,
        torque_ff_nm=torque_ff,
        torque_limit_nm=snapshot.target_effort_nm,
        next_target_rad=target,
    )
