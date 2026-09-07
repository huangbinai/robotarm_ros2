from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .gripper_motion_policy import GripperMotionSnapshot, GripperTickDecision


@dataclass
class GripperRuntimeState:
    """Mutable gripper command and feedback state guarded by the hardware lock."""

    target_angle: float = 0.0
    goal_angle: float = 0.0
    target_effort: float = 0.4
    close_force: float = 0.4
    hold_force: float = 0.4
    hold_angle: float = 0.0
    hold_deadline: float | None = None
    hold_release_reason: str | None = None
    mode: str = "idle"
    active: bool = False
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    command_error: str | None = None
    position_result: str = "idle"
    target_timeout_sec: float = 0.0
    target_deadline: float | None = None
    last_tick: float | None = None
    neutral_pending: tuple[float, str, bool] | None = None

    def update_feedback(self, position: float, velocity: float, torque: float) -> None:
        self.position = float(position)
        self.velocity = float(velocity)
        self.torque = float(torque)

    def start_position(
        self,
        *,
        start_angle: float,
        goal_angle: float,
        target_effort: float,
        now: float,
        timeout_sec: float,
    ) -> None:
        self.target_angle = float(start_angle)
        self.goal_angle = float(goal_angle)
        self.target_effort = float(target_effort)
        self.mode = "position"
        self.active = True
        self.position_result = "active"
        self.command_error = None
        self.last_tick = float(now)
        self.target_timeout_sec = float(timeout_sec)
        self.target_deadline = float(now) + float(timeout_sec)
        self.neutral_pending = None

    def start_grasp(self, *, close_force: float, hold_force: float) -> None:
        self.close_force = float(close_force)
        self.hold_force = float(hold_force)
        self.hold_deadline = None
        self.hold_release_reason = None
        self.command_error = None
        self.neutral_pending = None
        self.mode = "grasp_closing"
        self.active = True

    def start_hold(self, *, angle: float, force: float, deadline: float) -> None:
        self.hold_angle = float(angle)
        self.hold_force = float(force)
        self.hold_deadline = float(deadline)
        self.mode = "grasp_holding"
        self.active = True

    def request_stop(self, reason: str) -> None:
        message = str(reason)
        self.command_error = message
        self.position_result = "failed"
        self.hold_deadline = None
        self.neutral_pending = (float(self.position), message, False)
        self.mode = "neutral_pending"
        self.active = True

    def can_cancel_position(self) -> bool:
        return bool(self.active and self.mode == "position")

    def can_release_grasp(self) -> bool:
        return bool(self.active and self.mode in ("grasp_closing", "grasp_holding"))

    def set_idle(self) -> None:
        self.active = False
        self.mode = "idle"

    def apply_tick_decision(self, decision: GripperTickDecision, *, now: float) -> None:
        if decision.next_target_rad is not None:
            self.target_angle = float(decision.next_target_rad)
            self.last_tick = float(now)
        if decision.action == "queue_neutral":
            self.neutral_pending = (
                float(decision.position_rad),
                str(decision.reason),
                bool(decision.marks_success),
            )
            self.mode = "neutral_pending"
            return
        if decision.reason == "grasp release: hold timeout":
            self.hold_release_reason = "hold timeout"

    def complete_neutral(self, decision: GripperTickDecision) -> None:
        self.neutral_pending = None
        self.set_idle()
        if decision.marks_success:
            self.position_result = "succeeded"
            self.command_error = None

    def snapshot(self) -> GripperMotionSnapshot:
        return GripperMotionSnapshot(
            mode=self.mode,
            active=self.active,
            position_rad=self.position,
            target_rad=self.target_angle,
            goal_rad=self.goal_angle,
            target_effort_nm=self.target_effort,
            close_force_nm=self.close_force,
            hold_force_nm=self.hold_force,
            hold_angle_rad=self.hold_angle,
            hold_deadline=self.hold_deadline,
            target_deadline=self.target_deadline,
            last_tick=self.last_tick,
            neutral_pending=self.neutral_pending,
        )


class GripperRuntimeField:
    """Compatibility descriptor for legacy private HardwareManager fields."""

    def __init__(self, state_field: str) -> None:
        self._state_field = state_field

    @staticmethod
    def _state(instance: Any) -> GripperRuntimeState:
        state = instance.__dict__.get("_gripper_state")
        if state is None:
            state = GripperRuntimeState()
            instance.__dict__["_gripper_state"] = state
        return state

    def __get__(self, instance: Any, owner: type | None = None) -> Any:
        if instance is None:
            return self
        return getattr(self._state(instance), self._state_field)

    def __set__(self, instance: Any, value: Any) -> None:
        setattr(self._state(instance), self._state_field, value)
