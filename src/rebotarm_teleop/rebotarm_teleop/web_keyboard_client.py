from __future__ import annotations

import math
import threading
from typing import Any, Callable

from .teleop_core import WebKeyboardCommandDecision, validate_web_keyboard_command


def keyboard_decision_response(decision: WebKeyboardCommandDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "key": str(decision.key),
        "joint_name": str(decision.joint_name),
        "step_rad": float(decision.step_rad),
        "duration": float(decision.duration),
        "max_joint_speed_rad_s": float(decision.max_joint_speed_rad_s),
    }


def _number_or_default(value: Any, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(number):
        return float(default)
    return number


def _set_duration(duration_msg: Any, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


class WebKeyboardClient:
    """Own web-keyboard settings, validation, and trajectory dispatch."""

    def __init__(
        self,
        *,
        action_client: Any,
        joint_names: tuple[str, ...],
        joint_limits: dict[str, tuple[float, float]],
        joint_velocity_limits: dict[str, float],
        trajectory_factory: Callable[[], Any],
        trajectory_point_factory: Callable[[], Any],
        follow_goal_factory: Callable[[], Any],
        default_step_rad: float,
        default_duration: float,
        default_speed_rad_s: float,
    ) -> None:
        self._action_client = action_client
        self._joint_names = tuple(joint_names)
        self._joint_limits = dict(joint_limits)
        self._joint_velocity_limits = dict(joint_velocity_limits)
        self._trajectory_factory = trajectory_factory
        self._trajectory_point_factory = trajectory_point_factory
        self._follow_goal_factory = follow_goal_factory
        self._lock = threading.Lock()
        self._enabled = False
        self._step_rad = float(default_step_rad)
        self._duration = float(default_duration)
        self._speed_rad_s = float(default_speed_rad_s)

    def enable(
        self,
        payload: dict,
        *,
        min_step_rad: float,
        max_step_rad: float,
        min_duration: float,
        max_duration: float,
        max_speed_rad_s: float,
    ) -> dict:
        with self._lock:
            step = _number_or_default(payload.get("step_rad"), self._step_rad)
            duration = _number_or_default(payload.get("duration"), self._duration)
            speed = _number_or_default(
                payload.get("max_joint_speed_rad_s"),
                self._speed_rad_s,
            )
            self._step_rad = min(max(step, float(min_step_rad)), float(max_step_rad))
            self._duration = min(
                max(duration, float(min_duration)),
                float(max_duration),
            )
            self._speed_rad_s = min(max(speed, 0.05), float(max_speed_rad_s))
            self._enabled = True
            enabled_settings = (
                self._step_rad,
                self._duration,
                self._speed_rad_s,
            )
        return {
            "accepted": True,
            "source": "web_keyboard",
            "state": "ready",
            "message": "web keyboard teleop enabled",
            "step_rad": enabled_settings[0],
            "duration": enabled_settings[1],
            "max_joint_speed_rad_s": enabled_settings[2],
        }

    def disable(self) -> None:
        with self._lock:
            self._enabled = False

    def prepare_command(
        self,
        payload: dict,
        *,
        current_positions: dict[str, float],
        default_step_rad: float,
        min_step_rad: float,
        max_step_rad: float,
        default_duration: float,
        min_duration: float,
        max_duration: float,
        max_speed_rad_s: float,
    ) -> tuple[dict, WebKeyboardCommandDecision]:
        with self._lock:
            enabled = self._enabled
            step_rad = self._step_rad
            duration = self._duration
            speed_rad_s = self._speed_rad_s
        request_payload = dict(payload)
        request_payload.setdefault("step_rad", step_rad)
        request_payload.setdefault("duration", duration)
        request_payload.setdefault("max_joint_speed_rad_s", speed_rad_s)
        decision = validate_web_keyboard_command(
            request_payload,
            enabled=enabled,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            default_step_rad=default_step_rad,
            min_step_rad=min_step_rad,
            max_step_rad=max_step_rad,
            default_duration=default_duration,
            min_duration=min_duration,
            max_duration=max_duration,
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=max_speed_rad_s,
        )
        return request_payload, decision

    def send(self, decision: WebKeyboardCommandDecision) -> dict:
        if not self._action_client.wait_for_server(timeout_sec=0.05):
            return {
                "accepted": False,
                "message": "follow_joint_trajectory action unavailable",
                "goal_future": None,
            }
        trajectory = self._trajectory_factory()
        trajectory.joint_names = list(decision.joint_names)
        point = self._trajectory_point_factory()
        point.positions = [float(value) for value in decision.positions]
        _set_duration(point.time_from_start, decision.duration)
        trajectory.points = [point]
        goal = self._follow_goal_factory()
        goal.trajectory = trajectory
        return {
            "accepted": True,
            "message": decision.message,
            "goal_future": self._action_client.send_goal_async(goal),
            "trajectory": trajectory,
        }
