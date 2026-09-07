from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Callable, Protocol

import numpy as np

from .gripper_safety import is_gripper_contact_sample


GripperGraspResult = tuple[bool, bool, float, float, float, str]


class GripperGraspHardware(Protocol):
    def gripper_feedback_sample(self) -> Any: ...

    def gripper_position_m(self) -> float: ...

    def gripper_feedback_motion(self) -> tuple[float, float]: ...

    def gripper_command_canceled(self) -> bool: ...

    def begin_gripper_grasp(self, close_force: float, hold_force: float) -> None: ...

    def begin_gripper_hold(self, hold_force: float, deadline: float) -> None: ...

    def stop_gripper_motion(self, reason: str = "gripper command canceled") -> None: ...


@dataclass(frozen=True)
class GripperGraspConfig:
    feedback_stale_timeout_sec: float
    default_hold_timeout_sec: float
    maximum_hold_timeout_sec: float
    maximum_close_force_nm: float
    maximum_torque_nm: float
    empty_close_threshold_m: float
    contact_torque_min_nm: float
    contact_stable_samples: int


@dataclass(frozen=True)
class GripperGraspRequest:
    close_effort_nm: float
    hold_effort_nm: float
    close_timeout_sec: float
    minimum_close_time_sec: float
    velocity_threshold_rad_s: float
    minimum_closure_m: float
    hold_timeout_sec: float


def normalize_gripper_grasp_request(
    *,
    close_force: float,
    hold_force: float,
    close_timeout_sec: float,
    min_close_time_sec: float,
    velocity_threshold: float,
    min_closure_distance_m: float,
    hold_timeout_sec: float | None,
    config: GripperGraspConfig,
) -> GripperGraspRequest:
    close_effort = float(np.clip(close_force, 0.05, config.maximum_close_force_nm))
    hold_effort = float(np.clip(hold_force, 0.05, config.maximum_torque_nm))
    numeric_inputs = (
        close_force,
        hold_force,
        close_timeout_sec,
        min_close_time_sec,
        velocity_threshold,
        min_closure_distance_m,
    )
    if any(not math.isfinite(float(value)) for value in numeric_inputs):
        raise ValueError("gripper grasp parameters must be finite")
    requested_hold = (
        config.default_hold_timeout_sec
        if hold_timeout_sec is None
        else float(hold_timeout_sec)
    )
    if not math.isfinite(requested_hold):
        raise ValueError("gripper hold timeout must be finite")
    return GripperGraspRequest(
        close_effort_nm=close_effort,
        hold_effort_nm=hold_effort,
        close_timeout_sec=max(float(close_timeout_sec), 0.1),
        minimum_close_time_sec=max(float(min_close_time_sec), 0.0),
        velocity_threshold_rad_s=max(float(velocity_threshold), 0.0),
        minimum_closure_m=max(float(min_closure_distance_m), 0.0),
        hold_timeout_sec=float(
            np.clip(requested_hold, 0.1, config.maximum_hold_timeout_sec)
        ),
    )


class GripperGraspCoordinator:
    """Run the blocking close/contact/hold workflow over a hardware port."""

    def __init__(
        self,
        hardware: GripperGraspHardware,
        config: GripperGraspConfig,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._hardware = hardware
        self._config = config
        self._monotonic = monotonic
        self._sleep = sleep

    def execute(
        self,
        initial_sample: Any,
        *,
        close_force: float,
        hold_force: float,
        close_timeout_sec: float,
        min_close_time_sec: float,
        velocity_threshold: float,
        min_closure_distance_m: float,
        hold_timeout_sec: float | None,
    ) -> GripperGraspResult:
        if initial_sample.is_stale(
            self._monotonic(),
            self._config.feedback_stale_timeout_sec,
        ):
            raise RuntimeError("gripper feedback is stale before grasp command")
        request = normalize_gripper_grasp_request(
            close_force=close_force,
            hold_force=hold_force,
            close_timeout_sec=close_timeout_sec,
            min_close_time_sec=min_close_time_sec,
            velocity_threshold=velocity_threshold,
            min_closure_distance_m=min_closure_distance_m,
            hold_timeout_sec=hold_timeout_sec,
            config=self._config,
        )
        start_position_m = self._hardware.gripper_position_m()
        started = self._monotonic()
        stable_contact_samples = 0
        last_sequence = initial_sample.sequence
        self._hardware.begin_gripper_grasp(
            request.close_effort_nm,
            request.hold_effort_nm,
        )

        while self._monotonic() - started < request.close_timeout_sec:
            if self._hardware.gripper_command_canceled():
                self._hardware.stop_gripper_motion()
                return (
                    False,
                    False,
                    0.0,
                    self._hardware.gripper_position_m(),
                    request.hold_effort_nm,
                    "grasp canceled",
                )
            elapsed = self._monotonic() - started
            try:
                sample = self._hardware.gripper_feedback_sample()
            except Exception as exc:
                self._hardware.stop_gripper_motion(str(exc))
                return (
                    False,
                    False,
                    0.0,
                    float("nan"),
                    request.hold_effort_nm,
                    str(exc),
                )
            if sample.is_stale(
                self._monotonic(),
                self._config.feedback_stale_timeout_sec,
            ):
                reason = "gripper feedback stale during grasp"
                self._hardware.stop_gripper_motion(reason)
                return (
                    False,
                    False,
                    0.0,
                    self._hardware.gripper_position_m(),
                    request.hold_effort_nm,
                    reason,
                )
            if sample.sequence == last_sequence:
                self._sleep(0.005)
                continue
            last_sequence = sample.sequence
            reached_position_m = self._hardware.gripper_position_m()
            closure_m = max(start_position_m - reached_position_m, 0.0)
            velocity_rad_s, torque_nm = self._hardware.gripper_feedback_motion()
            contact_sample = (
                elapsed >= request.minimum_close_time_sec
                and is_gripper_contact_sample(
                    opening_m=reached_position_m,
                    closure_m=closure_m,
                    velocity_rad_s=velocity_rad_s,
                    torque_nm=torque_nm,
                    min_opening_m=self._config.empty_close_threshold_m,
                    min_closure_m=request.minimum_closure_m,
                    max_velocity_rad_s=request.velocity_threshold_rad_s,
                    min_torque_nm=self._config.contact_torque_min_nm,
                )
            )
            stable_contact_samples = (
                stable_contact_samples + 1 if contact_sample else 0
            )
            if stable_contact_samples >= self._config.contact_stable_samples:
                self._hardware.begin_gripper_hold(
                    request.hold_effort_nm,
                    self._monotonic() + request.hold_timeout_sec,
                )
                contact_position_m = self._hardware.gripper_position_m()
                return (
                    True,
                    True,
                    contact_position_m,
                    contact_position_m,
                    request.hold_effort_nm,
                    "contact detected; hold bounded to "
                    f"{request.hold_timeout_sec:g} s",
                )
            self._sleep(0.01)

        reason = "grasp close timeout before contact"
        self._hardware.stop_gripper_motion(reason)
        return (
            False,
            False,
            0.0,
            self._hardware.gripper_position_m(),
            request.hold_effort_nm,
            reason,
        )
