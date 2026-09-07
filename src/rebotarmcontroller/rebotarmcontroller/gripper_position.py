from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Protocol

import numpy as np

from .gripper_coordinates import GripperCoordinateModel


@dataclass(frozen=True)
class GripperPositionConfig:
    coordinates: GripperCoordinateModel
    feedback_stale_timeout_sec: float
    default_effort_nm: float
    torque_cap_nm: float
    maximum_speed_rad_s: float
    timeout_margin_sec: float
    arrival_tolerance_rad: float


@dataclass(frozen=True)
class GripperPositionProgress:
    goal_angle_rad: float
    position_angle_rad: float
    deadline: float | None
    active: bool
    result: str
    canceled: bool


class GripperPositionHardware(Protocol):
    def gripper_feedback_sample(self) -> Any: ...

    def gripper_validated_feedback_values(
        self,
        state: Any,
    ) -> tuple[float, float, float, int]: ...

    def begin_gripper_position(
        self,
        *,
        start_angle: float,
        goal_angle: float,
        target_effort: float,
        now: float,
        timeout_sec: float,
    ) -> None: ...

    def gripper_position_progress(self) -> GripperPositionProgress: ...

    def cancel_gripper_position_command(
        self,
        reason: str = "position command canceled",
    ) -> bool: ...


class GripperPositionCoordinator:
    """Coordinate one position goal without owning motor or feedback access."""

    def __init__(
        self,
        hardware: GripperPositionHardware,
        config: GripperPositionConfig,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._hardware = hardware
        self._config = config
        self._monotonic = monotonic
        self._sleep = sleep

    def start(self, position_m: float, effort_request_nm: float) -> None:
        sample = self._hardware.gripper_feedback_sample()
        if sample.is_stale(
            self._monotonic(),
            self._config.feedback_stale_timeout_sec,
        ):
            raise RuntimeError("gripper feedback is stale before position command")
        start_angle, _velocity, _torque, status = (
            self._hardware.gripper_validated_feedback_values(sample.state)
        )
        if status != 1:
            raise RuntimeError(f"gripper status_code={status}, expected 1")

        target_angle = self._config.coordinates.opening_to_angle(position_m)
        requested_effort = (
            self._config.default_effort_nm
            if effort_request_nm <= 0.0
            else effort_request_nm
        )
        now = self._monotonic()
        dynamic_timeout = (
            abs(target_angle - start_angle) / self._config.maximum_speed_rad_s
            + self._config.timeout_margin_sec
        )
        self._hardware.begin_gripper_position(
            start_angle=start_angle,
            goal_angle=target_angle,
            target_effort=float(
                np.clip(requested_effort, 0.05, self._config.torque_cap_nm)
            ),
            now=now,
            timeout_sec=dynamic_timeout,
        )

    def wait(self, timeout: float | None = None) -> bool:
        initial = self._hardware.gripper_position_progress()
        owned_goal = initial.goal_angle_rad
        deadline = initial.deadline
        if deadline is None:
            return False
        if timeout is not None:
            explicit = self._monotonic() + max(float(timeout), 0.0)
            deadline = min(deadline, explicit)
        while self._monotonic() < deadline:
            progress = self._hardware.gripper_position_progress()
            if progress.goal_angle_rad != owned_goal:
                return False
            if not progress.active:
                return progress.result == "succeeded"
            if progress.canceled:
                return False
            self._sleep(0.02)
        self._hardware.cancel_gripper_position_command(
            f"gripper target timeout: goal={owned_goal:.6f}rad"
        )
        return False

    def reached_target(self) -> bool:
        progress = self._hardware.gripper_position_progress()
        if progress.result == "succeeded":
            return True
        if progress.result == "failed" or not progress.active:
            return False
        return (
            abs(progress.position_angle_rad - progress.goal_angle_rad)
            < self._config.arrival_tolerance_rad
        )
