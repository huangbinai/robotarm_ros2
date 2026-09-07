from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class GripperPositionResult:
    success: bool
    reached_position: float


@dataclass(frozen=True)
class GripperGraspResult:
    success: bool
    contact_detected: bool
    contact_position: float
    reached_position: float
    hold_force: float
    message: str


class VisualGripperGateway:
    """Send visual-grasp gripper service requests and normalize responses."""

    def __init__(
        self,
        *,
        position_client: Any,
        grasp_client: Any,
        position_request_factory: Callable[[], Any],
        grasp_request_factory: Callable[[], Any],
        wait_for_future: Callable[[Any, float], bool],
        service_timeout_sec: float,
    ) -> None:
        self._position_client = position_client
        self._grasp_client = grasp_client
        self._position_request_factory = position_request_factory
        self._grasp_request_factory = grasp_request_factory
        self._wait_for_future = wait_for_future
        self._service_timeout_sec = float(service_timeout_sec)

    def set_position(
        self,
        *,
        position_m: float,
        max_effort: float,
    ) -> tuple[GripperPositionResult | None, str]:
        if not self._position_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            return None, "gripper service unavailable"
        request = self._position_request_factory()
        request.position = float(position_m)
        request.max_effort = float(max_effort)
        future = self._position_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec):
            return None, "gripper service call timed out"
        result = future.result()
        if result is None:
            return None, "gripper service returned no result"
        return (
            GripperPositionResult(
                success=bool(result.success),
                reached_position=float(result.reached_position),
            ),
            "",
        )

    def grasp(
        self,
        *,
        close_force: float,
        hold_force: float,
        close_timeout_sec: float,
        min_close_time_sec: float,
        velocity_threshold: float,
        min_closure_distance_m: float,
    ) -> tuple[GripperGraspResult | None, str]:
        if not self._grasp_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            return None, "gripper grasp service unavailable"
        request = self._grasp_request_factory()
        request.close_force = max(float(close_force), 0.0)
        request.hold_force = max(float(hold_force), 0.0)
        request.close_timeout_sec = float(close_timeout_sec)
        request.min_close_time_sec = float(min_close_time_sec)
        request.velocity_threshold = float(velocity_threshold)
        request.min_closure_distance_m = float(min_closure_distance_m)
        future = self._grasp_client.call_async(request)
        if not self._wait_for_future(
            future,
            self._service_timeout_sec + request.close_timeout_sec,
        ):
            return None, "gripper grasp service call timed out"
        result = future.result()
        if result is None:
            return None, "gripper grasp service returned no result"
        return (
            GripperGraspResult(
                success=bool(result.success),
                contact_detected=bool(result.contact_detected),
                contact_position=float(result.contact_position),
                reached_position=float(result.reached_position),
                hold_force=float(result.hold_force),
                message=str(result.message),
            ),
            "",
        )
