from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .visual_grasp_sequence import PoseTarget, VisualGraspStage


class VisualMotionGateway:
    """Build and send visual-grasp pose execution service requests."""

    def __init__(
        self,
        *,
        client: Any,
        request_factory: Callable[[], Any],
        target_message_factory: Callable[[PoseTarget, str], Any],
        wait_for_future: Callable[[Any, float], bool],
        target_frame: str,
        service_timeout_sec: float,
        motion_result_timeout_sec: float,
        acceleration_scaling: Callable[[], float],
    ) -> None:
        self._client = client
        self._request_factory = request_factory
        self._target_message_factory = target_message_factory
        self._wait_for_future = wait_for_future
        self._target_frame = str(target_frame)
        self._service_timeout_sec = float(service_timeout_sec)
        self._motion_result_timeout_sec = float(motion_result_timeout_sec)
        self._acceleration_scaling = acceleration_scaling

    def service_available(self) -> bool:
        return bool(
            self._client.wait_for_service(timeout_sec=self._service_timeout_sec)
        )

    def send(
        self,
        stage: VisualGraspStage,
        *,
        execute: bool,
        velocity_scaling: float,
    ) -> tuple[bool, str]:
        if stage.pose is None:
            return False, "missing move pose"
        request = self._request_factory()
        request.target_pose = self._target_message_factory(
            stage.pose,
            self._target_frame,
        )
        request.velocity_scaling = float(velocity_scaling)
        request.acceleration_scaling = float(self._acceleration_scaling())
        request.timeout_sec = self._motion_result_timeout_sec
        request.execute = bool(execute)
        future = self._client.call_async(request)
        if not self._wait_for_future(
            future,
            self._service_timeout_sec + self._motion_result_timeout_sec,
        ):
            return False, "motion execution service call timed out"
        result = future.result()
        if result is None:
            return False, "motion execution returned no result"
        return bool(result.success), f"{result.stage}: {result.message}"
