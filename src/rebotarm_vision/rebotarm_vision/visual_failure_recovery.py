from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from rebotarm_motion.real_failure_recovery import healthy_enabled_hold

from .visual_grasp_sequence import PoseTarget, VisualGraspStage


@dataclass(frozen=True)
class FailureRecoveryOperations:
    confirm_arm_stopped: Callable[[], tuple[bool, str]]
    wait_for_fresh_status: Callable[[], Any | None]
    disable: Callable[[str], tuple[bool, str]]
    execute_pose: Callable[[VisualGraspStage], tuple[bool, str]]
    request_stop: Callable[[bool], None]
    warn: Callable[[str], None]


class VisualFailureRecovery:
    """Coordinates task-failure recovery without owning ROS clients."""

    def __init__(self, operations: FailureRecoveryOperations) -> None:
        self._operations = operations

    def recover(
        self,
        *,
        execution_enabled: bool,
        recovery_mode: str,
        start_pose: PoseTarget | None,
        grasp_contact_detected: bool,
        failed_stage: str,
        failure_message: str,
    ) -> str:
        if not execution_enabled:
            return "not_required_in_plan_only_mode"

        stop_ok, stop_message = self._operations.confirm_arm_stopped()
        status = self._operations.wait_for_fresh_status()
        if status is None:
            return (
                "status_unavailable_leave_state_unchanged"
                if stop_ok
                else "stop_failed_status_unavailable_leave_state_unchanged:"
                f"{stop_message}"
            )
        if not bool(status.enabled) or not bool(status.control_loop_active):
            return "controller_not_in_enabled_hold"
        if not healthy_enabled_hold(status):
            ok, message = self._operations.disable("protective disable")
            return (
                "critical_status_protective_disable"
                if ok
                else f"critical_status_protective_disable_failed:{message}"
            )
        if not stop_ok:
            return (
                "stop_failed_healthy_enabled_hold_requires_operator_recovery:"
                f"{stop_message}"
            )
        if recovery_mode == "hold":
            return "healthy_enabled_hold_requires_operator_recovery"
        if start_pose is None:
            return "start_pose_unavailable_healthy_enabled_hold"

        self._operations.warn(
            "task failure recovery returning to the run start pose: "
            f"stage={failed_stage}, reason={failure_message}"
        )
        returned, return_message = self._operations.execute_pose(
            VisualGraspStage(
                name="failure_return_to_start",
                kind="move",
                pose=start_pose,
            )
        )
        if returned:
            disabled, disable_message = self._operations.disable(
                "disable after failure return"
            )
            return (
                "returned_to_start_then_disabled"
                if disabled
                else f"returned_to_start_disable_failed:{disable_message}"
            )

        self._operations.request_stop(not grasp_contact_detected)
        status = self._operations.wait_for_fresh_status()
        if status is None:
            return (
                "return_failed_status_unavailable_leave_state_unchanged:"
                f"{return_message}"
            )
        if not bool(status.enabled) or not bool(status.control_loop_active):
            return f"return_failed_controller_not_in_enabled_hold:{return_message}"
        if healthy_enabled_hold(status):
            return f"return_failed_healthy_enabled_hold:{return_message}"
        disabled, disable_message = self._operations.disable(
            "protective disable after failed return"
        )
        return (
            f"return_failed_critical_status_protective_disable:{return_message}"
            if disabled
            else "return_failed_critical_status_protective_disable_failed:"
            f"{disable_message}; return={return_message}"
        )
