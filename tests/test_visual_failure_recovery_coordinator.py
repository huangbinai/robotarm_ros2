from __future__ import annotations

from types import SimpleNamespace

from rebotarm_vision.visual_failure_recovery import (
    FailureRecoveryOperations,
    VisualFailureRecovery,
)
from rebotarm_vision.visual_grasp_sequence import PoseTarget


def _status(*, healthy: bool = True, enabled: bool = True):
    return SimpleNamespace(
        enabled=enabled,
        control_loop_active=enabled,
        per_joint_status_code=[1] * 6 if healthy else [1, 1, 0, 1, 1, 1],
        error_codes=[] if healthy else ["FEEDBACK_STALE"],
    )


def _coordinator(
    *,
    statuses,
    stop_result=(True, "stopped"),
    execute_result=(True, "executed"),
    disable_result=(True, "disabled"),
):
    events = []
    status_iter = iter(statuses)

    def wait_for_status():
        events.append(("status",))
        return next(status_iter)

    operations = FailureRecoveryOperations(
        confirm_arm_stopped=lambda: events.append(("stop",)) or stop_result,
        wait_for_fresh_status=wait_for_status,
        disable=lambda label: events.append(("disable", label)) or disable_result,
        execute_pose=lambda stage: events.append(("execute", stage)) or execute_result,
        request_stop=lambda stop_gripper: events.append(
            ("request_stop", stop_gripper)
        ),
        warn=lambda message: events.append(("warn", message)),
    )
    return VisualFailureRecovery(operations), events


def _recover(recovery: VisualFailureRecovery, **overrides) -> str:
    values = {
        "execution_enabled": True,
        "recovery_mode": "hold",
        "start_pose": PoseTarget(
            position=(0.3, 0.0, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
        ),
        "grasp_contact_detected": False,
        "failed_stage": "approach_grasp",
        "failure_message": "planning failed",
    }
    values.update(overrides)
    return recovery.recover(**values)


def test_plan_only_failure_never_calls_hardware_recovery_operations() -> None:
    recovery, events = _coordinator(statuses=[])

    outcome = _recover(recovery, execution_enabled=False)

    assert outcome == "not_required_in_plan_only_mode"
    assert events == []


def test_missing_status_leaves_hardware_state_unchanged() -> None:
    recovery, events = _coordinator(
        statuses=[None],
        stop_result=(False, "trajectory stop unavailable"),
    )

    outcome = _recover(recovery)

    assert outcome == (
        "stop_failed_status_unavailable_leave_state_unchanged:"
        "trajectory stop unavailable"
    )
    assert events == [("stop",), ("status",)]


def test_return_mode_without_start_pose_keeps_healthy_enabled_hold() -> None:
    recovery, events = _coordinator(statuses=[_status()])

    outcome = _recover(
        recovery,
        recovery_mode="return_then_disable",
        start_pose=None,
    )

    assert outcome == "start_pose_unavailable_healthy_enabled_hold"
    assert events == [("stop",), ("status",)]


def test_failed_return_with_critical_status_requests_protective_disable() -> None:
    recovery, events = _coordinator(
        statuses=[_status(), _status(healthy=False)],
        execute_result=(False, "planning failed"),
    )

    outcome = _recover(recovery, recovery_mode="return_then_disable")

    assert outcome == (
        "return_failed_critical_status_protective_disable:planning failed"
    )
    assert ("request_stop", True) in events
    assert events[-1] == (
        "disable",
        "protective disable after failed return",
    )
