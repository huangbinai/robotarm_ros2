from __future__ import annotations

from types import SimpleNamespace

from rebotarm_vision.visual_grasp_executor_node import VisualGraspExecutorNode
from rebotarm_vision.visual_grasp_sequence import PoseTarget


def _status(*, healthy: bool = True, enabled: bool = True):
    return SimpleNamespace(
        enabled=enabled,
        control_loop_active=enabled,
        per_joint_status_code=[1] * 6 if healthy else [1, 1, 0, 1, 1, 1],
        error_codes=[] if healthy else ["FEEDBACK_STALE"],
    )


def _node(*, mode: str = "hold") -> VisualGraspExecutorNode:
    node = object.__new__(VisualGraspExecutorNode)
    node._execution_mode = "real"
    node._failure_recovery_mode = mode
    node._failure_recovery_status_timeout_sec = 1.0
    node._failure_recovery_return_velocity_scaling = 0.04
    node._failure_recovery_start_pose = PoseTarget(
        position=(0.3, 0.0, 0.2),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    node._last_grasp_contact_detected = False
    node._disable_client = object()
    node._motion_stop_client = object()
    node._trajectory_stop_client = object()
    node._service_timeout_sec = 1.0
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)
    node._confirm_arm_stopped_for_recovery = lambda: (True, "stopped")
    return node


def test_healthy_task_failure_holds_without_disabling():
    node = _node(mode="hold")
    node._wait_for_fresh_arm_status = lambda **_kwargs: _status()
    disable_calls = []
    node._call_trigger_service = lambda *args, **kwargs: disable_calls.append(
        (args, kwargs)
    )

    outcome = VisualGraspExecutorNode._recover_task_failure(node, "approach", "failed")

    assert outcome == "healthy_enabled_hold_requires_operator_recovery"
    assert disable_calls == []


def test_critical_task_failure_requests_protective_disable():
    node = _node(mode="hold")
    node._wait_for_fresh_arm_status = lambda **_kwargs: _status(healthy=False)
    disable_calls = []
    node._call_trigger_service = lambda client, label, timeout: (
        disable_calls.append((client, label, timeout)) or (True, "disabled")
    )

    outcome = VisualGraspExecutorNode._recover_task_failure(node, "approach", "failed")

    assert outcome == "critical_status_protective_disable"
    assert disable_calls == [(node._disable_client, "protective disable", 1.0)]


def test_successful_controlled_return_disables_after_moveit_execution():
    node = _node(mode="return_then_disable")
    node._wait_for_fresh_arm_status = lambda **_kwargs: _status()
    stages = []
    node._call_execute_pose = lambda stage: stages.append(stage) or (True, "executed")
    disable_calls = []
    node._call_trigger_service = lambda client, label, timeout: (
        disable_calls.append((client, label, timeout)) or (True, "disabled")
    )

    outcome = VisualGraspExecutorNode._recover_task_failure(node, "lift", "failed")

    assert outcome == "returned_to_start_then_disabled"
    assert len(stages) == 1
    assert stages[0].name == "failure_return_to_start"
    assert stages[0].pose == node._failure_recovery_start_pose
    assert disable_calls == [
        (node._disable_client, "disable after failure return", 1.0)
    ]


def test_failed_controlled_return_keeps_healthy_controller_enabled():
    node = _node(mode="return_then_disable")
    statuses = iter((_status(), _status()))
    node._wait_for_fresh_arm_status = lambda **_kwargs: next(statuses)
    node._call_execute_pose = lambda _stage: (False, "planning failed")
    stop_calls = []
    node._request_stop = lambda **kwargs: stop_calls.append(kwargs)
    disable_calls = []
    node._call_trigger_service = lambda *args, **kwargs: disable_calls.append(
        (args, kwargs)
    )

    outcome = VisualGraspExecutorNode._recover_task_failure(node, "lift", "failed")

    assert outcome == "return_failed_healthy_enabled_hold:planning failed"
    assert stop_calls == [{"stop_gripper": True}]
    assert disable_calls == []


def test_failed_stop_does_not_start_controlled_return():
    node = _node(mode="return_then_disable")
    node._confirm_arm_stopped_for_recovery = lambda: (
        False,
        "trajectory_stop service unavailable",
    )
    node._wait_for_fresh_arm_status = lambda **_kwargs: _status()
    return_calls = []
    node._call_execute_pose = lambda stage: return_calls.append(stage) or (
        True,
        "executed",
    )
    disable_calls = []
    node._call_trigger_service = lambda *args, **kwargs: disable_calls.append(
        (args, kwargs)
    )

    outcome = VisualGraspExecutorNode._recover_task_failure(node, "lift", "failed")

    assert outcome == (
        "stop_failed_healthy_enabled_hold_requires_operator_recovery:"
        "trajectory_stop service unavailable"
    )
    assert return_calls == []
    assert disable_calls == []


def test_critical_status_disables_even_when_stop_confirmation_failed():
    node = _node(mode="return_then_disable")
    node._confirm_arm_stopped_for_recovery = lambda: (False, "stop failed")
    node._wait_for_fresh_arm_status = lambda **_kwargs: _status(healthy=False)
    node._call_trigger_service = lambda *_args, **_kwargs: (True, "disabled")
    return_calls = []
    node._call_execute_pose = lambda stage: return_calls.append(stage) or (
        True,
        "executed",
    )

    outcome = VisualGraspExecutorNode._recover_task_failure(node, "lift", "failed")

    assert outcome == "critical_status_protective_disable"
    assert return_calls == []


def test_stop_after_confirmed_contact_does_not_release_gripper():
    node = _node()
    node._motion_stop_client = object()
    node._trajectory_stop_client = object()
    node._gripper_stop_client = object()
    calls = []
    node._request_stop_service = lambda client, label: calls.append((client, label))

    VisualGraspExecutorNode._request_stop(node, stop_gripper=False)

    assert calls == [
        (node._motion_stop_client, "motion execution stop"),
        (node._trajectory_stop_client, "trajectory_stop"),
    ]


def test_failure_return_uses_dedicated_low_velocity_scaling():
    node = _node(mode="return_then_disable")

    assert (
        VisualGraspExecutorNode._velocity_scaling_for_stage(
            node, "failure_return_to_start"
        )
        == 0.04
    )
