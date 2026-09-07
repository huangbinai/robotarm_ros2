from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from rebotarm_vision.visual_grasp_executor_node import VisualGraspExecutorNode
from rebotarm_vision.visual_grasp_execution_state import VisualGraspExecutionState


def test_run_and_attempt_lifecycle_is_held_in_one_state_object() -> None:
    state = VisualGraspExecutionState()
    plan = SimpleNamespace(candidate="candidate")

    assert state.can_start_run() is True
    assert state.reserve_run() is True
    assert state.can_start_run() is False
    assert state.reserve_run() is False

    state.begin_run("start pose")
    state.begin_attempt(attempt_index=1, candidate_index=4, plan=plan)

    assert state.running is True
    assert state.run_counter == 1
    assert state.current_run_id == 1
    assert state.current_attempt_index == 1
    assert state.current_candidate_index == 4
    assert state.current_attempt_plan is not plan
    assert state.current_attempt_plan.candidate == "candidate"
    assert state.failure_recovery_start_pose == "start pose"

    state.clear_run_context()
    assert state.mark_stopped() is True
    assert state.current_attempt_plan is None
    assert state.failure_recovery_start_pose is None
    assert state.running is False


def test_attempt_reset_preserves_opening_baseline_and_clears_contact() -> None:
    state = VisualGraspExecutionState(
        last_gripper_reached_position=0.08,
        last_grasp_contact_detected=True,
        last_grasp_closure_distance_m=0.03,
        retry_retreat_stage="old retreat",
    )

    state.begin_attempt(attempt_index=2, candidate_index=7, plan=object())

    assert state.last_gripper_reached_position == pytest.approx(0.08)
    assert state.last_grasp_contact_detected is False
    assert state.last_grasp_closure_distance_m == pytest.approx(0.0)
    assert state.retry_retreat_stage is None


def test_gripper_results_update_contact_and_closure_diagnostics() -> None:
    state = VisualGraspExecutionState()
    state.record_open_position(0.08)

    state.record_grasp_result(contact_detected=True, reached_position=0.03)

    assert state.last_grasp_contact_detected is True
    assert state.last_grasp_closure_distance_m == pytest.approx(0.05)


def test_diagnostic_prefix_uses_current_run_attempt_and_candidate() -> None:
    state = VisualGraspExecutionState(
        current_run_id=3,
        current_attempt_index=2,
        current_candidate_index=5,
    )

    assert state.diagnostic_prefix("lift") == (
        "[visual_grasp][run=3][attempt=2][candidate=5][stage=lift]"
    )


def test_candidate_lookup_failure_does_not_leave_executor_running() -> None:
    node = object.__new__(VisualGraspExecutorNode)
    node._execution_lock = threading.Lock()
    node._execution_state = VisualGraspExecutionState()
    node._candidate_plans_for_attempts = lambda: (_ for _ in ()).throw(
        RuntimeError("candidate lookup failed")
    )

    with pytest.raises(RuntimeError, match="candidate lookup failed"):
        VisualGraspExecutorNode._execute_visual_grasp(
            node,
            None,
            SimpleNamespace(success=None, message=""),
        )

    assert node._execution_state.running is False
