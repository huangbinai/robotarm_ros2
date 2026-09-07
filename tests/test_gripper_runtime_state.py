from __future__ import annotations

import pytest

from rebotarmcontroller.gripper_motion_policy import GripperTickDecision
from rebotarmcontroller.gripper_runtime_state import (
    GripperRuntimeField,
    GripperRuntimeState,
)


def test_position_command_lifecycle_is_held_in_one_state_object() -> None:
    state = GripperRuntimeState(position=-1.0)

    state.start_position(
        start_angle=-1.0,
        goal_angle=-4.0,
        target_effort=0.6,
        now=10.0,
        timeout_sec=7.5,
    )

    snapshot = state.snapshot()
    assert snapshot.mode == "position"
    assert snapshot.active is True
    assert snapshot.target_rad == pytest.approx(-1.0)
    assert snapshot.goal_rad == pytest.approx(-4.0)
    assert snapshot.target_effort_nm == pytest.approx(0.6)
    assert snapshot.target_deadline == pytest.approx(17.5)
    assert state.position_result == "active"


def test_tick_transition_queues_and_completes_neutral_release() -> None:
    state = GripperRuntimeState(active=True, mode="position", position=-3.9)
    queued = GripperTickDecision(
        action="queue_neutral",
        position_rad=-3.9,
        next_target_rad=-4.0,
        reason="gripper position reached",
        marks_success=True,
    )

    state.apply_tick_decision(queued, now=11.0)

    assert state.mode == "neutral_pending"
    assert state.target_angle == pytest.approx(-4.0)
    assert state.last_tick == pytest.approx(11.0)
    assert state.neutral_pending == (-3.9, "gripper position reached", True)

    state.complete_neutral(
        GripperTickDecision(action="neutral", marks_success=True)
    )
    assert state.mode == "idle"
    assert state.active is False
    assert state.position_result == "succeeded"
    assert state.command_error is None


def test_grasp_hold_and_stop_transitions_preserve_release_reason() -> None:
    state = GripperRuntimeState(position=-2.5)
    state.start_grasp(close_force=0.4, hold_force=0.5)
    state.start_hold(angle=-2.5, force=0.5, deadline=20.0)
    state.hold_release_reason = "external release"
    state.request_stop("grasp release: external release")

    assert state.mode == "neutral_pending"
    assert state.active is True
    assert state.hold_deadline is None
    assert state.hold_release_reason == "external release"
    assert state.command_error == "grasp release: external release"
    assert state.neutral_pending == (
        -2.5,
        "grasp release: external release",
        False,
    )


def test_legacy_private_field_descriptor_uses_runtime_state() -> None:
    class Owner:
        _gripper_mode = GripperRuntimeField("mode")

    owner = Owner()
    owner._gripper_mode = "position"

    assert owner._gripper_mode == "position"
    assert owner._gripper_state.mode == "position"
