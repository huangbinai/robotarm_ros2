from __future__ import annotations

import pytest

from rebotarmcontroller.gripper_motion_policy import (
    GripperMotionPolicyConfig,
    GripperMotionSnapshot,
    decide_gripper_tick,
)


CONFIG = GripperMotionPolicyConfig(
    arrive_tolerance_rad=0.12,
    position_max_speed_rad_s=0.5,
    move_kp=5.0,
    move_kd=1.0,
    grasp_close_kp=0.0,
    grasp_close_kd=0.5,
    grasp_hold_kp=5.0,
    grasp_hold_kd=1.0,
    default_torque_limit_nm=1.5,
)


def _snapshot(**overrides) -> GripperMotionSnapshot:
    values = {
        "mode": "idle",
        "active": False,
        "position_rad": -2.0,
        "target_rad": -2.0,
        "goal_rad": -4.0,
        "target_effort_nm": 0.4,
        "close_force_nm": 0.5,
        "hold_force_nm": 0.6,
        "hold_angle_rad": -2.5,
        "hold_deadline": None,
        "target_deadline": None,
        "last_tick": None,
        "neutral_pending": None,
    }
    values.update(overrides)
    return GripperMotionSnapshot(**values)


def test_neutral_pending_is_completed_without_requiring_feedback() -> None:
    decision = decide_gripper_tick(
        _snapshot(
            active=True,
            mode="neutral_pending",
            neutral_pending=(-2.2, "target reached", True),
        ),
        now=10.0,
        config=CONFIG,
    )

    assert decision.action == "neutral"
    assert decision.position_rad == pytest.approx(-2.2)
    assert decision.torque_limit_nm == pytest.approx(0.05)
    assert decision.marks_success is True
    assert decision.requires_feedback is False


def test_grasp_close_and_hold_commands_keep_configured_gains() -> None:
    closing = decide_gripper_tick(
        _snapshot(active=True, mode="grasp_closing"),
        now=10.0,
        config=CONFIG,
    )
    holding = decide_gripper_tick(
        _snapshot(active=True, mode="grasp_holding"),
        now=10.0,
        config=CONFIG,
    )

    assert (
        closing.action,
        closing.position_rad,
        closing.kp,
        closing.kd,
        closing.torque_ff_nm,
        closing.torque_limit_nm,
    ) == ("mit", 0.0, 0.0, 0.5, 0.5, 1.5)
    assert (
        holding.action,
        holding.position_rad,
        holding.kp,
        holding.kd,
        holding.torque_ff_nm,
        holding.torque_limit_nm,
    ) == ("mit", -2.5, 5.0, 1.0, 0.6, 1.5)


def test_expired_hold_requests_bounded_release() -> None:
    decision = decide_gripper_tick(
        _snapshot(active=True, mode="grasp_holding", hold_deadline=9.0),
        now=10.0,
        config=CONFIG,
    )

    assert decision.action == "cancel"
    assert decision.reason == "grasp release: hold timeout"


def test_position_target_arrival_queues_zero_torque_release() -> None:
    decision = decide_gripper_tick(
        _snapshot(
            active=True,
            mode="position",
            position_rad=-3.95,
            goal_rad=-4.0,
        ),
        now=10.0,
        config=CONFIG,
    )

    assert decision.action == "queue_neutral"
    assert decision.position_rad == pytest.approx(-3.95)
    assert decision.marks_success is True


def test_position_target_is_rate_limited_before_mit_command() -> None:
    decision = decide_gripper_tick(
        _snapshot(
            active=True,
            mode="position",
            position_rad=-2.0,
            target_rad=-2.0,
            goal_rad=-4.0,
            last_tick=8.0,
            target_deadline=20.0,
        ),
        now=10.0,
        config=CONFIG,
    )

    assert decision.action == "mit"
    assert decision.position_rad == pytest.approx(-3.0)
    assert decision.next_target_rad == pytest.approx(-3.0)
    assert decision.kp == pytest.approx(5.0)
    assert decision.kd == pytest.approx(1.0)
    assert decision.torque_ff_nm == pytest.approx(0.0)
    assert decision.torque_limit_nm == pytest.approx(0.4)


def test_position_timeout_preserves_last_rate_limited_target() -> None:
    decision = decide_gripper_tick(
        _snapshot(
            active=True,
            mode="position",
            target_rad=-2.0,
            goal_rad=-4.0,
            last_tick=8.0,
            target_deadline=9.0,
        ),
        now=10.0,
        config=CONFIG,
    )

    assert decision.action == "cancel"
    assert decision.next_target_rad == pytest.approx(-3.0)
    assert decision.reason == "gripper dynamic position timeout"


def test_nonfinite_tick_time_is_rejected() -> None:
    with pytest.raises(ValueError, match="finite"):
        decide_gripper_tick(_snapshot(), now=float("nan"), config=CONFIG)
