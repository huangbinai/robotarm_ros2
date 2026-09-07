from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from rebotarmcontroller.feedback_sequence import VerifiedFeedbackSample
from rebotarmcontroller.gripper_grasp import (
    GripperGraspConfig,
    GripperGraspCoordinator,
    normalize_gripper_grasp_request,
)


CONFIG = GripperGraspConfig(
    feedback_stale_timeout_sec=0.15,
    default_hold_timeout_sec=30.0,
    maximum_hold_timeout_sec=120.0,
    maximum_close_force_nm=1.0,
    maximum_torque_nm=1.5,
    empty_close_threshold_m=0.003,
    contact_torque_min_nm=0.05,
    contact_stable_samples=3,
)


@dataclass
class _Clock:
    now: float = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += float(duration)


class _Hardware:
    def __init__(self, clock: _Clock) -> None:
        self.clock = clock
        self.opening_m = 0.08
        self.sequence = 1
        self.velocity_rad_s = 0.0
        self.torque_nm = 0.2
        self.canceled = False
        self.events = []

    def gripper_feedback_sample(self):
        self.sequence += 1
        self.opening_m = 0.04
        return VerifiedFeedbackSample(
            SimpleNamespace(),
            self.sequence,
            self.clock.monotonic(),
        )

    def gripper_position_m(self) -> float:
        return self.opening_m

    def gripper_feedback_motion(self) -> tuple[float, float]:
        return self.velocity_rad_s, self.torque_nm

    def gripper_command_canceled(self) -> bool:
        return self.canceled

    def begin_gripper_grasp(self, close_force: float, hold_force: float) -> None:
        self.events.append(("close", close_force, hold_force))

    def begin_gripper_hold(self, hold_force: float, deadline: float) -> None:
        self.events.append(("hold", hold_force, deadline))

    def stop_gripper_motion(self, reason: str = "gripper command canceled") -> None:
        self.events.append(("stop", reason))


def _execute(coordinator, initial_sample, **overrides):
    values = {
        "close_force": 0.4,
        "hold_force": 0.4,
        "close_timeout_sec": 2.0,
        "min_close_time_sec": 0.0,
        "velocity_threshold": 0.04,
        "min_closure_distance_m": 0.006,
        "hold_timeout_sec": None,
    }
    values.update(overrides)
    return coordinator.execute(initial_sample, **values)


def test_gripper_grasp_request_preserves_clamps_and_defaults() -> None:
    request = normalize_gripper_grasp_request(
        close_force=2.0,
        hold_force=2.0,
        close_timeout_sec=0.0,
        min_close_time_sec=-1.0,
        velocity_threshold=-1.0,
        min_closure_distance_m=-1.0,
        hold_timeout_sec=None,
        config=CONFIG,
    )

    assert request.close_effort_nm == 1.0
    assert request.hold_effort_nm == 1.5
    assert request.close_timeout_sec == 0.1
    assert request.minimum_close_time_sec == 0.0
    assert request.velocity_threshold_rad_s == 0.0
    assert request.minimum_closure_m == 0.0
    assert request.hold_timeout_sec == 30.0


@pytest.mark.parametrize(
    ("field", "message"),
    (
        ("close_force", "grasp parameters must be finite"),
        ("hold_force", "grasp parameters must be finite"),
        ("close_timeout_sec", "grasp parameters must be finite"),
        ("hold_timeout_sec", "hold timeout must be finite"),
    ),
)
def test_gripper_grasp_request_rejects_nonfinite_values(
    field: str,
    message: str,
) -> None:
    values = {
        "close_force": 0.4,
        "hold_force": 0.4,
        "close_timeout_sec": 2.0,
        "min_close_time_sec": 0.08,
        "velocity_threshold": 0.04,
        "min_closure_distance_m": 0.006,
        "hold_timeout_sec": None,
        field: float("nan"),
    }
    with pytest.raises(ValueError, match=message):
        normalize_gripper_grasp_request(config=CONFIG, **values)


def test_gripper_grasp_requires_three_fresh_contact_samples_before_hold() -> None:
    clock = _Clock()
    hardware = _Hardware(clock)
    coordinator = GripperGraspCoordinator(
        hardware,
        CONFIG,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    initial = VerifiedFeedbackSample(SimpleNamespace(), 1, 0.0)

    result = _execute(coordinator, initial)

    assert result == (
        True,
        True,
        0.04,
        0.04,
        0.4,
        "contact detected; hold bounded to 30 s",
    )
    assert hardware.events[0] == ("close", 0.4, 0.4)
    assert hardware.events[1][0:2] == ("hold", 0.4)
    assert hardware.sequence == 4


def test_gripper_grasp_cancellation_stops_and_returns_live_opening() -> None:
    clock = _Clock()
    hardware = _Hardware(clock)
    hardware.canceled = True
    coordinator = GripperGraspCoordinator(
        hardware,
        CONFIG,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    initial = VerifiedFeedbackSample(SimpleNamespace(), 1, 0.0)

    result = _execute(coordinator, initial)

    assert result == (False, False, 0.0, 0.08, 0.4, "grasp canceled")
    assert hardware.events[-1] == ("stop", "gripper command canceled")


def test_gripper_grasp_rejects_stale_initial_feedback_before_starting() -> None:
    clock = _Clock(now=1.0)
    hardware = _Hardware(clock)
    coordinator = GripperGraspCoordinator(
        hardware,
        CONFIG,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    stale = VerifiedFeedbackSample(SimpleNamespace(), 1, 0.0)

    with pytest.raises(RuntimeError, match="stale before grasp command"):
        _execute(coordinator, stale)

    assert hardware.events == []


def test_gripper_grasp_timeout_stops_without_claiming_contact() -> None:
    clock = _Clock()
    hardware = _Hardware(clock)
    hardware.torque_nm = 0.0
    coordinator = GripperGraspCoordinator(
        hardware,
        CONFIG,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    initial = VerifiedFeedbackSample(SimpleNamespace(), 1, 0.0)

    result = _execute(coordinator, initial, close_timeout_sec=0.1)

    assert result == (
        False,
        False,
        0.0,
        0.04,
        0.4,
        "grasp close timeout before contact",
    )
    assert hardware.events[-1] == ("stop", "grasp close timeout before contact")
