from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from rebotarmcontroller.feedback_sequence import VerifiedFeedbackSample
from rebotarmcontroller.gripper_coordinates import DEFAULT_GRIPPER_COORDINATES
from rebotarmcontroller.gripper_position import (
    GripperPositionConfig,
    GripperPositionCoordinator,
    GripperPositionProgress,
)


CONFIG = GripperPositionConfig(
    coordinates=DEFAULT_GRIPPER_COORDINATES,
    feedback_stale_timeout_sec=0.15,
    default_effort_nm=0.4,
    torque_cap_nm=1.0,
    maximum_speed_rad_s=0.5,
    timeout_margin_sec=1.5,
    arrival_tolerance_rad=0.12,
)


@dataclass
class _Clock:
    now: float = 10.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += float(duration)


class _Hardware:
    def __init__(self, clock: _Clock) -> None:
        self.clock = clock
        self.sample = VerifiedFeedbackSample(SimpleNamespace(), 1, clock.now)
        self.feedback = (-1.0, 0.0, 0.0, 1)
        self.progress = GripperPositionProgress(
            goal_angle_rad=-4.0,
            position_angle_rad=-1.0,
            deadline=20.0,
            active=True,
            result="active",
            canceled=False,
        )
        self.events = []

    def gripper_feedback_sample(self):
        return self.sample

    def gripper_validated_feedback_values(self, state):
        self.events.append(("validate_feedback", state))
        return self.feedback

    def begin_gripper_position(self, **values) -> None:
        self.events.append(("begin", values))

    def gripper_position_progress(self) -> GripperPositionProgress:
        return self.progress

    def cancel_gripper_position_command(self, reason: str) -> bool:
        self.events.append(("cancel", reason))
        return True


def _coordinator(hardware: _Hardware, clock: _Clock) -> GripperPositionCoordinator:
    return GripperPositionCoordinator(
        hardware,
        CONFIG,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )


def test_position_start_preserves_conversion_effort_and_dynamic_timeout() -> None:
    clock = _Clock()
    hardware = _Hardware(clock)
    coordinator = _coordinator(hardware, clock)

    coordinator.start(0.072, 2.0)

    begin = hardware.events[-1]
    assert begin[0] == "begin"
    values = begin[1]
    assert values["start_angle"] == pytest.approx(-1.0)
    assert values["goal_angle"] == pytest.approx(-4.0)
    assert values["target_effort"] == pytest.approx(1.0)
    assert values["now"] == pytest.approx(10.0)
    assert values["timeout_sec"] == pytest.approx(7.5)


def test_position_start_rejects_stale_or_disabled_feedback() -> None:
    clock = _Clock(now=10.0)
    hardware = _Hardware(clock)
    hardware.sample = VerifiedFeedbackSample(SimpleNamespace(), 1, 0.0)
    coordinator = _coordinator(hardware, clock)

    with pytest.raises(RuntimeError, match="stale before position command"):
        coordinator.start(0.04, 0.4)
    assert hardware.events == []

    hardware.sample = VerifiedFeedbackSample(SimpleNamespace(), 2, 10.0)
    hardware.feedback = (-1.0, 0.0, 0.0, 0)
    with pytest.raises(RuntimeError, match="status_code=0, expected 1"):
        coordinator.start(0.04, 0.4)


def test_position_wait_observes_success_replacement_and_timeout() -> None:
    clock = _Clock()
    hardware = _Hardware(clock)
    coordinator = _coordinator(hardware, clock)

    hardware.progress = GripperPositionProgress(
        goal_angle_rad=-4.0,
        position_angle_rad=-4.0,
        deadline=20.0,
        active=False,
        result="succeeded",
        canceled=False,
    )
    assert coordinator.wait() is True

    snapshots = iter(
        [
            GripperPositionProgress(-4.0, -1.0, 20.0, True, "active", False),
            GripperPositionProgress(-3.0, -1.0, 20.0, True, "active", False),
        ]
    )
    hardware.gripper_position_progress = lambda: next(snapshots)
    assert coordinator.wait() is False

    hardware.gripper_position_progress = lambda: GripperPositionProgress(
        -4.0, -1.0, 10.01, True, "active", False
    )
    assert coordinator.wait() is False
    assert hardware.events[-1] == (
        "cancel",
        "gripper target timeout: goal=-4.000000rad",
    )


@pytest.mark.parametrize(
    ("progress", "expected"),
    (
        (GripperPositionProgress(-4.0, -1.0, 20.0, False, "succeeded", False), True),
        (GripperPositionProgress(-4.0, -4.0, 20.0, False, "failed", False), False),
        (GripperPositionProgress(-4.0, -3.9, 20.0, True, "active", False), True),
        (GripperPositionProgress(-4.0, -3.8, 20.0, True, "active", False), False),
    ),
)
def test_position_reached_target_preserves_runtime_semantics(
    progress: GripperPositionProgress,
    expected: bool,
) -> None:
    clock = _Clock()
    hardware = _Hardware(clock)
    hardware.progress = progress

    assert _coordinator(hardware, clock).reached_target() is expected
