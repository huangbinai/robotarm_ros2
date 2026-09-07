from __future__ import annotations

from types import SimpleNamespace

import pytest

from rebotarmcontroller.hardware_feedback import build_controller_groups
from rebotarmcontroller.gripper_coordinates import DEFAULT_GRIPPER_COORDINATES
from rebotarmcontroller.hardware_feedback_validation import (
    validate_feedback_state,
    validated_gripper_feedback_values,
)


def test_build_controller_groups_combines_motors_by_controller_identity() -> None:
    controller_a = object()
    controller_b = object()
    joint1_motor = object()
    joint2_motor = object()
    gripper_motor = object()
    arm = SimpleNamespace(
        _ctrl_map={"a": controller_a, "b": controller_b},
        _motor_map={"joint1": joint1_motor, "joint2": joint2_motor},
        _joints=[
            SimpleNamespace(name="joint1", vendor="a"),
            SimpleNamespace(name="joint2", vendor="b"),
        ],
    )

    groups = build_controller_groups(
        arm,
        gripper_motor=gripper_motor,
        gripper_controller=controller_a,
    )

    assert groups == [
        (controller_a, [("joint1", joint1_motor), ("gripper", gripper_motor)]),
        (controller_b, [("joint2", joint2_motor)]),
    ]


def test_build_controller_groups_rejects_missing_feedback_hardware() -> None:
    arm = SimpleNamespace(
        _ctrl_map={"a": object()},
        _motor_map={},
        _joints=[SimpleNamespace(name="joint1", vendor="a")],
    )

    with pytest.raises(RuntimeError, match="joint1 feedback hardware unavailable"):
        build_controller_groups(
            arm,
            gripper_motor=None,
            gripper_controller=None,
        )


def test_feedback_validation_preserves_joint_and_gripper_ranges() -> None:
    validate_feedback_state(
        "joint2",
        SimpleNamespace(pos=0.02, vel=0.0, torq=0.0),
        joint_position_limits_rad={"joint2": (-3.14, 0.02)},
        gripper_coordinates=DEFAULT_GRIPPER_COORDINATES,
    )
    values = validated_gripper_feedback_values(
        SimpleNamespace(pos=-5.0005, vel=0.1, torq=0.2, status_code=1),
        coordinates=DEFAULT_GRIPPER_COORDINATES,
    )

    assert values == (-5.0005, 0.1, 0.2, 1)


@pytest.mark.parametrize(
    ("label", "state", "message"),
    (
        ("joint7", SimpleNamespace(pos=0.0, vel=0.0, torq=0.0), "no feedback limit"),
        ("joint2", SimpleNamespace(pos=0.0201, vel=0.0, torq=0.0), "outside feedback range"),
        ("joint2", SimpleNamespace(pos=0.0, vel=float("nan"), torq=0.0), "non-finite"),
    ),
)
def test_feedback_validation_rejects_invalid_joint_samples(
    label: str,
    state: SimpleNamespace,
    message: str,
) -> None:
    with pytest.raises(RuntimeError, match=message):
        validate_feedback_state(
            label,
            state,
            joint_position_limits_rad={"joint2": (-3.14, 0.02)},
            gripper_coordinates=DEFAULT_GRIPPER_COORDINATES,
        )
