from __future__ import annotations

import math

import pytest

from rebotarmcontroller.gripper_coordinates import DEFAULT_GRIPPER_COORDINATES


def test_default_gripper_coordinate_model_preserves_tuned_limits() -> None:
    coordinates = DEFAULT_GRIPPER_COORDINATES

    assert coordinates.maximum_distance_m == 0.09
    assert coordinates.maximum_command_opening_m == 0.085
    assert coordinates.open_angle_rad == -5.0
    assert coordinates.open_soft_limit_rad == -4.9
    assert coordinates.closed_feedback_tolerance_rad == pytest.approx(
        0.001 * 5.0 / 0.09
    )


def test_gripper_command_opening_does_not_expand_to_physical_width() -> None:
    coordinates = DEFAULT_GRIPPER_COORDINATES

    assert coordinates.validate_position_request(0.085, 0.0) == (0.085, 0.0)
    with pytest.raises(ValueError, match=r"\[0.0, 0.085\] m"):
        coordinates.validate_position_request(0.085001, 0.0)
    with pytest.raises(ValueError, match="must be finite"):
        coordinates.validate_position_request(float("nan"), 0.0)


def test_gripper_opening_and_angle_conversion_preserves_soft_limit() -> None:
    coordinates = DEFAULT_GRIPPER_COORDINATES

    assert coordinates.opening_to_angle(0.0) == 0.0
    assert coordinates.opening_to_angle(0.085) == pytest.approx(-4.7222222222)
    assert coordinates.opening_to_angle(0.09) == -4.9
    assert coordinates.angle_to_opening(-5.0) == pytest.approx(0.09)


def test_closed_end_tolerance_is_feedback_only() -> None:
    coordinates = DEFAULT_GRIPPER_COORDINATES
    inside = coordinates.closed_feedback_tolerance_rad
    outside = inside + 1.0e-9

    assert coordinates.accepts_feedback_angle(inside)
    assert coordinates.angle_to_opening(inside) == 0.0
    assert not coordinates.accepts_feedback_angle(outside)
    assert math.isnan(coordinates.angle_to_opening(outside))
