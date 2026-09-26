from types import SimpleNamespace

import pytest

from rebotarm_motion.mujoco_bottle_trial import (
    bottle_target,
    classify_grasp,
    safe_to_run_trial,
)


def _bottle(x=0.28, y=0.0, z=0.0):
    return SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z))


def test_known_bottle_targets_are_staged_above_table():
    bottle = _bottle()
    assert bottle_target(bottle, stage="pregrasp") == pytest.approx((0.295, 0.0, 0.24))
    assert bottle_target(bottle, stage="grasp") == pytest.approx((0.295, 0.0, 0.14))
    assert safe_to_run_trial(bottle)


def test_trial_rejects_bottle_outside_validated_workbench():
    with pytest.raises(ValueError, match="canonical tabletop"):
        bottle_target(_bottle(x=0.34), stage="grasp")
    assert not safe_to_run_trial(_bottle(x=0.30))
    with pytest.raises(ValueError, match="finite"):
        bottle_target(_bottle(z=float("nan")), stage="pregrasp")


def test_contact_is_not_reported_as_lift_success():
    assert classify_grasp(True, 0.0) == "bilateral_contact_without_lift"
    assert classify_grasp(True, 0.02) == "contact_and_lift"
    assert classify_grasp(False, 0.1) == "no_bilateral_contact"
