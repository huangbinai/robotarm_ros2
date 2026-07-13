import numpy as np
import pytest


def test_smoothstep_has_zero_and_one_endpoints():
    from rebotarmcontroller.mode_transition_policy import smoothstep

    assert smoothstep(-1.0) == 0.0
    assert smoothstep(0.0) == 0.0
    assert smoothstep(1.0) == 1.0
    assert smoothstep(2.0) == 1.0


def test_smoothstep_is_monotonic():
    from rebotarmcontroller.mode_transition_policy import smoothstep

    values = [smoothstep(step / 100.0) for step in range(101)]

    assert values == sorted(values)


def test_enter_blend_moves_tau_from_zero_to_gravity():
    from rebotarmcontroller.mode_transition_policy import blend_enter_tau

    gravity = np.array([1.0, -2.0, 3.0])

    np.testing.assert_allclose(blend_enter_tau(gravity, 0.0), np.zeros(3))
    np.testing.assert_allclose(blend_enter_tau(gravity, 1.0), gravity)


def test_exit_blend_moves_tau_from_gravity_to_zero():
    from rebotarmcontroller.mode_transition_policy import blend_exit_tau

    gravity = np.array([1.0, -2.0, 3.0])

    np.testing.assert_allclose(blend_exit_tau(gravity, 0.0), gravity)
    np.testing.assert_allclose(blend_exit_tau(gravity, 1.0), np.zeros(3))


def test_gain_blend_matches_exact_endpoints():
    from rebotarmcontroller.mode_transition_policy import blend_scalar

    assert blend_scalar(12.0, 7.0, 0.0) == 12.0
    assert blend_scalar(12.0, 7.0, 1.0) == 7.0


def test_velocity_mode_is_rejected_by_default():
    from rebotarmcontroller.mode_transition_policy import (
        ModeTransitionConfig,
        validate_mode_transition,
    )

    config = ModeTransitionConfig()

    with pytest.raises(ValueError, match="VEL mode is disabled"):
        validate_mode_transition("pos_vel", "vel", config)


def test_mit_and_vel_cannot_switch_directly():
    from rebotarmcontroller.mode_transition_policy import (
        ModeTransitionConfig,
        validate_mode_transition,
    )

    config = ModeTransitionConfig(allow_velocity_mode=True)

    with pytest.raises(ValueError, match="direct MIT.*VEL"):
        validate_mode_transition("mit", "vel", config)
    with pytest.raises(ValueError, match="direct MIT.*VEL"):
        validate_mode_transition("vel", "mit", config)


def test_feedback_gate_rejects_stale_or_fast_feedback():
    from rebotarmcontroller.mode_transition_policy import (
        FeedbackSample,
        ModeTransitionConfig,
        validate_feedback,
    )

    config = ModeTransitionConfig(
        feedback_timeout_sec=0.1,
        enter_max_velocity_rad_s=0.05,
    )

    with pytest.raises(ValueError, match="stale"):
        validate_feedback(
            FeedbackSample(
                positions=np.zeros(2),
                velocities=np.zeros(2),
                age_sec=0.2,
            ),
            config,
            max_velocity_rad_s=config.enter_max_velocity_rad_s,
        )

    with pytest.raises(ValueError, match="velocity"):
        validate_feedback(
            FeedbackSample(
                positions=np.zeros(2),
                velocities=np.array([0.01, 0.2]),
                age_sec=0.0,
            ),
            config,
            max_velocity_rad_s=config.enter_max_velocity_rad_s,
        )
