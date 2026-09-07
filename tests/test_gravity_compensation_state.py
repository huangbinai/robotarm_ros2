from __future__ import annotations

import numpy as np

from rebotarmcontroller.gravity_compensation_state import GravityCompensationState


def test_start_and_finish_own_all_gravity_runtime_values() -> None:
    source = np.array([1.0, 2.0])
    state = GravityCompensationState()

    state.start(source)
    source[:] = 0.0

    assert state.active is True
    np.testing.assert_allclose(state.target, [1.0, 2.0])
    np.testing.assert_allclose(state.last_position, [1.0, 2.0])
    np.testing.assert_allclose(state.integral, [0.0, 0.0])
    assert state.lock_counter == 0

    state.finish()
    assert state.active is False
    assert state.target is None
    assert state.last_position is None
    assert state.integral is None
    assert state.lock_counter == 0


def test_accumulate_error_clips_and_motion_retargets_with_integral_decay() -> None:
    state = GravityCompensationState()
    state.start(np.array([0.0, 0.0]))

    integral = state.accumulate_error(
        np.array([2.0, -2.0]),
        template=np.zeros(2),
    )
    np.testing.assert_allclose(integral, [0.5, -0.5])

    state.observe_motion(np.array([0.2, -0.1]), moving=True)
    np.testing.assert_allclose(state.target, [0.2, -0.1])
    np.testing.assert_allclose(state.integral, [0.45, -0.45])
    assert state.lock_counter == 0

    state.observe_motion(np.array([0.2, -0.1]), moving=False)
    assert state.lock_counter == 1


def test_target_and_remembered_position_are_returned_as_copies() -> None:
    state = GravityCompensationState()
    state.start(np.array([0.1, 0.2]))

    target = state.target_copy()
    remembered = state.remember_position(np.array([0.3, 0.4]))
    target[:] = 9.0
    remembered[:] = 8.0

    np.testing.assert_allclose(state.target, [0.1, 0.2])
    np.testing.assert_allclose(state.last_position, [0.3, 0.4])
