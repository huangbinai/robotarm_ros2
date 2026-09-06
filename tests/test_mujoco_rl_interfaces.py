from __future__ import annotations

import numpy as np
import pytest

from rebotarm_simulation.gym_adapter import (
    RebotArmGymnasiumEnv,
    gymnasium_available,
)
from rebotarm_simulation.reach_task import evaluate_reach
from rebotarm_simulation.rl_schema import (
    PICK_SCHEMA_V1,
    REACH_SCHEMA_V1,
    get_policy_schema,
)
from rebotarm_simulation.vector_env import SyncHeadlessVectorEnv


def _reach_observation(offset: float = 0.0):
    return {
        "joint_positions": np.full(6, offset, dtype=np.float32),
        "joint_velocities": np.zeros(6, dtype=np.float32),
        "gripper_width": 0.06,
        "ee_position": np.array([0.2 + offset, 0.0, 0.2], dtype=np.float32),
        "target_position": np.array([0.25, 0.0, 0.2], dtype=np.float32),
        "cube_pose": np.array([0.3, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        "max_contact_force": 0.0,
    }


class DummyEnv:
    schema = REACH_SCHEMA_V1

    def __init__(self):
        self.offset = 0.0
        self.closed = False

    def reset(self, *, seed=None, **_kwargs):
        self.offset = float(seed or 0) / 100.0
        return _reach_observation(self.offset), {"seed": seed}

    def step(self, action):
        self.offset += float(np.asarray(action)[0]) * 0.01
        return _reach_observation(self.offset), -self.offset, False, False, {}

    def close(self):
        self.closed = True


def test_policy_schema_is_versioned_and_has_deterministic_flatten_order():
    assert REACH_SCHEMA_V1.identifier == "rebotarm_reach/v1"
    assert PICK_SCHEMA_V1.identifier == "rebotarm_pick/v1"
    assert get_policy_schema(REACH_SCHEMA_V1.identifier) is REACH_SCHEMA_V1

    flat = REACH_SCHEMA_V1.flatten_observation(_reach_observation())
    assert flat.dtype == np.float32
    assert flat.shape == (REACH_SCHEMA_V1.flat_observation_size,)
    assert REACH_SCHEMA_V1.normalize_action([2.0] * 7) == pytest.approx([1.0] * 7)


def test_reach_task_evaluation_has_no_physics_backend_dependency():
    result = evaluate_reach(_reach_observation(), target_tolerance_m=0.06)

    assert result.distance_to_target_m == pytest.approx(0.05)
    assert result.reward == pytest.approx(-0.05)
    assert result.terminated is True


def test_sync_headless_vector_env_stacks_observations_and_steps_independently():
    environments = [DummyEnv(), DummyEnv()]
    with SyncHeadlessVectorEnv([lambda: environments[0], lambda: environments[1]]) as vector:
        observations, infos = vector.reset(seed=10)
        assert observations["joint_positions"].shape == (2, 6)
        assert observations["joint_positions"][:, 0] == pytest.approx([0.10, 0.11])
        assert [info["seed"] for info in infos] == [10, 11]

        result = vector.step(np.zeros((2, 7), dtype=np.float32))
        assert result[1].shape == (2,)
        assert result[2].dtype == np.bool_
        assert result[3].dtype == np.bool_

    assert all(environment.closed for environment in environments)


def test_sync_headless_vector_env_rejects_wrong_action_batch_shape():
    with SyncHeadlessVectorEnv([DummyEnv]) as vector:
        vector.reset(seed=0)
        with pytest.raises(ValueError, match="actions must have shape"):
            vector.step(np.zeros((1, 6), dtype=np.float32))


def test_gymnasium_adapter_fails_only_when_instantiated_if_dependency_is_missing():
    if gymnasium_available():
        pytest.skip("deployment has optional Gymnasium dependency installed")
    with pytest.raises(ImportError, match="Gymnasium support is optional"):
        RebotArmGymnasiumEnv(DummyEnv())
