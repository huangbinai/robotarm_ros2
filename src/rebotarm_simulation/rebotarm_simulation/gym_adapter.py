from __future__ import annotations

from typing import Any, Callable, Mapping, Sequence

import numpy as np

from .mujoco_env import RebotArmReachEnv
from .mujoco_pick_env import RebotArmPickEnv
from .rl_schema import PolicySchema, TensorField


try:  # Gymnasium is intentionally an optional training-only dependency.
    import gymnasium as gym
except ImportError:  # pragma: no cover - availability depends on deployment profile
    gym = None


GYMNASIUM_REACH_ID = "RebotArmReach-v1"
GYMNASIUM_PICK_ID = "RebotArmPick-v1"


def gymnasium_available() -> bool:
    return gym is not None


def _require_gymnasium():
    if gym is None:
        raise ImportError(
            "Gymnasium support is optional; install 'gymnasium>=0.29' to use "
            "rebotarm_simulation.gym_adapter"
        )
    return gym


def _field_space(field: TensorField):
    gym_module = _require_gymnasium()
    return gym_module.spaces.Box(
        low=field.low,
        high=field.high,
        shape=field.shape,
        dtype=field.dtype,
    )


def spaces_for_schema(schema: PolicySchema):
    gym_module = _require_gymnasium()
    action_space = _field_space(schema.action)
    observation_space = gym_module.spaces.Dict(
        {field.name: _field_space(field) for field in schema.observations}
    )
    return action_space, observation_space


_GymBase = gym.Env if gym is not None else object


class RebotArmGymnasiumEnv(_GymBase):
    """Strict Gymnasium adapter around a dependency-light core task env."""

    metadata = {"render_modes": []}

    def __init__(self, core_env) -> None:
        _require_gymnasium()
        super().__init__()
        self.core_env = core_env
        self.schema = core_env.schema
        self.action_space, self.observation_space = spaces_for_schema(self.schema)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: Mapping[str, Any] | None = None,
    ):
        super().reset(seed=seed)
        options = dict(options or {})
        unknown = set(options) - {"randomize", "randomization"}
        if unknown:
            raise ValueError(f"unsupported reset options: {', '.join(sorted(unknown))}")
        observation, info = self.core_env.reset(seed=seed, **options)
        return self.schema.normalize_observation(observation), info

    def step(self, action: Sequence[float]):
        normalized_action = self.schema.normalize_action(action)
        observation, reward, terminated, truncated, info = self.core_env.step(normalized_action)
        return (
            self.schema.normalize_observation(observation),
            float(reward),
            bool(terminated),
            bool(truncated),
            info,
        )

    def close(self) -> None:
        self.core_env.close()


class RebotArmReachGymnasiumEnv(RebotArmGymnasiumEnv):
    def __init__(self, model_path: str | None = None, *, config=None, sim_factory=None) -> None:
        super().__init__(
            RebotArmReachEnv(model_path, config=config, sim_factory=sim_factory)
        )


class RebotArmPickGymnasiumEnv(RebotArmGymnasiumEnv):
    def __init__(self, model_path: str | None = None, *, config=None, sim_factory=None) -> None:
        super().__init__(
            RebotArmPickEnv(model_path, config=config, sim_factory=sim_factory)
        )


def register_gymnasium_envs() -> None:
    """Register stable IDs explicitly, without adding import-time side effects."""

    gym_module = _require_gymnasium()
    registrations: tuple[tuple[str, str], ...] = (
        (
            GYMNASIUM_REACH_ID,
            "rebotarm_simulation.gym_adapter:RebotArmReachGymnasiumEnv",
        ),
        (
            GYMNASIUM_PICK_ID,
            "rebotarm_simulation.gym_adapter:RebotArmPickGymnasiumEnv",
        ),
    )
    for environment_id, entry_point in registrations:
        if environment_id not in gym_module.registry:
            gym_module.register(id=environment_id, entry_point=entry_point)
