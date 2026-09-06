from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from .mujoco_env import RebotArmReachEnv
from .mujoco_pick_env import RebotArmPickEnv
from .rl_schema import PolicySchema


class SyncHeadlessVectorEnv:
    """Dependency-light synchronous vector wrapper for independent simulators.

    This wrapper deliberately does not auto-reset completed environments. That
    keeps episode boundaries explicit and makes dataset generation reproducible.
    """

    def __init__(self, env_fns: Sequence[Callable[[], object]]) -> None:
        if not env_fns:
            raise ValueError("env_fns must contain at least one environment factory")
        self.envs = tuple(factory() for factory in env_fns)
        self.num_envs = len(self.envs)
        self.schema: PolicySchema = self.envs[0].schema
        if any(env.schema.identifier != self.schema.identifier for env in self.envs[1:]):
            self.close()
            raise ValueError("all vector environments must use the same policy schema")
        self._closed = False

    def reset(self, *, seed: int | Sequence[int] | None = None, **kwargs):
        seeds = _expand_seeds(seed, self.num_envs)
        results = [
            env.reset(seed=environment_seed, **kwargs)
            for env, environment_seed in zip(self.envs, seeds)
        ]
        observations, infos = zip(*results)
        return _stack_observations(observations, self.schema), list(infos)

    def step(self, actions: Sequence[Sequence[float]]):
        action_array = np.asarray(actions, dtype=np.float32)
        expected = (self.num_envs,) + self.schema.action.shape
        if action_array.shape != expected:
            raise ValueError(f"actions must have shape {expected}, got {action_array.shape}")
        results = [env.step(action) for env, action in zip(self.envs, action_array)]
        observations, rewards, terminated, truncated, infos = zip(*results)
        return (
            _stack_observations(observations, self.schema),
            np.asarray(rewards, dtype=np.float32),
            np.asarray(terminated, dtype=np.bool_),
            np.asarray(truncated, dtype=np.bool_),
            list(infos),
        )

    def close(self) -> None:
        if getattr(self, "_closed", False):
            return
        for env in self.envs:
            env.close()
        self._closed = True

    def __enter__(self) -> "SyncHeadlessVectorEnv":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def make_headless_vector_env(
    task: str,
    num_envs: int,
    *,
    model_path: str | None = None,
    config=None,
    sim_factory: Callable | None = None,
) -> SyncHeadlessVectorEnv:
    """Create independent headless Reach or Pick environments in one process."""

    if num_envs <= 0:
        raise ValueError("num_envs must be positive")
    env_class = {"reach": RebotArmReachEnv, "pick": RebotArmPickEnv}.get(task.lower())
    if env_class is None:
        raise ValueError("task must be 'reach' or 'pick'")

    def create_environment():
        return env_class(model_path, config=config, sim_factory=sim_factory)

    return SyncHeadlessVectorEnv([create_environment for _ in range(num_envs)])


def _expand_seeds(seed: int | Sequence[int] | None, count: int) -> list[int | None]:
    if seed is None:
        return [None] * count
    if isinstance(seed, (int, np.integer)):
        return [int(seed) + index for index in range(count)]
    seeds = list(seed)
    if len(seeds) != count:
        raise ValueError(f"seed sequence must contain {count} values")
    return [int(value) for value in seeds]


def _stack_observations(observations, schema: PolicySchema) -> dict[str, np.ndarray]:
    normalized = [schema.normalize_observation(observation) for observation in observations]
    return {
        field.name: np.stack([item[field.name] for item in normalized], axis=0)
        for field in schema.observations
    }
