from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence

import numpy as np

from .mujoco_sim import RebotArmMujoco


@dataclass(frozen=True)
class ReachEnvConfig:
    max_steps: int = 200
    physics_steps_per_action: int = 10
    action_scale_rad: float = 0.02
    gripper_action_scale_m: float = 0.002
    target_tolerance_m: float = 0.03
    home_on_reset: bool = True


class RebotArmReachEnv:
    """Small Gym-style Reach task wrapper around the MuJoCo simulator.

    It intentionally avoids depending on gymnasium so the ROS 2 simulation
    package remains lightweight. The default method signatures mirror
    Gymnasium: reset() -> (obs, info), step(action) -> (obs, reward,
    terminated, truncated, info). step_done(action) returns the older Gym
    shape: (obs, reward, done, info).
    """

    def __init__(
        self,
        model_path: str | None = None,
        *,
        config: ReachEnvConfig | None = None,
        sim_factory: Callable = RebotArmMujoco,
    ) -> None:
        self.config = config or ReachEnvConfig()
        if self.config.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.config.physics_steps_per_action <= 0:
            raise ValueError("physics_steps_per_action must be positive")
        self._sim = sim_factory(model_path)
        self._target = np.zeros(3, dtype=float)
        self._step_count = 0

    @property
    def sim(self):
        return self._sim

    @property
    def target_position(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self._target)

    def reset(self, *, seed: int | None = None, randomize: bool = True):
        self._step_count = 0
        if self.config.home_on_reset and hasattr(self._sim, "reset_home"):
            self._sim.reset_home(seed=seed)
        else:
            self._sim.reset(seed=seed)
        if randomize:
            scene = self._sim.randomize_scene(seed=seed)
            self._target[:] = np.asarray(scene.reach_target_position, dtype=float)
        else:
            self._target[:] = self._sim.get_state().end_effector_position
        obs = self._observation()
        return obs, self._info(obs)

    def step(self, action: Sequence[float]):
        action_vector = _finite_action(action)
        current_targets = np.asarray(self._sim.control_targets[:6], dtype=float)
        target = current_targets + action_vector[:6] * self.config.action_scale_rad
        self._sim.set_joint_position_targets(target)
        if len(action_vector) >= 7:
            current_width = float(self._sim.control_targets[-2] - self._sim.control_targets[-1])
            self._sim.set_gripper_width(
                current_width + action_vector[6] * self.config.gripper_action_scale_m
            )
        self._sim.step(self.config.physics_steps_per_action)
        self._step_count += 1
        obs = self._observation()
        distance = float(np.linalg.norm(obs["ee_position"] - obs["target_position"]))
        reward = -distance
        terminated = distance <= self.config.target_tolerance_m
        truncated = self._step_count >= self.config.max_steps
        info = self._info(obs)
        info["done"] = terminated or truncated
        info["terminated"] = terminated
        info["truncated"] = truncated
        return obs, reward, terminated, truncated, info

    def step_done(self, action: Sequence[float]):
        obs, reward, terminated, truncated, info = self.step(action)
        return obs, reward, bool(terminated or truncated), info

    def close(self) -> None:
        self._sim.close()

    def _observation(self) -> dict[str, np.ndarray | float]:
        state = self._sim.get_state()
        contacts = self._sim.get_contacts()
        max_contact_force = max((contact.force for contact in contacts), default=0.0)
        return {
            "joint_positions": np.asarray(state.joint_positions[:6], dtype=np.float32),
            "joint_velocities": np.asarray(state.joint_velocities[:6], dtype=np.float32),
            "gripper_width": float(state.gripper_width),
            "ee_position": np.asarray(state.end_effector_position, dtype=np.float32),
            "target_position": self._target.astype(np.float32).copy(),
            "cube_pose": np.asarray(state.object_poses["test_cube"], dtype=np.float32),
            "max_contact_force": float(max_contact_force),
        }

    def _info(self, obs: dict[str, np.ndarray | float]) -> dict[str, float | int | bool]:
        distance = float(np.linalg.norm(obs["ee_position"] - obs["target_position"]))
        return {
            "step_count": self._step_count,
            "distance_to_target_m": distance,
            "is_success": distance <= self.config.target_tolerance_m,
        }

    def __enter__(self) -> "RebotArmReachEnv":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def _finite_action(action: Sequence[float]) -> np.ndarray:
    if isinstance(action, (str, bytes)):
        raise TypeError("action must be a numeric sequence")
    result = np.asarray(tuple(action), dtype=float)
    if result.shape not in {(6,), (7,)}:
        raise ValueError(f"action must have shape (6,) or (7,), got {result.shape}")
    if not np.isfinite(result).all():
        raise ValueError("action values must be finite")
    return np.clip(result, -1.0, 1.0)
