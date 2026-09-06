from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Sequence

import numpy as np

from .mujoco_env import ReachEnvConfig, RebotArmReachEnv
from .pick_task import PickTaskConfig, PickTaskRuntime, summarize_cube_contacts
from .rl_schema import PICK_SCHEMA_V1, PolicySchema
from .simulation_protocol import SimulationProtocol


@dataclass(frozen=True)
class PickEnvConfig(ReachEnvConfig):
    max_steps: int = 400
    settle_steps: int = 200
    task: PickTaskConfig = field(default_factory=PickTaskConfig)

    def __post_init__(self) -> None:
        if self.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.physics_steps_per_action <= 0:
            raise ValueError("physics_steps_per_action must be positive")
        if self.settle_steps < 0:
            raise ValueError("settle_steps must be non-negative")


class RebotArmPickEnv(RebotArmReachEnv):
    """Gym-style cube Pick task with explicit contact and failure semantics."""

    def __init__(
        self,
        model_path: str | None = None,
        *,
        config: PickEnvConfig | None = None,
        sim_factory: Callable[[str | None], SimulationProtocol] | None = None,
    ) -> None:
        super().__init__(model_path, config=config or PickEnvConfig(), sim_factory=sim_factory)
        self.config: PickEnvConfig
        self._task_runtime = PickTaskRuntime(self.config.task)

    @property
    def schema(self) -> PolicySchema:
        return PICK_SCHEMA_V1

    @property
    def stage(self) -> str:
        return self._task_runtime.stage

    @property
    def failure_reason(self) -> str:
        return self._task_runtime.failure_reason

    def reset(
        self,
        *,
        seed: int | None = None,
        randomize: bool = True,
        randomization=None,
    ):
        super().reset(seed=seed, randomize=randomize, randomization=randomization)
        if self.config.settle_steps:
            self._sim.step(self.config.settle_steps)
        state = self._sim.get_state()
        cube_position = np.asarray(state.object_poses["test_cube"][:3], dtype=float)
        self._task_runtime.reset(initial_cube_z=float(cube_position[2]))
        self._target[:] = cube_position + np.array([0.0, 0.0, self.config.task.lift_height_m])
        obs = self._observation()
        return obs, self._pick_info(obs)

    def step(self, action: Sequence[float]):
        self._advance_action(action)
        obs = self._observation()
        contacts = summarize_cube_contacts(self._sim.get_contacts())
        result = self._task_runtime.evaluate(obs, contacts)
        terminated = result.terminated
        truncated = self._step_count >= self.config.max_steps
        reward = result.reward
        info = self._pick_info(obs, contacts=contacts)
        info.update(
            done=terminated or truncated,
            terminated=terminated,
            truncated=truncated,
            is_success=result.is_success,
            lift_height_m=result.lift_height_m,
        )
        return obs, reward, terminated, truncated, info

    def _observation(self) -> dict:
        obs = super()._observation()
        contacts = summarize_cube_contacts(self._sim.get_contacts())
        cube_position = np.asarray(obs["cube_pose"][:3], dtype=np.float32)
        ee_position = np.asarray(obs["ee_position"], dtype=np.float32)
        obs.update(
            cube_position=cube_position,
            cube_to_ee=cube_position - ee_position,
            lift_target_position=self._target.astype(np.float32).copy(),
            left_finger_contact=float(contacts.left_finger_contact),
            right_finger_contact=float(contacts.right_finger_contact),
            bilateral_finger_contact=float(contacts.bilateral_finger_contact),
            force_closure_candidate=float(
                contacts.force_closure_candidate(self.config.task.max_finger_normal_dot)
            ),
            finger_normal_dot=contacts.finger_normal_dot,
            left_finger_force_n=contacts.left_finger_force_n,
            right_finger_force_n=contacts.right_finger_force_n,
            cube_contact_count=contacts.cube_contact_count,
            max_contact_penetration_m=contacts.max_penetration_m,
        )
        return obs

    def _pick_info(self, obs, *, contacts=None) -> dict:
        contacts = contacts or summarize_cube_contacts(self._sim.get_contacts())
        return {
            "step_count": self._step_count,
            "stage": self.stage,
            "failure_reason": self.failure_reason,
            "is_success": self.stage == "success",
            "initial_cube_z_m": self._task_runtime.initial_cube_z,
            "lift_height_m": float(obs["cube_pose"][2]) - self._task_runtime.initial_cube_z,
            "grasp_stable_steps": self._task_runtime.grasp_stable_steps,
            "lift_stable_steps": self._task_runtime.lift_stable_steps,
            "lost_grasp_steps": self._task_runtime.lost_grasp_steps,
            "left_finger_contact": contacts.left_finger_contact,
            "right_finger_contact": contacts.right_finger_contact,
            "bilateral_finger_contact": contacts.bilateral_finger_contact,
            "force_closure_candidate": contacts.force_closure_candidate(
                self.config.task.max_finger_normal_dot
            ),
            "finger_normal_dot": contacts.finger_normal_dot,
            "left_finger_force_n": contacts.left_finger_force_n,
            "right_finger_force_n": contacts.right_finger_force_n,
            "max_cube_contact_force_n": contacts.max_contact_force_n,
            "max_contact_penetration_m": contacts.max_penetration_m,
            "cube_contact_count": contacts.cube_contact_count,
            "schema_version": self.schema.version,
            "schema_id": self.schema.identifier,
        }
