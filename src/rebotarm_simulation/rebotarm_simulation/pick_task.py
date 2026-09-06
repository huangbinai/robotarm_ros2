from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np


PICK_STAGES = ("approach", "contact", "grasp", "lift", "success", "failure")
PICK_FAILURES = (
    "none",
    "excessive_contact_force",
    "excessive_penetration",
    "cube_fell",
    "cube_out_of_workspace",
    "dropped_after_grasp",
)


@dataclass(frozen=True)
class PickTaskConfig:
    lift_height_m: float = 0.05
    approach_tolerance_m: float = 0.06
    max_grasp_width_m: float = 0.065
    max_finger_normal_dot: float = -0.3
    grasp_stability_steps: int = 5
    success_stability_steps: int = 5
    drop_patience_steps: int = 10
    max_contact_force_n: float = 20.0
    max_contact_penetration_m: float = 0.01
    min_cube_z_m: float = -0.02
    workspace_x: tuple[float, float] = (-0.10, 0.65)
    workspace_y: tuple[float, float] = (-0.50, 0.50)
    workspace_z_max_m: float = 0.70

    def __post_init__(self) -> None:
        positive = (
            "lift_height_m",
            "approach_tolerance_m",
            "max_grasp_width_m",
            "max_contact_force_n",
            "max_contact_penetration_m",
            "workspace_z_max_m",
        )
        for field_name in positive:
            value = float(getattr(self, field_name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field_name} must be positive")
            object.__setattr__(self, field_name, value)
        for field_name in (
            "grasp_stability_steps",
            "success_stability_steps",
            "drop_patience_steps",
        ):
            value = int(getattr(self, field_name))
            if value <= 0:
                raise ValueError(f"{field_name} must be positive")
            object.__setattr__(self, field_name, value)
        min_cube_z = float(self.min_cube_z_m)
        if not math.isfinite(min_cube_z):
            raise ValueError("min_cube_z_m must be finite")
        object.__setattr__(self, "min_cube_z_m", min_cube_z)
        normal_dot = float(self.max_finger_normal_dot)
        if not math.isfinite(normal_dot) or not -1.0 <= normal_dot <= 1.0:
            raise ValueError("max_finger_normal_dot must be between -1 and 1")
        object.__setattr__(self, "max_finger_normal_dot", normal_dot)
        for field_name in ("workspace_x", "workspace_y"):
            lower, upper = (float(value) for value in getattr(self, field_name))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError(f"{field_name} must contain ordered finite bounds")
            object.__setattr__(self, field_name, (lower, upper))


@dataclass(frozen=True)
class CubeContactSummary:
    left_finger_contact: bool
    right_finger_contact: bool
    table_contact: bool
    left_finger_force_n: float
    right_finger_force_n: float
    max_contact_force_n: float
    max_penetration_m: float
    cube_contact_count: int
    finger_normal_dot: float

    @property
    def bilateral_finger_contact(self) -> bool:
        return self.left_finger_contact and self.right_finger_contact

    def force_closure_candidate(self, max_normal_dot: float) -> bool:
        return self.bilateral_finger_contact and self.finger_normal_dot <= max_normal_dot


@dataclass(frozen=True)
class PickTaskResult:
    reward: float
    terminated: bool
    stage: str
    failure_reason: str
    is_success: bool
    lift_height_m: float


class PickTaskRuntime:
    """Backend-independent Pick episode state and task evaluation."""

    def __init__(self, config: PickTaskConfig) -> None:
        self.config = config
        self.reset(initial_cube_z=0.0)

    def reset(self, *, initial_cube_z: float) -> None:
        self.initial_cube_z = float(initial_cube_z)
        self.grasp_stable_steps = 0
        self.lift_stable_steps = 0
        self.lost_grasp_steps = 0
        self.ever_grasped = False
        self.stage = "approach"
        self.failure_reason = "none"

    def evaluate(
        self,
        observation: Mapping[str, object],
        contacts: CubeContactSummary,
    ) -> PickTaskResult:
        force_closure = contacts.force_closure_candidate(self.config.max_finger_normal_dot)
        grasp_candidate = (
            force_closure
            and float(observation["gripper_width"]) <= self.config.max_grasp_width_m
        )
        self.grasp_stable_steps = self.grasp_stable_steps + 1 if grasp_candidate else 0
        grasped = self.grasp_stable_steps >= self.config.grasp_stability_steps
        self.ever_grasped = self.ever_grasped or grasped
        if self.ever_grasped and not contacts.bilateral_finger_contact:
            self.lost_grasp_steps += 1
        else:
            self.lost_grasp_steps = 0

        lift_height = float(np.asarray(observation["cube_pose"])[2]) - self.initial_cube_z
        lifted = lift_height >= self.config.lift_height_m
        self.lift_stable_steps = self.lift_stable_steps + 1 if lifted and force_closure else 0
        success = self.lift_stable_steps >= self.config.success_stability_steps
        self.failure_reason = pick_failure_reason(
            config=self.config,
            cube_position=np.asarray(observation["cube_pose"])[0:3],
            contacts=contacts,
            ever_grasped=self.ever_grasped,
            lost_grasp_steps=self.lost_grasp_steps,
        )
        failed = self.failure_reason != "none"
        self.stage = self._resolve_stage(observation, contacts, success, failed)
        reward = self._reward(observation, contacts, lift_height, success, failed)
        return PickTaskResult(
            reward=reward,
            terminated=success or failed,
            stage=self.stage,
            failure_reason=self.failure_reason,
            is_success=success,
            lift_height_m=lift_height,
        )

    def _resolve_stage(self, observation, contacts, success: bool, failed: bool) -> str:
        if success:
            return "success"
        if failed:
            return "failure"
        if self.ever_grasped:
            return "lift"
        if contacts.force_closure_candidate(self.config.max_finger_normal_dot):
            return "grasp"
        if contacts.left_finger_contact or contacts.right_finger_contact:
            return "contact"
        distance = float(np.linalg.norm(observation["cube_to_ee"]))
        return "contact" if distance <= self.config.approach_tolerance_m else "approach"

    def _reward(self, observation, contacts, lift_height, success, failed) -> float:
        distance = float(np.linalg.norm(observation["cube_to_ee"]))
        reward = -distance + 2.0 * max(0.0, lift_height)
        reward += 0.1 * float(contacts.left_finger_contact)
        reward += 0.1 * float(contacts.right_finger_contact)
        reward += 0.25 * float(self.ever_grasped)
        if success:
            reward += 10.0
        if failed:
            reward -= 5.0
        return float(reward)


def summarize_cube_contacts(contacts: Sequence, *, cube_body: str = "test_cube") -> CubeContactSummary:
    left_force = right_force = max_force = max_penetration = 0.0
    left = right = table = False
    count = 0
    left_normal = right_normal = None
    for contact in contacts:
        if cube_body not in (contact.body1, contact.body2):
            continue
        count += 1
        names = f"{contact.body1} {contact.body2}"
        force = float(contact.force)
        penetration = float(getattr(contact, "penetration_depth", 0.0))
        max_force = max(max_force, force)
        max_penetration = max(max_penetration, penetration)
        if "left_finger" in names:
            left = True
            if force >= left_force:
                left_force = force
                left_normal = _cube_outward_normal(contact, cube_body)
        if "right_finger" in names:
            right = True
            if force >= right_force:
                right_force = force
                right_normal = _cube_outward_normal(contact, cube_body)
        if "table" in names:
            table = True
    normal_dot = 1.0
    if left_normal is not None and right_normal is not None:
        normal_dot = sum(left * right for left, right in zip(left_normal, right_normal))
    return CubeContactSummary(
        left_finger_contact=left,
        right_finger_contact=right,
        table_contact=table,
        left_finger_force_n=left_force,
        right_finger_force_n=right_force,
        max_contact_force_n=max_force,
        max_penetration_m=max_penetration,
        cube_contact_count=count,
        finger_normal_dot=float(normal_dot),
    )


def _cube_outward_normal(contact, cube_body: str) -> tuple[float, float, float]:
    normal = tuple(float(value) for value in getattr(contact, "normal", (0.0, 0.0, 0.0)))
    if contact.body2 == cube_body:
        normal = tuple(-value for value in normal)
    magnitude = math.sqrt(sum(value * value for value in normal))
    if magnitude <= 1e-12:
        return (0.0, 0.0, 0.0)
    return tuple(value / magnitude for value in normal)


def pick_failure_reason(
    *,
    config: PickTaskConfig,
    cube_position: Sequence[float],
    contacts: CubeContactSummary,
    ever_grasped: bool,
    lost_grasp_steps: int,
) -> str:
    x, y, z = (float(value) for value in cube_position)
    if contacts.max_contact_force_n > config.max_contact_force_n:
        return "excessive_contact_force"
    if contacts.max_penetration_m > config.max_contact_penetration_m:
        return "excessive_penetration"
    if z < config.min_cube_z_m:
        return "cube_fell"
    if (
        not config.workspace_x[0] <= x <= config.workspace_x[1]
        or not config.workspace_y[0] <= y <= config.workspace_y[1]
        or z > config.workspace_z_max_m
    ):
        return "cube_out_of_workspace"
    if ever_grasped and lost_grasp_steps >= config.drop_patience_steps:
        return "dropped_after_grasp"
    return "none"
