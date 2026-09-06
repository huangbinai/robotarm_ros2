from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np


RL_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class TensorField:
    """Versioned description of one policy-facing tensor."""

    name: str
    shape: tuple[int, ...]
    low: float = -np.inf
    high: float = np.inf
    dtype: np.dtype = np.dtype(np.float32)

    def coerce(self, value) -> np.ndarray:
        result = np.asarray(value, dtype=self.dtype)
        if result.shape != self.shape:
            raise ValueError(f"{self.name} must have shape {self.shape}, got {result.shape}")
        if not np.isfinite(result).all():
            raise ValueError(f"{self.name} must contain finite values")
        return result


@dataclass(frozen=True)
class PolicySchema:
    """Stable action/observation contract for saved policies and datasets."""

    name: str
    version: int
    action: TensorField
    observations: tuple[TensorField, ...]

    @property
    def identifier(self) -> str:
        return f"{self.name}/v{self.version}"

    @property
    def flat_observation_size(self) -> int:
        return sum(int(np.prod(field.shape, dtype=int)) for field in self.observations)

    def normalize_action(self, action: Sequence[float]) -> np.ndarray:
        result = self.action.coerce(action)
        return np.clip(result, self.action.low, self.action.high)

    def normalize_observation(self, observation: Mapping[str, object]) -> dict[str, np.ndarray]:
        missing = [field.name for field in self.observations if field.name not in observation]
        if missing:
            raise ValueError(f"missing observation fields: {', '.join(missing)}")
        return {
            field.name: field.coerce(observation[field.name])
            for field in self.observations
        }

    def flatten_observation(self, observation: Mapping[str, object]) -> np.ndarray:
        normalized = self.normalize_observation(observation)
        return np.concatenate(
            [normalized[field.name].reshape(-1) for field in self.observations]
        ).astype(np.float32, copy=False)


_ACTION_V1 = TensorField("action", (7,), -1.0, 1.0)

REACH_SCHEMA_V1 = PolicySchema(
    name="rebotarm_reach",
    version=RL_SCHEMA_VERSION,
    action=_ACTION_V1,
    observations=(
        TensorField("joint_positions", (6,)),
        TensorField("joint_velocities", (6,)),
        TensorField("gripper_width", (), 0.0, 0.2),
        TensorField("ee_position", (3,)),
        TensorField("target_position", (3,)),
        TensorField("cube_pose", (7,)),
        TensorField("max_contact_force", (), 0.0),
    ),
)

PICK_SCHEMA_V1 = PolicySchema(
    name="rebotarm_pick",
    version=RL_SCHEMA_VERSION,
    action=_ACTION_V1,
    observations=REACH_SCHEMA_V1.observations
    + (
        TensorField("cube_position", (3,)),
        TensorField("cube_to_ee", (3,)),
        TensorField("lift_target_position", (3,)),
        TensorField("left_finger_contact", (), 0.0, 1.0),
        TensorField("right_finger_contact", (), 0.0, 1.0),
        TensorField("bilateral_finger_contact", (), 0.0, 1.0),
        TensorField("force_closure_candidate", (), 0.0, 1.0),
        TensorField("finger_normal_dot", (), -1.0, 1.0),
        TensorField("left_finger_force_n", (), 0.0),
        TensorField("right_finger_force_n", (), 0.0),
        TensorField("cube_contact_count", (), 0.0),
        TensorField("max_contact_penetration_m", (), 0.0),
    ),
)

POLICY_SCHEMAS = MappingProxyType(
    {schema.identifier: schema for schema in (REACH_SCHEMA_V1, PICK_SCHEMA_V1)}
)


def get_policy_schema(identifier: str) -> PolicySchema:
    try:
        return POLICY_SCHEMAS[identifier]
    except KeyError as exc:
        supported = ", ".join(POLICY_SCHEMAS)
        raise ValueError(f"unsupported policy schema {identifier!r}; supported: {supported}") from exc
