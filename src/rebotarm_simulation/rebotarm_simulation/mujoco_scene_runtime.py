"""Free-object state and deterministic scene randomization for MuJoCo."""

from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np

from .model_contract import TEST_CUBE_BODY_NAME
from .mujoco_model_index import FreeBodyIndex
from .mujoco_types import RandomizedScene


def _finite_vector(values: Sequence[float], length: int, label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a numeric sequence") from exc
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


def _bounds(values: Sequence[Sequence[float]], count: int, label: str):
    bounds = tuple(_finite_vector(item, 2, label) for item in values)
    if len(bounds) != count:
        raise ValueError(f"{label} must contain exactly {count} bounds")
    for lower, upper in bounds:
        if lower > upper:
            raise ValueError(f"{label} lower bound must be <= upper bound")
    return bounds


class MujocoSceneRuntime:
    def __init__(
        self,
        mj: Any,
        model: Any,
        data: Any,
        free_bodies: dict[str, FreeBodyIndex],
    ) -> None:
        self._mj = mj
        self._model = model
        self._data = data
        self._free_bodies = dict(free_bodies)
        self._rng = np.random.default_rng()

    def reset_random_generator(self, seed: int | None) -> None:
        self._rng = np.random.default_rng(seed)

    def object_poses(self) -> dict[str, tuple[float, ...]]:
        return {
            name: (
                *(float(value) for value in self._data.qpos[item.qpos_address : item.qpos_address + 3]),
                *(float(value) for value in self._data.qpos[item.qpos_address + 4 : item.qpos_address + 7]),
                float(self._data.qpos[item.qpos_address + 3]),
            )
            for name, item in self._free_bodies.items()
        }

    def set_object_pose(
        self,
        body_name: str,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        zero_velocity: bool = True,
    ) -> tuple[float, ...]:
        if body_name not in self._free_bodies:
            raise ValueError(f"Body {body_name!r} is not a free object")
        position_values = _finite_vector(position, 3, "position")
        quaternion = np.asarray(
            _finite_vector(orientation, 4, "orientation"), dtype=float
        )
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            raise ValueError("orientation quaternion must have non-zero norm")
        quaternion /= norm
        item = self._free_bodies[body_name]
        orientation_xyzw = tuple(float(value) for value in quaternion)
        self._data.qpos[item.qpos_address : item.qpos_address + 7] = (
            *position_values,
            orientation_xyzw[3],
            *orientation_xyzw[:3],
        )
        if zero_velocity:
            joint_id = int(self._model.body_jntadr[item.body_id])
            dof_address = int(self._model.jnt_dofadr[joint_id])
            self._data.qvel[dof_address : dof_address + 6] = 0.0
        self._mj.mj_forward(self._model, self._data)
        return (*position_values, *orientation_xyzw)

    def randomize(
        self,
        seed: int | None = None,
        *,
        cube_xy_bounds: Sequence[Sequence[float]] = ((0.22, 0.38), (-0.14, 0.14)),
        cube_z: float = 0.04,
        reach_target_bounds: Sequence[Sequence[float]] = (
            (0.18, 0.45),
            (-0.22, 0.22),
            (0.08, 0.35),
        ),
    ) -> RandomizedScene:
        rng = np.random.default_rng(seed) if seed is not None else self._rng
        cube_x_bounds, cube_y_bounds = _bounds(cube_xy_bounds, 2, "cube_xy_bounds")
        target_x_bounds, target_y_bounds, target_z_bounds = _bounds(
            reach_target_bounds, 3, "reach_target_bounds"
        )
        cube_position = (
            float(rng.uniform(*cube_x_bounds)),
            float(rng.uniform(*cube_y_bounds)),
            float(cube_z),
        )
        target = (
            float(rng.uniform(*target_x_bounds)),
            float(rng.uniform(*target_y_bounds)),
            float(rng.uniform(*target_z_bounds)),
        )
        cube_pose = self.set_object_pose(
            TEST_CUBE_BODY_NAME, cube_position, (0.0, 0.0, 0.0, 1.0)
        )
        return RandomizedScene(
            cube_pose=cube_pose,
            reach_target_position=target,
            seed=seed,
        )
