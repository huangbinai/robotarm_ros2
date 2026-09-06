"""Resolved model indices and compatibility identity for the MuJoCo runtime."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from typing import Any, Sequence

import numpy as np

from .model_contract import END_EFFECTOR_SITE_NAME, JOINT_NAMES, actuator_name_for_joint


@dataclass(frozen=True)
class FreeBodyIndex:
    body_id: int
    qpos_address: int


class MujocoModelIndex:
    """Resolve and validate every stable model identifier in one place.

    Runtime code consumes this immutable index instead of repeatedly coupling
    itself to MuJoCo's name lookup and address tables.
    """

    def __init__(self, mj: Any, model: Any) -> None:
        self._mj = mj
        self._model = model
        self.joint_ids = tuple(
            self._required_id(mj.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES
        )
        self.actuator_ids = tuple(
            self._required_id(mj.mjtObj.mjOBJ_ACTUATOR, actuator_name_for_joint(name))
            for name in JOINT_NAMES
        )
        self.end_effector_site_id = self._required_id(
            mj.mjtObj.mjOBJ_SITE, END_EFFECTOR_SITE_NAME
        )
        self.free_bodies = self._find_free_bodies()
        self.state_spec = (
            int(mj.mjtState.mjSTATE_INTEGRATION)
            | int(mj.mjtState.mjSTATE_USER)
            | int(mj.mjtState.mjSTATE_CTRL)
        )
        self.model_dimensions = tuple(
            int(value)
            for value in (
                model.nq,
                model.nv,
                model.na,
                model.nu,
                model.nbody,
                model.njnt,
                model.ngeom,
                mj.mj_stateSize(model, self.state_spec),
            )
        )
        self.model_fingerprint = self._fingerprint_model()

    def _required_id(self, object_type: Any, name: str) -> int:
        identifier = int(self._mj.mj_name2id(self._model, object_type, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model is missing required {name!r}")
        return identifier

    def _find_free_bodies(self) -> dict[str, FreeBodyIndex]:
        result: dict[str, FreeBodyIndex] = {}
        free_type = int(self._mj.mjtJoint.mjJNT_FREE)
        for body_id in range(1, int(self._model.nbody)):
            joint_start = int(self._model.body_jntadr[body_id])
            joint_count = int(self._model.body_jntnum[body_id])
            for joint_id in range(joint_start, joint_start + joint_count):
                if int(self._model.jnt_type[joint_id]) != free_type:
                    continue
                name = self._mj.mj_id2name(
                    self._model, self._mj.mjtObj.mjOBJ_BODY, body_id
                )
                result[str(name)] = FreeBodyIndex(
                    body_id=body_id,
                    qpos_address=int(self._model.jnt_qposadr[joint_id]),
                )
        return result

    def _fingerprint_model(self) -> str:
        digest = hashlib.sha256()
        digest.update(repr(self.model_dimensions).encode("ascii"))
        values: Sequence[Any] = (
            self._model.names,
            self._model.jnt_type,
            self._model.jnt_qposadr,
            self._model.jnt_dofadr,
            self._model.jnt_range,
            self._model.actuator_trnid,
            self._model.geom_bodyid,
        )
        for item in values:
            digest.update(np.asarray(item).tobytes())
        digest.update(
            np.asarray([self._model.opt.timestep], dtype=np.float64).tobytes()
        )
        return digest.hexdigest()
