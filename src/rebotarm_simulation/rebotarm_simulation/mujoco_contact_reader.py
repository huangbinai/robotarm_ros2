"""Read-only conversion of native MuJoCo contacts to public domain types."""

from __future__ import annotations

from typing import Any

import numpy as np

from .mujoco_types import ContactInfo


class MujocoContactReader:
    def __init__(self, mj: Any, model: Any, data: Any) -> None:
        self._mj = mj
        self._model = model
        self._data = data

    def read(self) -> tuple[ContactInfo, ...]:
        contacts: list[ContactInfo] = []
        force = np.zeros(6, dtype=float)
        for index in range(int(self._data.ncon)):
            contact = self._data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            body1 = int(self._model.geom_bodyid[geom1])
            body2 = int(self._model.geom_bodyid[geom2])
            force.fill(0.0)
            self._mj.mj_contactForce(self._model, self._data, index, force)
            contacts.append(
                ContactInfo(
                    body1=self._name_or_default(
                        self._mj.mjtObj.mjOBJ_BODY, body1, "world"
                    ),
                    body2=self._name_or_default(
                        self._mj.mjtObj.mjOBJ_BODY, body2, "world"
                    ),
                    geom1=self._name_or_default(
                        self._mj.mjtObj.mjOBJ_GEOM, geom1, f"geom{geom1}"
                    ),
                    geom2=self._name_or_default(
                        self._mj.mjtObj.mjOBJ_GEOM, geom2, f"geom{geom2}"
                    ),
                    position=tuple(float(value) for value in contact.pos),
                    force=float(np.linalg.norm(force[:3])),
                    penetration_depth=max(0.0, -float(contact.dist)),
                    normal=tuple(float(value) for value in contact.frame[:3]),
                )
            )
        return tuple(contacts)

    def _name_or_default(self, object_type: Any, identifier: int, default: str) -> str:
        return str(
            self._mj.mj_id2name(self._model, object_type, identifier) or default
        )
