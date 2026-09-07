from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class GravityCompensationState:
    """Mutable targets and accumulators used by gravity compensation."""

    active: bool = False
    target: np.ndarray | None = None
    integral: np.ndarray | None = None
    lock_counter: int = 0
    last_position: np.ndarray | None = None

    def start(self, target: np.ndarray) -> None:
        self.target = np.asarray(target, dtype=np.float64).copy()
        self.last_position = self.target.copy()
        self.integral = np.zeros_like(self.target)
        self.lock_counter = 0
        self.active = True

    def finish(self) -> None:
        self.active = False
        self.target = None
        self.integral = None
        self.lock_counter = 0
        self.last_position = None

    def deactivate(self) -> None:
        self.active = False

    def target_copy(self) -> np.ndarray | None:
        if self.target is None:
            return None
        return self.target.copy()

    def remember_position(self, position: np.ndarray) -> np.ndarray:
        self.last_position = np.array(position, dtype=np.float64, copy=True)
        return self.last_position.copy()

    def accumulate_error(
        self,
        error: np.ndarray,
        *,
        template: np.ndarray,
    ) -> np.ndarray:
        if self.integral is None:
            self.integral = np.zeros_like(template)
        self.integral += error * 1.0
        np.clip(self.integral, -0.5, 0.5, out=self.integral)
        return self.integral

    def observe_motion(self, position: np.ndarray, *, moving: bool) -> None:
        if moving:
            self.target = position.copy()
            self.lock_counter = 0
            if self.integral is not None:
                self.integral *= 0.9
            return
        self.lock_counter += 1


class GravityCompensationField:
    """Compatibility descriptor for legacy private HardwareManager fields."""

    def __init__(self, state_field: str) -> None:
        self._state_field = state_field

    @staticmethod
    def _state(instance: Any) -> GravityCompensationState:
        state = instance.__dict__.get("_gravity_state")
        if state is None:
            state = GravityCompensationState()
            instance.__dict__["_gravity_state"] = state
        return state

    def __get__(self, instance: Any, owner: type | None = None) -> Any:
        if instance is None:
            return self
        return getattr(self._state(instance), self._state_field)

    def __set__(self, instance: Any, value: Any) -> None:
        setattr(self._state(instance), self._state_field, value)
