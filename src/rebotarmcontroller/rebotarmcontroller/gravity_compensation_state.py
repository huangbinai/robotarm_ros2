from __future__ import annotations

from dataclasses import dataclass
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
