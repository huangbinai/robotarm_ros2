from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class GravityDynamicsAdapter:
    """Expose the gravity operations needed by the hardware control loop."""

    model: Any
    data: Any
    end_effector_frame_id: Any
    compute_generalized_gravity: Callable[..., Any]
    pinocchio: Any

    @classmethod
    def create(
        cls,
        *,
        load_robot_model: Callable[[], Any],
        compute_generalized_gravity: Callable[..., Any],
        pinocchio: Any,
        end_effector_frame: str,
    ) -> "GravityDynamicsAdapter":
        model = load_robot_model()
        return cls(
            model=model,
            data=model.createData(),
            end_effector_frame_id=model.getFrameId(end_effector_frame),
            compute_generalized_gravity=compute_generalized_gravity,
            pinocchio=pinocchio,
        )

    def gravity_torque(self, positions: np.ndarray) -> np.ndarray:
        return self.compute_generalized_gravity(q=positions)

    def end_effector_speeds(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
    ) -> tuple[float, float]:
        self.pinocchio.computeJointJacobians(self.model, self.data, positions)
        self.pinocchio.updateFramePlacements(self.model, self.data)
        jacobian = self.pinocchio.getFrameJacobian(
            self.model,
            self.data,
            self.end_effector_frame_id,
            self.pinocchio.ReferenceFrame.WORLD,
        )
        spatial_velocity = jacobian @ velocities
        return (
            float(np.linalg.norm(spatial_velocity[:3])),
            float(np.linalg.norm(spatial_velocity[3:])),
        )
