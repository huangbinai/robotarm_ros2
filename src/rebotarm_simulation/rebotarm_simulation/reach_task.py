from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class ReachTaskResult:
    reward: float
    terminated: bool
    distance_to_target_m: float
    is_success: bool


def evaluate_reach(
    observation: Mapping[str, object], *, target_tolerance_m: float
) -> ReachTaskResult:
    """Evaluate Reach without depending on MuJoCo or an environment class."""

    distance = float(
        np.linalg.norm(
            np.asarray(observation["ee_position"], dtype=float)
            - np.asarray(observation["target_position"], dtype=float)
        )
    )
    success = distance <= float(target_tolerance_m)
    return ReachTaskResult(
        reward=-distance,
        terminated=success,
        distance_to_target_m=distance,
        is_success=success,
    )
