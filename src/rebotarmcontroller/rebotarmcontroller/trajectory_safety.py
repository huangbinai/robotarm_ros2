from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence

import numpy as np


ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
ARM_POSITION_MIN_RAD = np.array([-2.8, -3.14, -3.14, -1.87, -1.57, -3.14])
ARM_POSITION_MAX_RAD = np.array([2.8, 0.0, 0.0, 1.57, 1.57, 3.14])
ARM_MAX_VELOCITY_RAD_S = np.array([3.0, 3.0, 3.0, 1.8, 1.8, 1.8])
ARM_MAX_ACCELERATION_RAD_S2 = np.full(6, 5.0)


@dataclass(frozen=True)
class TrajectorySafetyLimits:
    position_min: np.ndarray = field(default_factory=lambda: ARM_POSITION_MIN_RAD.copy())
    position_max: np.ndarray = field(default_factory=lambda: ARM_POSITION_MAX_RAD.copy())
    max_velocity: np.ndarray = field(default_factory=lambda: ARM_MAX_VELOCITY_RAD_S.copy())
    max_acceleration: np.ndarray = field(default_factory=lambda: ARM_MAX_ACCELERATION_RAD_S2.copy())
    start_tolerance_rad: float = 0.10

    def __post_init__(self) -> None:
        for name in ("position_min", "position_max", "max_velocity", "max_acceleration"):
            array = np.asarray(getattr(self, name), dtype=np.float64).reshape(-1)
            if array.shape != (6,) or not np.all(np.isfinite(array)):
                raise ValueError(f"{name} must contain six finite values")
            object.__setattr__(self, name, array.copy())
        if np.any(self.position_min >= self.position_max):
            raise ValueError("position_min must be below position_max")
        if np.any(self.max_velocity <= 0.0) or np.any(self.max_acceleration <= 0.0):
            raise ValueError("trajectory velocity and acceleration limits must be positive")
        if not math.isfinite(self.start_tolerance_rad) or self.start_tolerance_rad < 0.0:
            raise ValueError("start_tolerance_rad must be finite and non-negative")


def validate_trajectory(
    joint_names: Sequence[str],
    points: Sequence[object],
    current: np.ndarray,
    limits: TrajectorySafetyLimits,
) -> tuple[list[float], list[np.ndarray]]:
    names = list(joint_names)
    if len(names) != len(set(names)):
        raise ValueError("joint_names must not contain duplicates")
    if set(names) != set(ARM_JOINT_NAMES):
        raise ValueError(f"trajectory joints must match {list(ARM_JOINT_NAMES)}")
    order = np.array([ARM_JOINT_NAMES.index(name) for name in names])
    position_min = limits.position_min[order]
    position_max = limits.position_max[order]
    max_velocity = limits.max_velocity[order]
    max_acceleration = limits.max_acceleration[order]
    current = np.asarray(current, dtype=np.float64).reshape(-1)
    _validate_vector("current joint state", current, 6)
    _validate_position(current, position_min, position_max, "current joint state", names)
    if not points:
        raise ValueError("trajectory must include at least one point")

    times = [0.0]
    positions = [current.copy()]
    supplied_velocities: list[np.ndarray | None] = [np.zeros(6)]
    supplied_accelerations: list[np.ndarray | None] = [np.zeros(6)]
    previous_input_time: float | None = None

    for index, point in enumerate(points):
        point_time = _point_time(point)
        if point_time < 0.0:
            raise ValueError("trajectory time_from_start must be non-negative")
        if previous_input_time is not None and point_time <= previous_input_time:
            raise ValueError("trajectory time_from_start must be strictly increasing")
        previous_input_time = point_time

        point_positions = np.asarray(getattr(point, "positions", ()), dtype=np.float64).reshape(-1)
        _validate_vector(f"trajectory point {index} positions", point_positions, 6)
        _validate_position(
            point_positions,
            position_min,
            position_max,
            f"trajectory point {index}",
            names,
        )

        point_velocities = _optional_vector(
            point, "velocities", index, max_velocity, names
        )
        point_accelerations = _optional_vector(
            point, "accelerations", index, max_acceleration, names
        )

        if index == 0:
            start_delta = float(np.max(np.abs(point_positions - current)))
            if start_delta > limits.start_tolerance_rad:
                raise ValueError(
                    "first trajectory point is too far from current joint state "
                    f"(max delta {start_delta:.3f} rad)"
                )
            if point_time == 0.0:
                positions[0] = point_positions
                supplied_velocities[0] = (
                    point_velocities if point_velocities is not None else np.zeros(6)
                )
                supplied_accelerations[0] = (
                    point_accelerations
                    if point_accelerations is not None
                    else np.zeros(6)
                )
                continue

        times.append(point_time)
        positions.append(point_positions)
        supplied_velocities.append(point_velocities)
        supplied_accelerations.append(point_accelerations)

    if len(times) == 1:
        return times, positions

    segment_velocities: list[np.ndarray] = []
    for index in range(1, len(times)):
        duration = times[index] - times[index - 1]
        if duration <= 0.0:
            raise ValueError("trajectory time_from_start must be strictly increasing")
        velocity = (positions[index] - positions[index - 1]) / duration
        _validate_limit("velocity", velocity, max_velocity, index, names)
        segment_velocities.append(velocity)

    for index in range(1, len(segment_velocities)):
        transition_time = 0.5 * (
            (times[index] - times[index - 1]) + (times[index + 1] - times[index])
        )
        acceleration = (segment_velocities[index] - segment_velocities[index - 1]) / transition_time
        _validate_limit(
            "acceleration", acceleration, max_acceleration, index + 1, names
        )

    return times, positions


def interpolate_trajectory(
    times: Sequence[float], positions: Sequence[np.ndarray], elapsed_sec: float
) -> np.ndarray:
    if len(times) != len(positions) or not times:
        raise ValueError("trajectory samples are inconsistent")
    elapsed = float(elapsed_sec)
    if not math.isfinite(elapsed):
        raise ValueError("elapsed time must be finite")
    if elapsed <= times[0]:
        return np.asarray(positions[0], dtype=np.float64).copy()
    if elapsed >= times[-1]:
        return np.asarray(positions[-1], dtype=np.float64).copy()
    upper = int(np.searchsorted(np.asarray(times), elapsed, side="right"))
    lower = upper - 1
    duration = float(times[upper] - times[lower])
    ratio = (elapsed - float(times[lower])) / duration
    return np.asarray(positions[lower]) + ratio * (
        np.asarray(positions[upper]) - np.asarray(positions[lower])
    )


def _point_time(point: object) -> float:
    stamp = getattr(point, "time_from_start", None)
    if stamp is None:
        raise ValueError("trajectory point is missing time_from_start")
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if not math.isfinite(value):
        raise ValueError("trajectory time_from_start must be finite")
    return value


def _optional_vector(
    point: object,
    name: str,
    index: int,
    limit: np.ndarray,
    joint_names: Sequence[str],
) -> np.ndarray | None:
    values = getattr(point, name, ())
    if not values:
        return None
    vector = np.asarray(values, dtype=np.float64).reshape(-1)
    _validate_vector(f"trajectory point {index} {name}", vector, 6)
    _validate_limit(name.rstrip("s"), vector, limit, index, joint_names)
    return vector


def _validate_vector(label: str, vector: np.ndarray, size: int) -> None:
    if vector.shape != (size,):
        raise ValueError(f"{label} must contain {size} values")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} must contain only finite values")


def _validate_position(
    position: np.ndarray,
    position_min: np.ndarray,
    position_max: np.ndarray,
    label: str,
    joint_names: Sequence[str],
) -> None:
    bad = np.flatnonzero((position < position_min) | (position > position_max))
    if bad.size:
        index = int(bad[0])
        raise ValueError(
            f"{label} exceeds position limit for {joint_names[index]}: "
            f"{position[index]:.4f} not in "
            f"[{position_min[index]:.4f}, {position_max[index]:.4f}]"
        )


def _validate_limit(
    label: str,
    values: np.ndarray,
    limits: np.ndarray,
    point_index: int,
    joint_names: Sequence[str] = ARM_JOINT_NAMES,
) -> None:
    bad = np.flatnonzero(np.abs(values) > limits + 1e-9)
    if bad.size:
        index = int(bad[0])
        raise ValueError(
            f"trajectory {label} limit exceeded at point {point_index}, "
            f"{joint_names[index]}: {abs(values[index]):.4f} > {limits[index]:.4f}"
        )
