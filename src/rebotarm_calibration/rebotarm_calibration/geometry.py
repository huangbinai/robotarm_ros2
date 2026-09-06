from __future__ import annotations

from collections.abc import Sequence
import math


def quaternion_to_rotation_matrix(
    rotation_xyzw: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    if len(rotation_xyzw) != 4:
        raise ValueError("rotation_xyzw must contain exactly 4 values")
    qx, qy, qz, qw = (float(value) for value in rotation_xyzw)
    if not all(math.isfinite(value) for value in (qx, qy, qz, qw)):
        raise ValueError("rotation_xyzw must contain only finite values")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("rotation_xyzw must have non-zero length")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    return (
        (
            1.0 - 2.0 * qy * qy - 2.0 * qz * qz,
            2.0 * qx * qy - 2.0 * qz * qw,
            2.0 * qx * qz + 2.0 * qy * qw,
        ),
        (
            2.0 * qx * qy + 2.0 * qz * qw,
            1.0 - 2.0 * qx * qx - 2.0 * qz * qz,
            2.0 * qy * qz - 2.0 * qx * qw,
        ),
        (
            2.0 * qx * qz - 2.0 * qy * qw,
            2.0 * qy * qz + 2.0 * qx * qw,
            1.0 - 2.0 * qx * qx - 2.0 * qy * qy,
        ),
    )
