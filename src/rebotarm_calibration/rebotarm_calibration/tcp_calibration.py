from __future__ import annotations

from collections.abc import Iterable, Sequence
import math

from .geometry import quaternion_to_rotation_matrix

Vector3 = tuple[float, float, float]


def _vector3(values: Sequence[float], name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    result = (float(values[0]), float(values[1]), float(values[2]))
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def estimate_sample_offset(
    *,
    end_link_position: Sequence[float],
    end_link_orientation_xyzw: Sequence[float],
    tcp_reference_position: Sequence[float],
) -> Vector3:
    end_position = _vector3(end_link_position, "end_link_position")
    reference_position = _vector3(tcp_reference_position, "tcp_reference_position")
    rotation = quaternion_to_rotation_matrix(end_link_orientation_xyzw)
    delta_base = tuple(reference - end for reference, end in zip(reference_position, end_position))
    return (
        rotation[0][0] * delta_base[0] + rotation[1][0] * delta_base[1] + rotation[2][0] * delta_base[2],
        rotation[0][1] * delta_base[0] + rotation[1][1] * delta_base[1] + rotation[2][1] * delta_base[2],
        rotation[0][2] * delta_base[0] + rotation[1][2] * delta_base[1] + rotation[2][2] * delta_base[2],
    )


def average_offsets(offsets: Iterable[Sequence[float]]) -> Vector3:
    samples = [_vector3(offset, "offset") for offset in offsets]
    if not samples:
        raise ValueError("at least one offset sample is required")
    count = float(len(samples))
    return tuple(sum(sample[index] for sample in samples) / count for index in range(3))  # type: ignore[return-value]


def format_tcp_offset_yaml(offset: Sequence[float]) -> str:
    ox, oy, oz = _vector3(offset, "offset")
    return f"tcp_offset_xyz: [{ox:.6f}, {oy:.6f}, {oz:.6f}]"
