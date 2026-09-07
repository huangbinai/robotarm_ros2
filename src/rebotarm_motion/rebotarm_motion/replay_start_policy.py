from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .teach_sample_processing import RetimedTeachPoint


class ReplayStartBand(str, Enum):
    DIRECT = "direct"
    ALIGN = "align"
    MOVEIT_ALIGN = "moveit_align"
    REJECT = "reject"


@dataclass(frozen=True)
class ReplayStartDecision:
    band: ReplayStartBand
    max_error: float
    per_joint_error: tuple[float, ...]
    allow_replay: bool
    allow_auto_align: bool
    message: str


def classify_replay_start(
    *,
    current_positions: tuple[float, ...],
    start_positions: tuple[float, ...],
    direct_threshold: float,
    align_threshold: float,
) -> ReplayStartDecision:
    if len(current_positions) != len(start_positions):
        return ReplayStartDecision(
            band=ReplayStartBand.REJECT,
            max_error=float("inf"),
            per_joint_error=(),
            allow_replay=False,
            allow_auto_align=False,
            message="current and start joint vectors have different lengths",
        )
    errors = tuple(
        abs(float(current) - float(start))
        for current, start in zip(current_positions, start_positions)
    )
    max_error = max(errors, default=0.0)
    if max_error < float(direct_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.DIRECT,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=False,
            message="current pose is close enough to replay start",
        )
    if max_error < float(align_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.ALIGN,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=True,
            message="small return_to_start alignment required",
        )
    return ReplayStartDecision(
        band=ReplayStartBand.REJECT,
        max_error=max_error,
        per_joint_error=errors,
        allow_replay=False,
        allow_auto_align=False,
        message="start error too large; manually drag the arm near the recording start",
    )


def interpolate_joint_positions(
    *,
    current_positions: tuple[float, ...],
    target_positions: tuple[float, ...],
    steps: int,
) -> list[tuple[float, ...]]:
    if len(current_positions) != len(target_positions):
        raise ValueError("current and target joint vectors have different lengths")
    count = max(int(steps), 2)
    points: list[tuple[float, ...]] = []
    for index in range(count):
        alpha = float(index) / float(count - 1)
        points.append(
            tuple(
                float(current + (target - current) * alpha)
                for current, target in zip(current_positions, target_positions)
            )
        )
    return points


def build_replay_start_soft_points(
    *,
    current_positions: tuple[float, ...],
    first_positions: tuple[float, ...],
    start_band: str,
    start_hold_sec: float = 0.8,
    soft_start_duration: float = 1.0,
    soft_start_steps: int = 30,
    align_duration: float = 3.0,
    align_steps: int = 30,
    first_hold_sec: float = 0.3,
) -> list[RetimedTeachPoint]:
    if len(current_positions) != len(first_positions):
        raise ValueError("current and first joint vectors have different lengths")
    elapsed = 0.0
    points: list[RetimedTeachPoint] = []
    hold = max(float(start_hold_sec), 0.0)
    if hold > 0.0:
        elapsed += hold
        points.append(
            RetimedTeachPoint(
                time_from_start=elapsed,
                positions=tuple(float(value) for value in current_positions),
                source_sample=-1,
            )
        )
    band = str(start_band or "").strip().lower()
    if band == ReplayStartBand.ALIGN.value:
        duration = max(float(align_duration), 0.0)
        steps = int(align_steps)
    else:
        duration = max(float(soft_start_duration), 0.0)
        steps = int(soft_start_steps)
    align_points = interpolate_joint_positions(
        current_positions=tuple(float(value) for value in current_positions),
        target_positions=tuple(float(value) for value in first_positions),
        steps=steps,
    )
    for index, positions in enumerate(align_points):
        ratio = float(index) / float(max(len(align_points) - 1, 1))
        timestamp = elapsed + duration * ratio
        if points and timestamp <= points[-1].time_from_start:
            continue
        points.append(
            RetimedTeachPoint(
                time_from_start=timestamp,
                positions=tuple(float(value) for value in positions),
                source_sample=-1,
            )
        )
    elapsed += duration
    first_hold = max(float(first_hold_sec), 0.0)
    if first_hold > 0.0:
        elapsed += first_hold
        if not points or elapsed > points[-1].time_from_start:
            points.append(
                RetimedTeachPoint(
                    time_from_start=elapsed,
                    positions=tuple(float(value) for value in first_positions),
                    source_sample=-1,
                )
            )
    return points


def compute_auto_align_duration(
    max_error_rad: float | None,
    *,
    target_speed_rad_s: float = 0.15,
    min_duration_sec: float = 3.0,
    max_duration_sec: float = 10.0,
) -> float:
    try:
        error = abs(float(max_error_rad))
    except (TypeError, ValueError):
        error = 0.0
    speed = max(float(target_speed_rad_s), 0.01)
    duration = error / speed if error > 0.0 else float(min_duration_sec)
    return min(max(duration, float(min_duration_sec)), float(max_duration_sec))
