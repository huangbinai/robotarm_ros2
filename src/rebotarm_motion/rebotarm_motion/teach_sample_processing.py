from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Protocol, TypeVar


class TeachSampleLike(Protocol):
    stamp: float
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    motor_status: dict[str, int]
    arm_state: str


SampleT = TypeVar("SampleT", bound=TeachSampleLike)


@dataclass(frozen=True)
class RetimedTeachPoint:
    time_from_start: float
    positions: tuple[float, ...]
    source_sample: int
    velocities: tuple[float, ...] = ()


def _positive_finite(value: float, name: str, *, minimum: float) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return max(number, minimum)


def _velocity_limits(max_velocity_rad_s, joint_names: tuple[str, ...]) -> tuple[float, ...]:
    if isinstance(max_velocity_rad_s, dict):
        fallback = _positive_finite(max_velocity_rad_s.get("*", 2.0), "max_velocity_rad_s[*]", minimum=0.01)
        values = [max_velocity_rad_s.get(name, fallback) for name in joint_names]
    elif isinstance(max_velocity_rad_s, (list, tuple)):
        if len(max_velocity_rad_s) != len(joint_names):
            raise ValueError("max_velocity_rad_s length must match joint_names")
        values = list(max_velocity_rad_s)
    else:
        values = [max_velocity_rad_s] * len(joint_names)
    return tuple(
        _positive_finite(value, f"max_velocity_rad_s[{index}]", minimum=0.01)
        for index, value in enumerate(values)
    )


def _validate_samples(samples: list[TeachSampleLike]) -> tuple[str, ...]:
    if not samples:
        return ()
    expected_joints = samples[0].joint_names
    expected_len = len(expected_joints)
    for index, sample in enumerate(samples):
        if sample.joint_names != expected_joints:
            raise ValueError(f"joint_names mismatch at sample {index}")
        if len(sample.positions) != expected_len:
            raise ValueError(f"positions length mismatch at sample {index}")
        values = (float(sample.stamp), *(float(value) for value in sample.positions))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"sample {index} must contain only finite timestamps and positions")
    return expected_joints


def retime_teach_samples(
    samples: list[TeachSampleLike],
    *,
    replay_speed: float,
    max_velocity_rad_s,
    max_acceleration_rad_s2: float = 999.0,
    max_jerk_rad_s3: float = 999.0,
    initial_delay_sec: float = 0.2,
    boundary_zero_velocity: bool = True,
) -> list[RetimedTeachPoint]:
    if not samples:
        return []
    expected_joints = _validate_samples(samples)
    speed = _positive_finite(replay_speed, "replay_speed", minimum=0.01)
    velocity_limits = _velocity_limits(max_velocity_rad_s, expected_joints)
    acceleration_limit = _positive_finite(max_acceleration_rad_s2, "max_acceleration_rad_s2", minimum=0.01)
    jerk_limit = _positive_finite(max_jerk_rad_s3, "max_jerk_rad_s3", minimum=0.01)
    delay = float(initial_delay_sec)
    if not math.isfinite(delay) or delay < 0.0:
        raise ValueError("initial_delay_sec must be finite and non-negative")
    zero_velocity = tuple(0.0 for _ in samples[0].positions)
    retimed = [RetimedTeachPoint(delay, tuple(float(v) for v in samples[0].positions), 0, zero_velocity)]
    previous = samples[0]
    previous_velocity = zero_velocity
    previous_acceleration = zero_velocity
    elapsed = delay
    for index, sample in enumerate(samples[1:], start=1):
        recorded_dt = max(0.0, float(sample.stamp) - float(previous.stamp)) / speed
        min_velocity_dt = max(
            (abs(float(current) - float(last)) / limit for current, last, limit in zip(sample.positions, previous.positions, velocity_limits)),
            default=0.0,
        )
        dt = max(recorded_dt, min_velocity_dt, 0.001)
        delta = tuple(float(current) - float(last) for current, last in zip(sample.positions, previous.positions))
        for _ in range(24):
            velocity = tuple(value / dt for value in delta)
            acceleration = tuple((current - last) / dt for current, last in zip(velocity, previous_velocity))
            max_acceleration = max((abs(value) for value in acceleration), default=0.0)
            max_jerk = max((abs(current - last) / dt for current, last in zip(acceleration, previous_acceleration)), default=0.0)
            if max_acceleration <= acceleration_limit + 1e-12 and max_jerk <= jerk_limit + 1e-12:
                break
            dt *= 1.25
        velocity = tuple(value / dt for value in delta)
        acceleration = tuple((current - last) / dt for current, last in zip(velocity, previous_velocity))
        elapsed += dt
        retimed.append(RetimedTeachPoint(elapsed, tuple(float(v) for v in sample.positions), index, velocity))
        previous = sample
        previous_velocity = velocity
        previous_acceleration = acceleration
    if boundary_zero_velocity and retimed:
        retimed[0] = replace(retimed[0], velocities=zero_velocity)
        if len(retimed) > 1:
            previous_point, last_point = retimed[-2], retimed[-1]
            prior_acceleration = zero_velocity
            if len(retimed) > 2:
                before_previous = retimed[-3]
                prior_dt = max(previous_point.time_from_start - before_previous.time_from_start, 1e-9)
                prior_acceleration = tuple((current - last) / prior_dt for current, last in zip(previous_point.velocities, before_previous.velocities))
            adjusted_dt = max(last_point.time_from_start - previous_point.time_from_start, 0.001)
            for _ in range(24):
                final_acceleration = tuple(-value / adjusted_dt for value in previous_point.velocities)
                max_acceleration = max((abs(value) for value in final_acceleration), default=0.0)
                max_jerk = max((abs(current - last) / adjusted_dt for current, last in zip(final_acceleration, prior_acceleration)), default=0.0)
                if max_acceleration <= acceleration_limit + 1e-12 and max_jerk <= jerk_limit + 1e-12:
                    break
                adjusted_dt *= 1.25
            retimed[-1] = replace(
                last_point,
                time_from_start=previous_point.time_from_start + adjusted_dt,
                velocities=tuple(0.0 for _ in last_point.positions),
            )
    return retimed


def lowpass_filter_teach_samples(
    samples: list[SampleT], *, sample_rate_hz: float, cutoff_hz: float, preserve_start_end: bool = True
) -> list[SampleT]:
    if len(samples) <= 2:
        return list(samples)
    rate = _positive_finite(sample_rate_hz, "sample_rate_hz", minimum=1.0)
    cutoff = _positive_finite(cutoff_hz, "cutoff_hz", minimum=0.01)
    alpha = (1.0 / rate) / (1.0 / (2.0 * math.pi * cutoff) + 1.0 / rate)
    positions = [tuple(float(value) for value in samples[0].positions)]
    for sample in samples[1:]:
        positions.append(tuple(last + alpha * (float(current) - last) for current, last in zip(sample.positions, positions[-1])))
    backward = [positions[-1]]
    for current_positions in reversed(positions[:-1]):
        backward.append(tuple(last + alpha * (current - last) for current, last in zip(current_positions, backward[-1])))
    positions = list(reversed(backward))
    if preserve_start_end:
        positions[0], positions[-1] = samples[0].positions, samples[-1].positions
    return [replace(sample, positions=tuple(float(value) for value in values)) for sample, values in zip(samples, positions)]


def smooth_teach_samples(samples: list[SampleT], *, window: int = 5, preserve_start_end: bool = True) -> list[SampleT]:
    if len(samples) <= 2:
        return list(samples)
    width = max(int(window), 1)
    width += int(width % 2 == 0)
    radius = width // 2
    result: list[SampleT] = []
    for index, sample in enumerate(samples):
        if preserve_start_end and index in (0, len(samples) - 1):
            result.append(sample)
            continue
        segment = samples[max(0, index - radius):min(len(samples), index + radius + 1)]
        positions = tuple(sum(float(item.positions[joint]) for item in segment) / len(segment) for joint in range(len(sample.positions)))
        result.append(replace(sample, positions=positions))
    return result


def resample_teach_samples(samples: list[SampleT], *, rate_hz: float = 50.0) -> list[SampleT]:
    if len(samples) <= 1:
        return list(samples)
    _validate_samples(samples)
    rate = _positive_finite(rate_hz, "rate_hz", minimum=1.0)
    start_stamp, end_stamp = float(samples[0].stamp), float(samples[-1].stamp)
    duration = end_stamp - start_stamp
    if duration <= 0.0:
        return list(samples)
    period = 1.0 / rate
    result: list[SampleT] = []
    source_index = 0
    count = max(int(round(duration / period)), 1)
    for output_index in range(count + 1):
        stamp = start_stamp + min(output_index * period, duration)
        while source_index + 1 < len(samples) and float(samples[source_index + 1].stamp) < stamp:
            source_index += 1
        previous = samples[source_index]
        following = samples[min(source_index + 1, len(samples) - 1)]
        span = max(float(following.stamp) - float(previous.stamp), 1e-9)
        alpha = 0.0 if following is previous else min(max((stamp - float(previous.stamp)) / span, 0.0), 1.0)
        positions = tuple(float(a) + (float(b) - float(a)) * alpha for a, b in zip(previous.positions, following.positions))
        result.append(replace(previous, stamp=stamp, positions=positions, velocities=(), efforts=()))
    result[0], result[-1] = samples[0], samples[-1]
    return result
