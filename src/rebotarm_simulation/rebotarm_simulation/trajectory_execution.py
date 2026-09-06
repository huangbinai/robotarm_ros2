"""ROS-independent trajectory execution policy for the MuJoCo adapter.

This module owns validation, settling, feedback pacing, and the thread-safe
goal lifecycle.  Keeping these policies independent from ``rclpy`` makes the
behavior reusable and directly unit-testable on development hosts without ROS.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Sequence

from .trajectory_sampler import ARM_JOINT_NAMES, NamedTrajectoryPoint, TrajectorySampler


DEFAULT_MAX_TRAJECTORY_POINTS = 10_000
DEFAULT_MAX_TRAJECTORY_DURATION_SEC = 300.0


@dataclass(frozen=True)
class GoalSettlingPolicy:
    position_tolerance: float = 0.02
    velocity_tolerance: float = 0.05
    time_tolerance_sec: float = 5.0

    def __post_init__(self) -> None:
        values = (
            self.position_tolerance,
            self.velocity_tolerance,
            self.time_tolerance_sec,
        )
        if any(
            not math.isfinite(float(value)) or float(value) <= 0.0
            for value in values
        ):
            raise ValueError("goal tolerances must be positive finite values")

    def evaluate(self, desired, actual, velocities, settle_elapsed: float) -> str:
        vectors = (tuple(desired), tuple(actual), tuple(velocities))
        if any(len(vector) != len(ARM_JOINT_NAMES) for vector in vectors):
            raise ValueError("goal state must contain six arm values")
        numeric = tuple(
            tuple(float(value) for value in vector) for vector in vectors
        )
        elapsed = float(settle_elapsed)
        if any(
            not math.isfinite(value) for vector in numeric for value in vector
        ) or not math.isfinite(elapsed):
            raise ValueError("goal state must be finite")
        if elapsed < 0.0:
            raise ValueError("settling time must be non-negative")
        position_error = max(
            abs(target - reached)
            for target, reached in zip(numeric[0], numeric[1])
        )
        max_velocity = max(abs(value) for value in numeric[2])
        if (
            position_error <= self.position_tolerance
            and max_velocity <= self.velocity_tolerance
        ):
            return "succeeded"
        if elapsed >= self.time_tolerance_sec:
            return "timed_out"
        return "settling"


def duration_to_seconds(duration: Any) -> float:
    try:
        value = float(duration.sec) + float(duration.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("trajectory time is invalid") from exc
    if not math.isfinite(value):
        raise ValueError("trajectory time must be finite")
    return value


def trajectory_to_sampler(
    trajectory: Any,
    *,
    initial_positions: Sequence[float],
    max_points: int = DEFAULT_MAX_TRAJECTORY_POINTS,
    max_duration_sec: float = DEFAULT_MAX_TRAJECTORY_DURATION_SEC,
) -> TrajectorySampler:
    """Validate an untrusted JointTrajectory-like object and build a sampler."""
    if isinstance(max_points, bool) or int(max_points) <= 0:
        raise ValueError("max points must be positive")
    duration_cap = float(max_duration_sec)
    if not math.isfinite(duration_cap) or duration_cap <= 0.0:
        raise ValueError("max duration must be finite and positive")
    try:
        names = tuple(trajectory.joint_names)
        raw_points = tuple(trajectory.points)
    except (AttributeError, TypeError) as exc:
        raise ValueError("trajectory structure is invalid") from exc
    if len(raw_points) > int(max_points):
        raise ValueError("trajectory contains too many points")

    points = tuple(
        NamedTrajectoryPoint(
            duration_to_seconds(point.time_from_start), point.positions
        )
        for point in raw_points
    )
    sampler = TrajectorySampler(
        names, points, initial_positions=initial_positions
    )
    if sampler.duration > duration_cap:
        raise ValueError("trajectory duration exceeds configured limit")
    return sampler


def seconds_to_stamp_parts(simulation_time: Any) -> tuple[int, int]:
    try:
        value = float(simulation_time)
    except (TypeError, ValueError) as exc:
        raise ValueError("simulation time must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    seconds = math.floor(value)
    nanoseconds = int(round((value - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return int(seconds), nanoseconds


class MonotonicStamp:
    def __init__(self) -> None:
        self._nanoseconds = 0
        self._lock = threading.Lock()

    def update(self, simulation_time: Any) -> tuple[int, int]:
        seconds, nanoseconds = seconds_to_stamp_parts(simulation_time)
        candidate = seconds * 1_000_000_000 + nanoseconds
        with self._lock:
            self._nanoseconds = max(self._nanoseconds, candidate)
            return divmod(self._nanoseconds, 1_000_000_000)


class FeedbackRateLimiter:
    def __init__(self, rate_hz: Any) -> None:
        try:
            rate = float(rate_hz)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "feedback rate must be finite and in (0, 200]"
            ) from exc
        if not math.isfinite(rate) or rate <= 0.0 or rate > 200.0:
            raise ValueError("feedback rate must be finite and in (0, 200]")
        self.rate_hz = rate
        self._period = 1.0 / rate
        self._last_publish: float | None = None

    def should_publish(self, monotonic_time: Any, *, final: bool = False) -> bool:
        now = float(monotonic_time)
        if not math.isfinite(now):
            raise ValueError("feedback clock must be finite")
        due = (
            self._last_publish is None
            or now - self._last_publish >= self._period
        )
        if final or due:
            self._last_publish = now
            return True
        return False


class ActiveTrajectory:
    """Thread-safe single-goal admission and cooperative cancellation gate."""

    def __init__(self, lock: threading.RLock | None = None) -> None:
        self._lock = lock or threading.RLock()
        self._token: object | None = None
        self._cancel_requested = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._token is not None

    @property
    def cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    @property
    def token(self) -> object | None:
        with self._lock:
            return self._token

    def try_start(self, token: object) -> bool:
        with self._lock:
            if self._token is not None:
                return False
            self._token = token
            self._cancel_requested = False
            return True

    def stop(self) -> bool:
        with self._lock:
            if self._token is None:
                return False
            self._cancel_requested = True
            return True

    def finish(self, token: object) -> None:
        with self._lock:
            if self._token is token:
                self._token = None
                self._cancel_requested = False


class GateOutcome(Enum):
    APPLIED = auto()
    ACTION_CANCEL = auto()
    SERVICE_STOP = auto()
    INACTIVE = auto()
    SUCCEEDED = auto()


def terminal_disposition(
    outcome: GateOutcome, action_cancel_requested: bool
) -> str:
    if outcome is GateOutcome.ACTION_CANCEL and bool(action_cancel_requested):
        return "canceled"
    return "aborted"


class TrajectoryCommandGate:
    """Atomically arbitrate trajectory commands and stop/hold operations."""

    def __init__(self, active: ActiveTrajectory) -> None:
        self._active = active

    def apply_if_active(self, token, cancel_requested, apply, hold) -> bool:
        return (
            self.apply_with_reason(token, cancel_requested, apply, hold)
            is GateOutcome.APPLIED
        )

    def apply_with_reason(
        self, token, action_cancel_requested, apply, hold
    ) -> GateOutcome:
        with self._active._lock:
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            apply()
            return GateOutcome.APPLIED

    def stop_and_hold(self, hold) -> bool:
        with self._active._lock:
            stopped = self._active._token is not None
            if stopped:
                self._active._cancel_requested = True
                hold()
            return stopped

    def complete_if_active(
        self, token, cancel_requested, hold, succeed
    ) -> bool:
        return (
            self.complete_with_reason(
                token, cancel_requested, hold, succeed
            )
            is GateOutcome.SUCCEEDED
        )

    def complete_with_reason(
        self, token, action_cancel_requested, hold, succeed
    ) -> GateOutcome:
        """Linearize cancellation versus the terminal success transition."""
        with self._active._lock:
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            succeed()
            self._active._token = None
            self._active._cancel_requested = False
            return GateOutcome.SUCCEEDED


class ExecutionLifecycle:
    """Best-effort failure cleanup that never leaks an admitted goal token."""

    def __init__(
        self, active: ActiveTrajectory, gate: TrajectoryCommandGate
    ) -> None:
        self._active = active
        self._gate = gate

    def fail(self, token, hold, abort) -> None:
        try:
            self._gate.stop_and_hold(hold)
        except Exception:
            pass
        try:
            abort()
        finally:
            self._active.finish(token)
