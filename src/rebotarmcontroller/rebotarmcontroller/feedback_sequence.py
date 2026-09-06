from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Generic, TypeVar


UINT64_MAX = (1 << 64) - 1
UINT64_HALF_RANGE = 1 << 63

StateT = TypeVar("StateT")


@dataclass(frozen=True)
class VerifiedFeedbackSample(Generic[StateT]):
    """One feedback frame whose receive sequence advanced after a request."""

    state: StateT
    sequence: int
    observed_at: float

    def __post_init__(self) -> None:
        validate_sequence(self.sequence)
        if not math.isfinite(self.observed_at):
            raise ValueError("feedback observation time must be finite")

    def age_sec(self, now: float) -> float:
        current = float(now)
        if not math.isfinite(current):
            raise ValueError("feedback age reference time must be finite")
        return max(current - self.observed_at, 0.0)

    def is_stale(self, now: float, timeout_sec: float) -> bool:
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("feedback stale timeout must be finite and positive")
        return self.age_sec(now) > timeout


def validate_sequence(value: int) -> int:
    sequence = int(value)
    if sequence != value or not 0 <= sequence <= UINT64_MAX:
        raise ValueError("feedback sequence must be an unsigned 64-bit integer")
    return sequence


def sequence_advanced(current: int, baseline: int) -> bool:
    """Return whether ``current`` is a newer uint64 receive sequence.

    The half-range rule makes wraparound ordering unambiguous.  MotorBridge
    reserves zero as its uninitialised value, so a transition back to zero is
    never accepted as a newly received frame.
    """

    current_value = validate_sequence(current)
    baseline_value = validate_sequence(baseline)
    if current_value == 0 and baseline_value != 0:
        return False
    delta = (current_value - baseline_value) & UINT64_MAX
    return 0 < delta < UINT64_HALF_RANGE
