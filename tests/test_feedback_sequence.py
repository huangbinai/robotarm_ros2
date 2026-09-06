from __future__ import annotations

import pytest

from rebotarmcontroller.feedback_sequence import (
    UINT64_HALF_RANGE,
    UINT64_MAX,
    VerifiedFeedbackSample,
    sequence_advanced,
    validate_sequence,
)


def test_same_sequence_is_not_fresh() -> None:
    assert sequence_advanced(41, 41) is False


def test_normal_increment_is_fresh() -> None:
    assert sequence_advanced(42, 41) is True


def test_uint64_wraparound_is_fresh_without_accepting_zero() -> None:
    assert sequence_advanced(0, UINT64_MAX) is False
    assert sequence_advanced(1, UINT64_MAX) is True


def test_ambiguous_or_backward_half_range_is_rejected() -> None:
    assert sequence_advanced(UINT64_HALF_RANGE, 0) is False
    assert sequence_advanced(2, 3) is False


@pytest.mark.parametrize("value", [-1, UINT64_MAX + 1, 1.5])
def test_sequence_range_and_type_are_validated(value) -> None:
    with pytest.raises(ValueError, match="unsigned 64-bit"):
        validate_sequence(value)


def test_verified_sample_reports_age_and_staleness() -> None:
    sample = VerifiedFeedbackSample(state=object(), sequence=9, observed_at=10.0)
    assert sample.age_sec(10.08) == pytest.approx(0.08)
    assert sample.is_stale(10.08, 0.1) is False
    assert sample.is_stale(10.11, 0.1) is True
