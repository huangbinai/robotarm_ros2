from __future__ import annotations

import pytest

from rebotarm_motion.replay_start_policy import (
    ReplayStartBand,
    build_replay_start_soft_points,
    classify_replay_start,
    compute_auto_align_duration,
    interpolate_joint_positions,
)


def test_replay_start_policy_classifies_direct_align_and_reject() -> None:
    direct = classify_replay_start(
        current_positions=(0.0, 0.0),
        start_positions=(0.01, 0.02),
        direct_threshold=0.05,
        align_threshold=0.5,
    )
    align = classify_replay_start(
        current_positions=(0.0, 0.0),
        start_positions=(0.1, 0.2),
        direct_threshold=0.05,
        align_threshold=0.5,
    )
    rejected = classify_replay_start(
        current_positions=(0.0, 0.0),
        start_positions=(0.1, 0.6),
        direct_threshold=0.05,
        align_threshold=0.5,
    )

    assert direct.band is ReplayStartBand.DIRECT
    assert align.band is ReplayStartBand.ALIGN
    assert rejected.band is ReplayStartBand.REJECT


def test_replay_start_policy_rejects_mismatched_vectors() -> None:
    decision = classify_replay_start(
        current_positions=(0.0,),
        start_positions=(0.0, 0.0),
        direct_threshold=0.05,
        align_threshold=0.5,
    )

    assert decision.band is ReplayStartBand.REJECT
    assert decision.max_error == float("inf")
    assert decision.per_joint_error == ()
    with pytest.raises(ValueError, match="different lengths"):
        interpolate_joint_positions(
            current_positions=(0.0,),
            target_positions=(0.0, 0.0),
            steps=3,
        )


def test_replay_start_soft_points_preserve_holds_and_alignment_duration() -> None:
    points = build_replay_start_soft_points(
        current_positions=(0.0, 0.0),
        first_positions=(1.0, 2.0),
        start_band=ReplayStartBand.ALIGN.value,
        start_hold_sec=0.8,
        align_duration=3.0,
        align_steps=3,
        first_hold_sec=0.3,
    )

    assert [point.time_from_start for point in points] == pytest.approx(
        [0.8, 2.3, 3.8, 4.1]
    )
    assert points[-1].positions == (1.0, 2.0)


def test_auto_align_duration_preserves_bounds() -> None:
    assert compute_auto_align_duration(0.02) == pytest.approx(3.0)
    assert compute_auto_align_duration(0.9) == pytest.approx(6.0)
    assert compute_auto_align_duration(3.0) == pytest.approx(10.0)
