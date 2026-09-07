from __future__ import annotations

import pytest

from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray, GraspPlan
from rebotarm_vision.grasp_plan_store import GraspPlanStore
from rebotarm_vision.grasp_retry_policy import RetryPolicyConfig


def _valid_plan(source: str = "filtered") -> GraspPlan:
    plan = GraspPlan()
    plan.valid = True
    plan.source = source
    plan.candidate.class_name = "primary"
    return plan


def _candidates(best_index: int = 1) -> GraspCandidateArray:
    candidates = GraspCandidateArray()
    candidates.best_index = best_index
    for index in range(3):
        candidate = GraspCandidate()
        candidate.class_name = f"candidate-{index}"
        candidate.jaw_width = 0.02 + index * 0.01
        candidate.pose.position.x = float(index)
        candidates.candidates.append(candidate)
    return candidates


def test_current_plan_is_fresh_and_returns_a_defensive_snapshot() -> None:
    now = [10.0]
    store = GraspPlanStore(monotonic=lambda: now[0])
    store.update_plan(_valid_plan())

    snapshot = store.current_plan(1.0)
    assert snapshot is not None
    assert snapshot.source == "filtered"
    snapshot.source = "mutated"

    assert store.current_plan(1.0).source == "filtered"
    now[0] = 11.01
    assert store.current_plan(1.0) is None


def test_invalid_plan_clears_snapshot_without_advancing_revision() -> None:
    store = GraspPlanStore()
    store.update_plan(_valid_plan())
    assert store.revision == 1

    store.update_plan(GraspPlan())

    assert store.revision == 1
    assert store.latest_plan() is None
    assert store.refreshed_plan_after(0) is None


def test_refreshed_plan_after_returns_revision_and_defensive_snapshot() -> None:
    store = GraspPlanStore()
    store.update_plan(_valid_plan("first"))
    store.update_plan(_valid_plan("second"))

    refreshed = store.refreshed_plan_after(1)
    assert refreshed is not None
    revision, plan = refreshed
    assert revision == 2
    assert plan.source == "second"
    plan.source = "mutated"

    assert store.refreshed_plan_after(1)[1].source == "second"
    assert store.refreshed_plan_after(2) is None


def test_candidate_attempts_keep_primary_plan_and_add_fresh_retries() -> None:
    store = GraspPlanStore()
    store.update_plan(_valid_plan())
    store.update_candidates(_candidates())

    attempts = store.candidate_attempts(
        plan_max_age_sec=1.0,
        candidates_max_age_sec=1.0,
        retry_config=RetryPolicyConfig(enabled=True, max_attempts=3),
    )

    assert [index for index, _plan in attempts] == [-1, 0, 2]
    assert [plan.candidate.class_name for _index, plan in attempts] == [
        "primary",
        "candidate-0",
        "candidate-2",
    ]
    assert attempts[1][1].source == "visual_grasp_executor_retry"
    assert attempts[1][1].pregrasp_pose.position.x == pytest.approx(0.0)
    assert attempts[2][1].jaw_width == pytest.approx(0.04)


def test_candidate_attempts_ignore_stale_candidates() -> None:
    now = [5.0]
    store = GraspPlanStore(monotonic=lambda: now[0])
    store.update_plan(_valid_plan())
    store.update_candidates(_candidates())
    now[0] = 6.1

    attempts = store.candidate_attempts(
        plan_max_age_sec=2.0,
        candidates_max_age_sec=1.0,
        retry_config=RetryPolicyConfig(enabled=True, max_attempts=3),
    )

    assert [index for index, _plan in attempts] == [-1]


def test_invalidate_all_clears_plan_and_candidate_attempts() -> None:
    store = GraspPlanStore()
    store.update_plan(_valid_plan())
    store.update_candidates(_candidates())

    store.invalidate_all()

    assert store.latest_plan() is None
    assert store.candidate_attempts(
        plan_max_age_sec=1.0,
        candidates_max_age_sec=1.0,
        retry_config=RetryPolicyConfig(enabled=True, max_attempts=3),
    ) == []


def test_candidate_attempts_atomically_clear_stale_plan_and_candidates() -> None:
    now = [5.0]
    store = GraspPlanStore(monotonic=lambda: now[0])
    store.update_plan(_valid_plan())
    store.update_candidates(_candidates())
    now[0] = 6.1

    assert store.candidate_attempts(
        plan_max_age_sec=1.0,
        candidates_max_age_sec=2.0,
        retry_config=RetryPolicyConfig(enabled=True, max_attempts=3),
    ) == []
    assert store.latest_plan() is None
