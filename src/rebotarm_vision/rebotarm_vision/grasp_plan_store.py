from __future__ import annotations

from copy import deepcopy
import threading
import time
from typing import Callable

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan

from .freshness import FreshnessTracker
from .grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices


class GraspPlanStore:
    """Owns thread-safe grasp plan snapshots and their freshness metadata."""

    def __init__(self, monotonic: Callable[[], float] = time.monotonic) -> None:
        self._lock = threading.RLock()
        self._latest_plan: GraspPlan | None = None
        self._latest_candidates: GraspCandidateArray | None = None
        self._freshness = FreshnessTracker(monotonic)
        self._revision = 0

    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    def update_plan(self, plan: GraspPlan) -> None:
        with self._lock:
            if plan.valid:
                self._latest_plan = deepcopy(plan)
                self._freshness.touch("plan")
                self._revision += 1
                return
            self._latest_plan = None
            self._freshness.invalidate("plan")

    def update_candidates(self, candidates: GraspCandidateArray) -> None:
        with self._lock:
            if candidates.candidates:
                self._latest_candidates = deepcopy(candidates)
                self._freshness.touch("candidates")
                return
            self._latest_candidates = None
            self._freshness.invalidate("candidates")

    def current_plan(self, max_age_sec: float) -> GraspPlan | None:
        with self._lock:
            if self._latest_plan is None or not self._freshness.is_fresh(
                "plan", max_age_sec
            ):
                return None
            return deepcopy(self._latest_plan)

    def latest_plan(self) -> GraspPlan | None:
        with self._lock:
            if self._latest_plan is None:
                return None
            return deepcopy(self._latest_plan)

    def refreshed_plan_after(self, revision: int) -> tuple[int, GraspPlan] | None:
        with self._lock:
            if self._revision <= int(revision) or self._latest_plan is None:
                return None
            return self._revision, deepcopy(self._latest_plan)

    def candidate_attempts(
        self,
        *,
        plan_max_age_sec: float,
        candidates_max_age_sec: float,
        retry_config: RetryPolicyConfig,
    ) -> list[tuple[int, GraspPlan]]:
        with self._lock:
            if self._latest_plan is None or not self._freshness.is_fresh(
                "plan", plan_max_age_sec
            ):
                self._invalidate_all_locked()
                return []
            attempts = [(-1, deepcopy(self._latest_plan))]
            candidates = self._latest_candidates
            if (
                candidates is None
                or not candidates.candidates
                or not self._freshness.is_fresh(
                    "candidates", candidates_max_age_sec
                )
            ):
                return attempts
            indices = ordered_candidate_indices(
                candidate_count=len(candidates.candidates),
                best_index=int(candidates.best_index),
                failed_indices=set(),
                config=retry_config,
            )
            for index in indices:
                if index == int(candidates.best_index):
                    continue
                attempts.append((index, self._plan_from_candidate(candidates, index)))
            return attempts

    def invalidate_all(self) -> None:
        with self._lock:
            self._invalidate_all_locked()

    def _invalidate_all_locked(self) -> None:
        self._latest_plan = None
        self._latest_candidates = None
        self._freshness.invalidate("plan")
        self._freshness.invalidate("candidates")

    @staticmethod
    def _plan_from_candidate(
        candidates: GraspCandidateArray, index: int
    ) -> GraspPlan:
        candidate = candidates.candidates[int(index)]
        plan = GraspPlan()
        plan.header = candidates.header
        plan.candidate = deepcopy(candidate)
        plan.pregrasp_pose = deepcopy(candidate.pose)
        plan.grasp_pose = deepcopy(candidate.pose)
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.source = "visual_grasp_executor_retry"
        plan.reason = ""
        return plan
