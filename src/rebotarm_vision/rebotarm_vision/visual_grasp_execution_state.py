from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any


@dataclass
class VisualGraspExecutionState:
    """Mutable state for one visual-grasp executor and its current run."""

    last_gripper_reached_position: float | None = None
    last_grasp_contact_detected: bool = False
    last_grasp_closure_distance_m: float = 0.0
    retry_retreat_stage: Any | None = None
    run_counter: int = 0
    current_run_id: int = 0
    current_attempt_index: int = 0
    current_candidate_index: int = -1
    current_attempt_plan: Any | None = None
    running: bool = False
    failure_recovery_start_pose: Any | None = None

    def can_start_run(self) -> bool:
        return not self.running

    def reserve_run(self) -> bool:
        if self.running:
            return False
        self.running = True
        return True

    def begin_run(self, recovery_start_pose: Any | None) -> None:
        self.failure_recovery_start_pose = recovery_start_pose
        self.run_counter += 1
        self.current_run_id = self.run_counter

    def begin_attempt(
        self,
        *,
        attempt_index: int,
        candidate_index: int,
        plan: Any,
    ) -> None:
        self.current_attempt_index = int(attempt_index)
        self.current_candidate_index = int(candidate_index)
        self.current_attempt_plan = deepcopy(plan)
        self.last_grasp_contact_detected = False
        self.last_grasp_closure_distance_m = 0.0
        self.retry_retreat_stage = None

    def remember_retry_retreat(self, stage: Any) -> None:
        self.retry_retreat_stage = stage

    def record_open_position(self, reached_position: float) -> None:
        self.last_gripper_reached_position = float(reached_position)

    def record_grasp_result(
        self,
        *,
        contact_detected: bool,
        reached_position: float,
    ) -> None:
        self.last_grasp_contact_detected = bool(contact_detected)
        self.last_grasp_closure_distance_m = max(
            0.0,
            float(self.last_gripper_reached_position or 0.0)
            - float(reached_position),
        )

    def clear_run_context(self) -> None:
        self.current_attempt_plan = None
        self.failure_recovery_start_pose = None

    def mark_stopped(self) -> bool:
        was_running = self.running
        self.running = False
        return was_running

    def diagnostic_prefix(self, stage: str) -> str:
        return (
            f"[visual_grasp][run={self.current_run_id}]"
            f"[attempt={self.current_attempt_index}]"
            f"[candidate={self.current_candidate_index}]"
            f"[stage={stage}]"
        )
