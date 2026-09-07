from __future__ import annotations

from collections.abc import Callable, Sequence
import threading
import time
from typing import Any

from .feedback_sequence import (
    VerifiedFeedbackSample,
    sequence_advanced,
    validate_sequence,
)


ControllerGroup = tuple[object, list[tuple[str, object]]]
FeedbackObservation = tuple[object, int]


def build_controller_groups(
    arm: Any,
    *,
    gripper_motor: object | None,
    gripper_controller: object | None,
) -> list[ControllerGroup]:
    groups: list[ControllerGroup] = []

    def add(controller: object, label: str, motor: object) -> None:
        for existing, entries in groups:
            if existing is controller:
                entries.append((label, motor))
                return
        groups.append((controller, [(label, motor)]))

    controller_map = getattr(arm, "_ctrl_map", {})
    motor_map = getattr(arm, "_motor_map", {})
    for joint in getattr(arm, "_joints", []):
        label = str(joint.name)
        controller = controller_map.get(getattr(joint, "vendor", None))
        motor = motor_map.get(label)
        if controller is None or motor is None:
            raise RuntimeError(f"{label} feedback hardware unavailable")
        add(controller, label, motor)
    if gripper_motor is not None:
        if gripper_controller is None:
            raise RuntimeError("gripper feedback controller unavailable")
        add(gripper_controller, "gripper", gripper_motor)
    if not groups:
        raise RuntimeError("hardware feedback controller map unavailable")
    return groups


class HardwareFeedbackCoordinator:
    """Owns feedback polling, sequence verification, caching, and freshness."""

    def __init__(
        self,
        *,
        feedback_period_sec: float,
        stale_timeout_sec: float,
        controller_groups: Callable[[], Sequence[ControllerGroup]],
        joint_labels: Callable[[], Sequence[str]],
        has_gripper: Callable[[], bool],
        validate_state: Callable[[str, object], None],
        on_verified: Callable[[str, object, float], None],
        refresh_retries: int,
        retry_interval_sec: float,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._feedback_period_sec = float(feedback_period_sec)
        self._stale_timeout_sec = float(stale_timeout_sec)
        self._controller_groups = controller_groups
        self._joint_labels = joint_labels
        self._has_gripper = has_gripper
        self._validate_state = validate_state
        self._on_verified = on_verified
        self._refresh_retries = int(refresh_retries)
        self._retry_interval_sec = float(retry_interval_sec)
        self._monotonic = monotonic
        self._sleep = sleep
        self._lock = threading.RLock()
        self._verified_by_label: dict[str, VerifiedFeedbackSample] = {}
        self._request_baseline_by_label: dict[str, int] = {}
        self._request_deadline_by_label: dict[str, float] = {}
        self._error_by_label: dict[str, str] = {}
        self._next_refresh_monotonic: float | None = None
        self._arm_updated_monotonic: float | None = None
        self._arm_error: str | None = "arm feedback not received"
        self._gripper_updated_monotonic: float | None = None
        self._gripper_error: str | None = "gripper feedback not received"

    @property
    def arm_updated_monotonic(self) -> float | None:
        with self._lock:
            return self._arm_updated_monotonic

    @property
    def gripper_updated_monotonic(self) -> float | None:
        with self._lock:
            return self._gripper_updated_monotonic

    @property
    def gripper_error(self) -> str | None:
        with self._lock:
            return self._gripper_error

    def verified_samples(self) -> dict[str, VerifiedFeedbackSample]:
        with self._lock:
            return dict(self._verified_by_label)

    def pending_labels(self) -> set[str]:
        with self._lock:
            return set(self._request_baseline_by_label)

    def sample(self, label: str) -> VerifiedFeedbackSample:
        with self._lock:
            sample = self._verified_by_label.get(str(label))
        if sample is None:
            raise RuntimeError(f"{label} verified feedback unavailable")
        return sample

    def refresh_if_due(
        self,
        *,
        force: bool = False,
        now: float | None = None,
    ) -> bool:
        observed_at = self._monotonic() if now is None else float(now)
        with self._lock:
            due = self._next_refresh_monotonic
            if not force and due is not None and observed_at < due:
                return False
            self._next_refresh_monotonic = observed_at + self._feedback_period_sec
            try:
                if force:
                    self._force_refresh()
                else:
                    self._refresh_batch(observed_at=observed_at)
                return True
            except Exception:
                if force:
                    raise
                return False

    def arm_failure_reason(self, *, now: float | None = None) -> str | None:
        with self._lock:
            if self._arm_error:
                return f"arm feedback unavailable: {self._arm_error}"
            current = self._monotonic() if now is None else float(now)
            updated = self._arm_updated_monotonic
            age = float("inf") if updated is None else max(current - updated, 0.0)
            if age > self._stale_timeout_sec:
                return (
                    f"arm feedback stale: age={age:.3f}s "
                    f"limit={self._stale_timeout_sec:.3f}s"
                )
            return None

    @staticmethod
    def state_with_sequence(label: str, motor) -> FeedbackObservation:
        getter = getattr(motor, "get_state_with_sequence", None)
        if not callable(getter):
            raise RuntimeError(
                f"{label} feedback requires patched MotorBridge "
                "get_state_with_sequence(); run "
                "tools/setup_motorbridge_fresh_feedback.py"
            )
        state, raw_sequence = getter()
        try:
            sequence = validate_sequence(raw_sequence)
        except ValueError as exc:
            raise RuntimeError(f"{label} feedback sequence invalid: {exc}") from exc
        return state, sequence

    def _record_verified(
        self,
        label: str,
        state: object,
        sequence: int,
        observed_at: float,
    ) -> None:
        self._validate_state(label, state)
        self._verified_by_label[label] = VerifiedFeedbackSample(
            state,
            sequence,
            observed_at,
        )
        self._error_by_label.pop(label, None)
        if label == "gripper":
            self._gripper_updated_monotonic = observed_at
        self._on_verified(label, state, observed_at)

    def _inspect_pending(
        self,
        observations: dict[str, FeedbackObservation],
        *,
        observed_at: float,
    ) -> None:
        for label, baseline in list(self._request_baseline_by_label.items()):
            observation = observations.get(label)
            if observation is None:
                continue
            state, sequence = observation
            if sequence_advanced(sequence, baseline):
                try:
                    self._record_verified(label, state, sequence, observed_at)
                except Exception as exc:
                    self._error_by_label[label] = f"{label} feedback invalid: {exc}"
                self._clear_pending(label)
                continue
            deadline = self._request_deadline_by_label[label]
            if observed_at >= deadline:
                self._error_by_label[label] = (
                    f"{label} feedback deadline expired: sequence did not advance "
                    f"beyond baseline={baseline}"
                )
                self._clear_pending(label)

    def _refresh_batch(
        self,
        *,
        observed_at: float,
        inspect_after_poll: bool = False,
    ) -> None:
        groups = list(self._controller_groups())
        observations: dict[str, FeedbackObservation] = {}
        group_errors: list[str] = []
        readable_groups: list[ControllerGroup] = []
        for controller, entries in groups:
            try:
                for label, motor in entries:
                    observations[label] = self.state_with_sequence(label, motor)
                readable_groups.append((controller, entries))
            except Exception as exc:
                message = self._group_error_message(controller, entries, exc)
                group_errors.append(message)
                for label, _motor in entries:
                    self._error_by_label[label] = (
                        f"shared feedback batch failed before request: {message}"
                    )

        self._inspect_pending(observations, observed_at=observed_at)
        response_window = self._response_window_sec()
        for _controller, entries in readable_groups:
            for label, _motor in entries:
                if label not in self._request_baseline_by_label:
                    _state, sequence = observations[label]
                    self._request_baseline_by_label[label] = sequence
                    self._request_deadline_by_label[label] = (
                        observed_at + response_window
                    )

        successful_groups: list[ControllerGroup] = []
        for controller, entries in readable_groups:
            try:
                self._poll_group(controller, entries)
                successful_groups.append((controller, entries))
            except Exception as exc:
                message = self._group_error_message(controller, entries, exc)
                group_errors.append(message)
                for label, _motor in entries:
                    self._error_by_label[label] = (
                        f"shared feedback batch failed: {message}"
                    )
                    self._clear_pending(label)

        if inspect_after_poll:
            self._inspect_successful_groups_after_poll(
                successful_groups,
                group_errors,
            )
        self._sync_health()
        if group_errors:
            raise RuntimeError(
                "shared feedback batch failed: " + "; ".join(group_errors)
            )

    def _force_refresh(self) -> None:
        groups = list(self._controller_groups())
        initial: dict[str, FeedbackObservation] = {}
        initial_errors: list[str] = []
        for controller, entries in groups:
            try:
                for label, motor in entries:
                    initial[label] = self.state_with_sequence(label, motor)
            except Exception as exc:
                message = self._group_error_message(controller, entries, exc)
                initial_errors.append(message)
                for label, _motor in entries:
                    self._error_by_label[label] = message
        if initial_errors:
            self._sync_health()
            raise RuntimeError(
                "forced feedback baseline failed before request: "
                + "; ".join(initial_errors)
            )

        required_baselines = {
            label: sequence for label, (_state, sequence) in initial.items()
        }
        prior_samples = {
            label: self._verified_by_label.get(label) for label in required_baselines
        }

        def forced_sample_satisfies(label: str, baseline: int) -> bool:
            sample = self._verified_by_label.get(label)
            return bool(
                label not in self._error_by_label
                and sample is not None
                and sample is not prior_samples[label]
                and sequence_advanced(sample.sequence, baseline)
            )

        for label in required_baselines:
            self._clear_pending(label)

        last_error: Exception | None = None
        for attempt in range(self._refresh_retries):
            attempt_error: Exception | None = None
            try:
                self._refresh_batch(
                    observed_at=self._monotonic(),
                    inspect_after_poll=True,
                )
            except Exception as exc:
                last_error = exc
                attempt_error = exc
            if attempt_error is None and all(
                forced_sample_satisfies(label, baseline)
                for label, baseline in required_baselines.items()
            ):
                return
            if attempt + 1 < self._refresh_retries:
                self._sleep(self._retry_interval_sec)

        missing: list[str] = []
        for label, baseline in required_baselines.items():
            if not forced_sample_satisfies(label, baseline):
                reason = self._error_by_label.get(label)
                if reason is None:
                    reason = (
                        f"{label} feedback timeout: sequence did not advance "
                        f"beyond baseline={baseline}"
                    )
                self._error_by_label[label] = reason
                missing.append(reason)
            self._clear_pending(label)
        self._sync_health()
        if last_error is not None:
            raise last_error
        raise RuntimeError("fresh hardware feedback unavailable: " + "; ".join(missing))

    def _inspect_successful_groups_after_poll(
        self,
        successful_groups: Sequence[ControllerGroup],
        group_errors: list[str],
    ) -> None:
        completed_at = self._monotonic()
        observations: dict[str, FeedbackObservation] = {}
        for controller, entries in successful_groups:
            try:
                for label, motor in entries:
                    observations[label] = self.state_with_sequence(label, motor)
            except Exception as exc:
                message = self._group_error_message(controller, entries, exc)
                group_errors.append(message)
                for label, _motor in entries:
                    self._error_by_label[label] = message
        self._inspect_pending(observations, observed_at=completed_at)

    def _sync_health(self) -> None:
        labels = [str(label) for label in self._joint_labels()]
        errors = [
            self._error_by_label[label]
            for label in labels
            if label in self._error_by_label
        ]
        missing = [label for label in labels if label not in self._verified_by_label]
        if errors:
            self._arm_error = "; ".join(errors)
        elif missing:
            self._arm_error = "verified feedback pending: " + ",".join(missing)
        else:
            self._arm_error = None
            self._arm_updated_monotonic = min(
                self._verified_by_label[label].observed_at for label in labels
            )
        if self._has_gripper():
            self._gripper_error = self._error_by_label.get("gripper")
            if "gripper" not in self._verified_by_label:
                self._gripper_error = "gripper feedback not received"

    def _response_window_sec(self) -> float:
        return max(
            self._feedback_period_sec * self._refresh_retries,
            self._retry_interval_sec * self._refresh_retries,
        )

    @staticmethod
    def _poll_group(controller: object, entries: Sequence[tuple[str, object]]) -> None:
        def transaction() -> None:
            for _label, motor in entries:
                motor.request_feedback()
            controller.poll_feedback_once()

        lock = getattr(controller, "_bus_lock", None)
        if lock is None:
            transaction()
            return
        with lock:
            transaction()

    @staticmethod
    def _group_error_message(
        controller: object,
        entries: Sequence[tuple[str, object]],
        error: Exception,
    ) -> str:
        labels = ",".join(label for label, _motor in entries)
        return f"controller={type(controller).__name__} motors={labels}: {error}"

    def _clear_pending(self, label: str) -> None:
        self._request_baseline_by_label.pop(label, None)
        self._request_deadline_by_label.pop(label, None)
