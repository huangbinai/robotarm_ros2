from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable

import numpy as np

from .mode_transition_policy import (
    ModeTransitionConfig,
    blend_enter_tau,
    blend_exit_tau,
    blend_scalar,
    validate_feedback,
    validate_mode_transition,
)


@dataclass(frozen=True)
class ModeTransitionResult:
    success: bool
    source_mode: str
    target_mode: str
    stage: str
    duration_sec: float
    max_velocity_rad_s: float = 0.0
    max_position_delta_rad: float = 0.0
    failure_reason: str = ""


class ModeTransitionCoordinator:
    def __init__(
        self,
        hardware,
        config: ModeTransitionConfig,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        control_period_sec: float = 0.02,
        on_stage: Callable[[str], None] | None = None,
    ) -> None:
        self._hardware = hardware
        self._config = config
        self._monotonic = monotonic
        self._sleep = sleep
        self._control_period_sec = max(float(control_period_sec), 1e-4)
        self._on_stage = on_stage or (lambda _stage: None)
        self._lock = threading.Lock()
        self._stage = "IDLE"

    @property
    def in_progress(self) -> bool:
        return self._lock.locked()

    @property
    def stage(self) -> str:
        return self._stage

    def enter_gravity_compensation(self) -> ModeTransitionResult:
        return self._run_transition("pos_vel", "mit", self._enter)

    def exit_gravity_compensation(self) -> ModeTransitionResult:
        return self._run_transition("mit", "pos_vel", self._exit)

    def _run_transition(self, source: str, target: str, operation) -> ModeTransitionResult:
        started = self._monotonic()
        if not self._lock.acquire(blocking=False):
            return ModeTransitionResult(
                success=False,
                source_mode=source,
                target_mode=target,
                stage=self._stage,
                duration_sec=0.0,
                failure_reason="mode transition already in progress",
            )

        metrics = {"max_velocity": 0.0, "max_delta": 0.0}
        try:
            validate_mode_transition(source, target, self._config)
            if str(self._hardware.mode).lower() != source:
                raise ValueError(
                    f"mode transition expected {source}, got {self._hardware.mode}"
                )
            final_stage = operation(started, metrics)
            return ModeTransitionResult(
                success=True,
                source_mode=source,
                target_mode=target,
                stage=final_stage,
                duration_sec=self._monotonic() - started,
                max_velocity_rad_s=metrics["max_velocity"],
                max_position_delta_rad=metrics["max_delta"],
            )
        except Exception as exc:
            failed_stage = self._stage
            self._recover_to_safe_state()
            return ModeTransitionResult(
                success=False,
                source_mode=source,
                target_mode=target,
                stage=failed_stage,
                duration_sec=self._monotonic() - started,
                max_velocity_rad_s=metrics["max_velocity"],
                max_position_delta_rad=metrics["max_delta"],
                failure_reason=str(exc),
            )
        finally:
            self._lock.release()

    def _enter(self, started: float, metrics: dict[str, float]) -> str:
        sample = self._hardware.feedback()
        validate_feedback(
            sample,
            self._config,
            max_velocity_rad_s=self._config.enter_max_velocity_rad_s,
        )
        q_hold = sample.positions.copy()
        self._observe(sample, q_hold, metrics)
        self._set_stage("ENTERING_GRAVITY_COMP")
        self._hardware.preload_position_hold(q_hold)
        self._hardware.stop_control_loop()
        self._hardware.switch_mode(
            "mit",
            kp=self._config.hold_kp,
            kd=self._config.hold_kd,
        )

        for progress in self._progress(self._config.enter_ramp_duration_sec):
            self._check_global_timeout(started)
            sample = self._hardware.feedback()
            validate_feedback(sample, self._config)
            self._observe(sample, q_hold, metrics)
            gravity = self._hardware.gravity_torque(sample.positions)
            self._hardware.send_mit(
                position=q_hold,
                kp=blend_scalar(
                    self._config.hold_kp,
                    self._config.gravity_kp,
                    progress,
                ),
                kd=blend_scalar(
                    self._config.hold_kd,
                    self._config.gravity_kd,
                    progress,
                ),
                torque=blend_enter_tau(gravity, progress),
            )

        self._hardware.start_gravity_loop(q_hold)
        self._set_stage("GRAVITY_COMP")
        return self._stage

    def _exit(self, started: float, metrics: dict[str, float]) -> str:
        self._set_stage("EXIT_DAMPING")
        self._hardware.stop_control_loop()
        damping_started = self._monotonic()
        q_hold = None
        while self._monotonic() - damping_started <= self._config.exit_velocity_wait_timeout_sec:
            self._check_global_timeout(started)
            sample = self._hardware.feedback()
            validate_feedback(sample, self._config)
            self._observe(sample, sample.positions, metrics)
            maximum = self._maximum_velocity(sample.velocities)
            gravity = self._hardware.gravity_torque(sample.positions)
            damping_progress = min(
                (self._monotonic() - damping_started)
                / max(self._config.exit_damping_duration_sec, 1e-6),
                1.0,
            )
            self._hardware.send_mit(
                position=sample.positions,
                kp=self._config.gravity_kp,
                kd=blend_scalar(
                    self._config.gravity_kd,
                    self._config.hold_kd,
                    damping_progress,
                ),
                torque=gravity,
            )
            if maximum <= self._config.exit_max_lock_velocity_rad_s:
                q_hold = sample.positions.copy()
                break
            self._sleep(self._control_period_sec)

        if q_hold is None:
            raise RuntimeError("joint velocity did not settle before exit transition")

        self._set_stage("EXIT_BLENDING")
        for progress in self._progress(self._config.exit_blend_duration_sec):
            self._check_global_timeout(started)
            sample = self._hardware.feedback()
            validate_feedback(sample, self._config)
            self._observe(sample, q_hold, metrics)
            if metrics["max_delta"] > self._config.max_position_jump_rad:
                raise RuntimeError(
                    "position changed beyond transition limit "
                    f"({metrics['max_delta']:.4f} rad)"
                )
            gravity = self._hardware.gravity_torque(sample.positions)
            self._hardware.send_mit(
                position=q_hold,
                kp=blend_scalar(
                    self._config.gravity_kp,
                    self._config.hold_kp,
                    progress,
                ),
                kd=blend_scalar(
                    self._config.gravity_kd,
                    self._config.hold_kd,
                    progress,
                ),
                torque=blend_exit_tau(gravity, progress),
            )

        self._hardware.finish_gravity_compensation()
        self._hardware.switch_mode("pos_vel")
        self._set_stage("POS_VEL_SETTLING")
        self._hardware.start_position_hold(q_hold, zero_velocity_limit=True)
        self._sleep(self._config.pos_vel_settle_duration_sec)
        self._hardware.restore_position_velocity_limit()
        self._set_stage("POS_VEL_HOLD")
        return self._stage

    def _recover_to_safe_state(self) -> None:
        try:
            sample = self._hardware.feedback()
            validate_feedback(sample, self._config)
        except Exception:
            self._hardware.disable_immediately()
            self._set_stage("TRANSITION_FAILED")
            return

        try:
            self._hardware.preload_position_hold(sample.positions)
            if str(self._hardware.mode).lower() != "pos_vel":
                self._hardware.stop_control_loop()
                self._hardware.switch_mode("pos_vel")
            self._hardware.start_position_hold(
                sample.positions,
                zero_velocity_limit=True,
            )
        except Exception:
            self._hardware.disable_immediately()
        self._set_stage("TRANSITION_FAILED")

    def _progress(self, duration_sec: float):
        duration = max(float(duration_sec), 0.0)
        if duration == 0.0:
            yield 1.0
            return
        started = self._monotonic()
        while True:
            elapsed = self._monotonic() - started
            progress = min(elapsed / duration, 1.0)
            yield progress
            if progress >= 1.0:
                return
            self._sleep(min(self._control_period_sec, duration - elapsed))

    def _check_global_timeout(self, started: float) -> None:
        if self._monotonic() - started > self._config.transition_timeout_sec:
            raise TimeoutError("mode transition timed out")

    def _observe(self, sample, reference: np.ndarray, metrics: dict[str, float]) -> None:
        metrics["max_velocity"] = max(
            metrics["max_velocity"],
            self._maximum_velocity(sample.velocities),
        )
        if sample.positions.size:
            metrics["max_delta"] = max(
                metrics["max_delta"],
                float(np.max(np.abs(sample.positions - reference))),
            )

    @staticmethod
    def _maximum_velocity(velocities: np.ndarray) -> float:
        return float(np.max(np.abs(velocities))) if velocities.size else 0.0

    def _set_stage(self, stage: str) -> None:
        self._stage = stage
        self._on_stage(stage)
