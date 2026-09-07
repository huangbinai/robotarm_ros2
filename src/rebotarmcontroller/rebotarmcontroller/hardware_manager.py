from __future__ import annotations

import threading
import time
import logging
from typing import Optional, Sequence

import numpy as np

from .bus_synchronization import patch_arm_bus_lock
from .command_arbiter import CommandArbiter
from .conversions import fk_to_pose
from .feedback_sequence import VerifiedFeedbackSample
from .gripper_coordinates import DEFAULT_GRIPPER_COORDINATES
from .gripper_grasp import GripperGraspConfig, GripperGraspCoordinator
from .gripper_motion_policy import (
    GripperMotionPolicyConfig,
    GripperTickDecision,
    decide_gripper_tick,
)
from .gripper_motor_commands import (
    dispatch_gripper_motor_command,
    resolve_gripper_motor_command,
    send_safe_gripper_mit,
)
from .gripper_position import (
    GripperPositionConfig,
    GripperPositionCoordinator,
    GripperPositionProgress,
)
from .gripper_runtime_state import GripperRuntimeField, GripperRuntimeState
from .gripper_sdk_adapter import GripperSdkAdapter
from .gravity_compensation_state import (
    GravityCompensationField,
    GravityCompensationState,
)
from .hardware_feedback import HardwareFeedbackCoordinator, build_controller_groups
from .hardware_feedback_validation import (
    validate_feedback_state as validate_hardware_feedback_state,
    validated_gripper_feedback_values,
)
from .hardware_disable import (
    attempt_verified_disable,
    disable_unique_controller_buses,
)
from .hardware_lifecycle_state import (
    HardwareLifecycleField,
    HardwareLifecycleState,
)
from .hardware_runtime_config import HardwareRuntimeConfig
from .hardware_sdk_runtime import create_hardware_sdk_runtime
from .joint_motor_commands import (
    dispatch_joint_motor_command,
    resolve_joint_motor_command,
)
from .mode_transition import ModeTransitionCoordinator
from .mode_transition_policy import (
    FeedbackSample,
    ModeTransitionConfig,
    validate_mode_transition,
)

_LOG = logging.getLogger(__name__)

_GRIPPER_COORDINATES = DEFAULT_GRIPPER_COORDINATES
_G_ZERO_VERIFY_TIMEOUT_SEC = 0.5
_G_ZERO_VERIFY_SAMPLES = 3
_G_ARRIVE_TOL = 0.12
_G_TAU_MAX = 1.5
_G_POSITION_TORQUE_CAP_NM = 1.0
_G_POSITION_MAX_SPEED_RAD_S = 0.5
_G_POSITION_TIMEOUT_MARGIN_SEC = 1.5
_G_KP_MOVE = 5.0
_G_KD_MOVE = 1.0
_G_DEFAULT_FORCE = 0.40
_G_GRASP_CLOSE_KP = 0.0
_G_GRASP_CLOSE_KD = 0.5
_G_GRASP_HOLD_KP = 5.0
_G_GRASP_HOLD_KD = 1.0
_G_GRASP_CLOSE_FORCE_DEFAULT = 0.40
_G_GRASP_CLOSE_FORCE_MAX = 1.0
_G_GRASP_HOLD_FORCE_DEFAULT = 0.40
_G_GRASP_VEL_THRESHOLD = 0.04
_G_GRASP_MIN_CLOSE_TIME = 0.08
_G_GRASP_MIN_CLOSURE_M = 0.006
_G_GRASP_TIMEOUT = 2.0
_G_GRASP_HOLD_TIMEOUT_SEC = 30.0
_G_GRASP_HOLD_TIMEOUT_MAX_SEC = 120.0
_G_GRASP_EMPTY_CLOSE_THRESHOLD_M = 0.003
_G_GRASP_CONTACT_TORQUE_MIN = 0.0
_G_GRASP_CONTACT_STABLE_SAMPLES = 3
_HARDWARE_FEEDBACK_RATE_HZ = 50.0
_FEEDBACK_REFRESH_RETRIES = 3
_FEEDBACK_RETRY_INTERVAL_SEC = 0.005
_FEEDBACK_STALE_TIMEOUT_SEC = 0.15
_GC_VEL_THRESHOLD = 0.04
_GC_W_VEL_THRESHOLD = 0.08
_GC_EE_FRAME = "end_link"
_GC_KP = 7.0
_GC_KD = 0.8
_GC_TAU_SCALE = np.ones(6, dtype=np.float64)

_JOINT_FEEDBACK_LIMITS_RAD = {
    "joint1": (-2.8, 2.8),
    "joint2": (-3.14, 0.02),
    "joint3": (-3.14, 0.02),
    "joint4": (-1.87, 1.57),
    "joint5": (-1.57, 1.57),
    "joint6": (-3.14, 3.14),
}

def apply_gravity_compensation_tau_scale(tau: np.ndarray) -> np.ndarray:
    scaled = np.array(tau, dtype=np.float64, copy=True)
    if scaled.shape == _GC_TAU_SCALE.shape:
        scaled *= _GC_TAU_SCALE
    return scaled


class HardwareManager:
    """Owns the single RobotArm instance used by the ROS driver."""

    _connected = HardwareLifecycleField("connected")
    _enabled = HardwareLifecycleField("enabled")
    _lifecycle_state = HardwareLifecycleField("lifecycle_state")
    _state_machine = HardwareLifecycleField("state_machine")
    _gravity_comp_active = GravityCompensationField("active")
    _gravity_comp_q_target = GravityCompensationField("target")
    _gravity_comp_integral = GravityCompensationField("integral")
    _gravity_comp_lock_counter = GravityCompensationField("lock_counter")
    _gravity_comp_q_last = GravityCompensationField("last_position")

    _gripper_target_angle = GripperRuntimeField("target_angle")
    _gripper_goal_angle = GripperRuntimeField("goal_angle")
    _gripper_target_effort = GripperRuntimeField("target_effort")
    _gripper_close_force = GripperRuntimeField("close_force")
    _gripper_hold_force = GripperRuntimeField("hold_force")
    _gripper_hold_angle = GripperRuntimeField("hold_angle")
    _gripper_hold_deadline = GripperRuntimeField("hold_deadline")
    _gripper_hold_release_reason = GripperRuntimeField("hold_release_reason")
    _gripper_mode = GripperRuntimeField("mode")
    _gripper_active = GripperRuntimeField("active")
    _gripper_pos = GripperRuntimeField("position")
    _gripper_vel = GripperRuntimeField("velocity")
    _gripper_torque = GripperRuntimeField("torque")
    _gripper_command_error = GripperRuntimeField("command_error")
    _gripper_position_result = GripperRuntimeField("position_result")
    _gripper_target_timeout_sec = GripperRuntimeField("target_timeout_sec")
    _gripper_target_deadline_monotonic = GripperRuntimeField("target_deadline")
    _gripper_last_tick_monotonic = GripperRuntimeField("last_tick")
    _gripper_neutral_pending = GripperRuntimeField("neutral_pending")

    def __init__(
        self,
        arm_cfg: Optional[str] = None,
        gripper_cfg: Optional[str] = None,
        channel: str = "",
        mode_transition_config: ModeTransitionConfig | None = None,
        hardware_feedback_rate_hz: float = _HARDWARE_FEEDBACK_RATE_HZ,
        feedback_stale_timeout_sec: float = _FEEDBACK_STALE_TIMEOUT_SEC,
        gripper_position_torque_cap_nm: float = _G_POSITION_TORQUE_CAP_NM,
        gripper_position_max_speed_rad_s: float = _G_POSITION_MAX_SPEED_RAD_S,
        gripper_position_timeout_margin_sec: float = _G_POSITION_TIMEOUT_MARGIN_SEC,
        grasp_hold_timeout_sec: float = _G_GRASP_HOLD_TIMEOUT_SEC,
        gripper_contact_torque_min_nm: float = _G_GRASP_CONTACT_TORQUE_MIN,
    ) -> None:
        runtime_config = HardwareRuntimeConfig.validate(
            hardware_feedback_rate_hz=hardware_feedback_rate_hz,
            feedback_stale_timeout_sec=feedback_stale_timeout_sec,
            gripper_position_torque_cap_nm=gripper_position_torque_cap_nm,
            gripper_position_max_speed_rad_s=gripper_position_max_speed_rad_s,
            gripper_position_timeout_margin_sec=gripper_position_timeout_margin_sec,
            grasp_hold_timeout_sec=grasp_hold_timeout_sec,
            gripper_contact_torque_min_nm=gripper_contact_torque_min_nm,
        )
        self._feedback_stale_timeout_sec = runtime_config.feedback_stale_timeout_sec
        self._gripper_position_torque_cap_nm = runtime_config.gripper_position_torque_cap_nm
        self._gripper_position_max_speed_rad_s = runtime_config.gripper_position_max_speed_rad_s
        self._gripper_position_timeout_margin_sec = runtime_config.gripper_position_timeout_margin_sec
        self._grasp_hold_timeout_sec = runtime_config.grasp_hold_timeout_sec
        self._gripper_contact_torque_min_nm = runtime_config.gripper_contact_torque_min_nm
        sdk_runtime = create_hardware_sdk_runtime(
            module_path=__file__,
            arm_config=arm_cfg,
            gripper_config=gripper_cfg,
            channel=channel,
            end_effector_frame=_GC_EE_FRAME,
        )
        self._sdk_root = sdk_runtime.sdk_root
        self._arm = sdk_runtime.arm
        self._gravity_dynamics = sdk_runtime.gravity_dynamics
        self._gripper_cfg_path = sdk_runtime.gripper_config_path
        self._gripper_cfg = None
        self._gripper_mot = None
        self._gripper_ctrl = None
        self._gripper_state = GripperRuntimeState(
            target_effort=_G_DEFAULT_FORCE,
            close_force=_G_GRASP_CLOSE_FORCE_DEFAULT,
            hold_force=_G_GRASP_HOLD_FORCE_DEFAULT,
        )
        self._gripper_command_cancel = threading.Event()
        self._gripper_lock = threading.RLock()
        self._gripper_position = GripperPositionCoordinator(
            self,
            GripperPositionConfig(
                coordinates=_GRIPPER_COORDINATES,
                feedback_stale_timeout_sec=self._feedback_stale_timeout_sec,
                default_effort_nm=_G_DEFAULT_FORCE,
                torque_cap_nm=self._gripper_position_torque_cap_nm,
                maximum_speed_rad_s=self._gripper_position_max_speed_rad_s,
                timeout_margin_sec=self._gripper_position_timeout_margin_sec,
                arrival_tolerance_rad=_G_ARRIVE_TOL,
            ),
        )
        self._gripper_grasp = GripperGraspCoordinator(
            self,
            GripperGraspConfig(
                feedback_stale_timeout_sec=self._feedback_stale_timeout_sec,
                default_hold_timeout_sec=self._grasp_hold_timeout_sec,
                maximum_hold_timeout_sec=_G_GRASP_HOLD_TIMEOUT_MAX_SEC,
                maximum_close_force_nm=_G_GRASP_CLOSE_FORCE_MAX,
                maximum_torque_nm=_G_TAU_MAX,
                empty_close_threshold_m=_G_GRASP_EMPTY_CLOSE_THRESHOLD_M,
                contact_torque_min_nm=self._gripper_contact_torque_min_nm,
                contact_stable_samples=_G_GRASP_CONTACT_STABLE_SAMPLES,
            ),
        )

        self._feedback_coordinator = HardwareFeedbackCoordinator(
            feedback_period_sec=runtime_config.hardware_feedback_period_sec,
            stale_timeout_sec=runtime_config.feedback_stale_timeout_sec,
            controller_groups=self._feedback_controller_groups,
            joint_labels=lambda: self.joint_names,
            has_gripper=lambda: self._gripper_mot is not None,
            validate_state=self._validate_feedback_state,
            on_verified=self._on_verified_feedback,
            refresh_retries=_FEEDBACK_REFRESH_RETRIES,
            retry_interval_sec=_FEEDBACK_RETRY_INTERVAL_SEC,
        )
        self._gripper_zero_error: str | None = None
        self._motor_lifecycle_lock = threading.RLock()

        self._endpos_ctrl = sdk_runtime.create_endpos_controller()
        self._lifecycle = HardwareLifecycleState()
        self.command_arbiter = CommandArbiter()
        self._error_codes: list[str] = []
        self._gravity_state = GravityCompensationState()

        self._mode_transition_config = mode_transition_config or ModeTransitionConfig()
        self._mode_transition = ModeTransitionCoordinator(
            self,
            self._mode_transition_config,
            control_period_sec=1.0 / float(getattr(self._arm, "_rate", 500.0)),
            on_stage=self._on_mode_transition_stage,
        )

        patch_arm_bus_lock(self._arm)

    @property
    def arm(self):
        return self._arm

    @property
    def endpos_ctrl(self):
        return self._endpos_ctrl

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm.mode)

    @property
    def enabled(self) -> bool:
        return self._lifecycle.enabled

    @property
    def connected(self) -> bool:
        return self._lifecycle.connected

    @property
    def control_loop_active(self) -> bool:
        return bool(self._arm.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return self._gripper_mot is not None

    @property
    def state_machine(self) -> str:
        return self._lifecycle.state_machine

    @property
    def lifecycle_state(self) -> str:
        return self._lifecycle.lifecycle_state

    @property
    def ready_for_motion(self) -> bool:
        """Return whether new arm or gripper motion goals may be accepted."""
        return self._lifecycle.ready_for_motion

    @property
    def gripper_active(self) -> bool:
        with self._gripper_lock:
            return bool(self._gripper_state.active)

    @property
    def gripper_mode(self) -> str:
        with self._gripper_lock:
            return str(self._gripper_state.mode)

    @property
    def error_codes(self) -> list[str]:
        codes = list(self._error_codes)
        if not getattr(self, "_connected", False):
            return codes
        arm_failure = self._arm_feedback_failure_reason()
        if arm_failure is not None:
            codes.append(f"ARM_FEEDBACK: {arm_failure}")
        if self._gripper_mot is not None:
            gripper_failure = self._gripper_feedback_failure_reason()
            if gripper_failure is not None:
                codes.append(f"GRIPPER_FEEDBACK: {gripper_failure}")
        return codes

    def set_state_machine(self, state: str) -> None:
        self._lifecycle.set_state_machine(state)

    def _set_lifecycle_state(self, state: str) -> None:
        self._lifecycle.set_lifecycle_state(state)

    def _require_connected(self) -> None:
        self._lifecycle.require_connected()

    def _require_enabled(self) -> None:
        self._lifecycle.require_enabled()

    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._arm.connect()
            self._connected = True
            self.init_gripper(str(self._gripper_cfg_path))
            _positions, _velocities, _torques, statuses = self._validated_joint_feedback()
            gripper_status = None
            if self._gripper_mot is not None:
                gripper_status = self._validated_gripper_status()
            if any(status != 0 for status in statuses) or gripper_status not in (None, 0):
                self._disable_all_motors()
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            self._enabled = False
            self._set_lifecycle_state("CONNECTED_DISABLED")
        except Exception:
            self._disconnect_after_failed_connect()
            raise

    def _disconnect_after_failed_connect(self) -> None:
        try:
            self._stop_control_loop()
        except Exception:
            pass
        try:
            self._disable_all_motors()
        except Exception:
            pass
        try:
            self._arm.disconnect()
        except Exception:
            pass
        self._connected = False
        self._enabled = False
        self._set_lifecycle_state("DISCONNECTED")

    def shutdown(self) -> bool:
        if not self._connected:
            return True
        was_enabled = self._enabled
        disable_verified = False
        disable_error: Exception | None = None
        disconnect_error: Exception | None = None
        try:
            try:
                self.stop_gravity_compensation()
            except Exception as exc:
                # Shutdown must continue to hard-disable even when a graceful
                # mode transition cannot be completed.
                message = f"SHUTDOWN_GRAVITY_STOP_FAILED: {exc}"
                if message not in self._error_codes:
                    self._error_codes.append(message)
            try:
                disable_verified = self.disable_immediately()
            except Exception as exc:
                disable_error = exc
                disable_verified = False
            if not disable_verified:
                detail = f": {disable_error}" if disable_error is not None else ""
                message = f"SHUTDOWN_DISABLE_UNVERIFIED{detail}"
                if message not in self._error_codes:
                    self._error_codes.append(message)
            self._endpos_ctrl._running = False
            try:
                self._arm.disconnect()
            except Exception as exc:
                disconnect_error = exc
        finally:
            if disconnect_error is None:
                self._connected = False
                # Preserve the last known enabled state when hard-disable could
                # not be verified.  DISCONNECTED prevents new commands while
                # the error code records that the physical state is unknown.
                self._enabled = False if disable_verified else was_enabled
                self._set_lifecycle_state("DISCONNECTED")
            else:
                self._connected = True
                self._enabled = False if disable_verified else was_enabled
                self._set_lifecycle_state(
                    "CONNECTED_DISABLED" if disable_verified else "DISABLING"
                )
        if disconnect_error is not None:
            message = f"SHUTDOWN_DISCONNECT_FAILED: {disconnect_error}"
            if message not in self._error_codes:
                self._error_codes.append(message)
        return bool(disable_verified and disconnect_error is None)

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._motor_lifecycle_lock:
            if not self._connected:
                return self._arm.get_state()
            if not self.control_loop_active:
                self.refresh_feedback_if_due()
            if self._lifecycle_state == "DISABLING":
                positions, velocities, torques, statuses = self._validated_joint_feedback(
                    expected_status=None,
                    refresh=False,
                )
                gripper_status = self._cached_gripper_status()
                if all(status == 0 for status in statuses) and gripper_status in (None, 0):
                    self._enabled = False
                    self._set_lifecycle_state("CONNECTED_DISABLED")
                return positions, velocities, torques
            positions, velocities, torques, _statuses = self._validated_joint_feedback(
                expected_status=1 if self._enabled else 0,
                refresh=False,
            )
            return positions, velocities, torques

    def get_cached_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._motor_lifecycle_lock:
            if not self._connected:
                return self._arm.get_state()
            positions, velocities, torques, _statuses = self._validated_joint_feedback(
                expected_status=1 if self._enabled else 0,
                refresh=False,
            )
            return positions, velocities, torques

    def hold_current_position(self) -> np.ndarray:
        q, _, _ = self.get_joint_state()
        current = np.array(q, dtype=np.float64, copy=True)
        self._endpos_ctrl._q_target[:] = current
        return current

    def stop_active_motion(self) -> None:
        self._endpos_ctrl._stop_send.set()
        self._endpos_ctrl._moving = False
        self.hold_current_position()
        self.set_state_machine("IDLE")

    def safe_home(self) -> bool:
        self.stop_gravity_compensation()
        self.ensure_pos_vel_control()
        reached = bool(self._endpos_ctrl.safe_home(state_reader=self.get_joint_state))
        if not reached:
            self.hold_current_position()
            self._error_codes.append("SAFE_HOME_TIMEOUT")
        return reached

    def enable(self) -> None:
        from motorbridge import Mode

        with self._motor_lifecycle_lock:
            self._require_connected()
            if self._enabled:
                self.hold_current_position()
                self._set_lifecycle_state("ENABLED_HOLD")
                return
            self._set_lifecycle_state("ENABLING")
            try:
                positions, _velocities, _torques, _statuses = (
                    self._validated_joint_feedback(expected_status=0)
                )
                zero_error = getattr(self, "_gripper_zero_error", None)
                if zero_error is not None:
                    raise RuntimeError(zero_error)
                self._validated_gripper_status(expected_status=0)
                self._endpos_ctrl._q_target[:] = positions
                if self._arm.mode_pos_vel() is False:
                    raise RuntimeError("failed to enter position-velocity control mode")
                if self._gripper_mot is not None:
                    self._gripper_mot.ensure_mode(Mode.MIT, 1000)
                self._arm.enable()
                if self._gripper_mot is not None and hasattr(self._gripper_mot, "enable"):
                    self._gripper_mot.enable()
                self._validated_joint_feedback(expected_status=1)
                self._validated_gripper_status(expected_status=1)
                self._enabled = True
                self._start_pos_vel_loop(target=positions)
                self.set_state_machine("IDLE")
            except Exception as exc:
                rollback_error = self._rollback_failed_enable()
                if rollback_error:
                    raise RuntimeError(
                        f"enable failed: {exc}; rollback verification failed: {rollback_error}"
                    ) from exc
                raise RuntimeError(f"enable failed and was rolled back: {exc}") from exc

    def _rollback_failed_enable(self) -> str | None:
        attempt = attempt_verified_disable(
            stop_control_loop=self._stop_control_loop,
            disable_all_motors=self._disable_all_motors,
            verify_disabled_feedback=self._verify_disabled_feedback,
        )
        self._state_machine = "IDLE"
        if not attempt.verified:
            self._enabled = True
            self._error_codes.append("ENABLE_ROLLBACK_FAILED")
            self._set_lifecycle_state("DISABLING")
            return attempt.error_detail
        self._enabled = False
        self._set_lifecycle_state("CONNECTED_DISABLED")
        return None

    def disable(self) -> None:
        with self._motor_lifecycle_lock:
            self._require_connected()
            self._set_lifecycle_state("DISABLING")
            was_enabled = self._enabled
            transition_error: Exception | None = None
            try:
                self.stop_gravity_compensation()
            except Exception as exc:
                # A graceful transition failure must not prevent an explicit
                # operator disable request from reaching the motors.
                transition_error = exc
            self._stop_control_loop()
            self._enabled = False
            try:
                self._disable_all_motors()
                self._verify_disabled_feedback()
            except Exception:
                self._enabled = was_enabled
                self._error_codes.append("DISABLE_VERIFICATION_FAILED")
                raise
            else:
                self._enabled = False
                self._state_machine = "IDLE"
                self._set_lifecycle_state("CONNECTED_DISABLED")
            if transition_error is not None:
                raise RuntimeError(str(transition_error)) from transition_error

    def set_mode(self, mode: str) -> bool:
        mode = mode.strip().lower()
        if mode not in ("mit", "pos_vel", "vel"):
            raise ValueError(f"unsupported mode: {mode}")
        if self._mode_transition.in_progress:
            raise RuntimeError("mode transition in progress")
        validate_mode_transition(self.mode, mode, self._mode_transition_config)
        if mode == "mit" and self.mode != "mit":
            raise ValueError(
                "direct MIT mode entry is disabled; use gravity compensation service"
            )
        self.stop_gravity_compensation()

        if mode == self.mode:
            if mode == "pos_vel" and self._enabled:
                if self.control_loop_active:
                    self.hold_current_position()
                else:
                    self._start_pos_vel_loop()
            self.set_state_machine("IDLE")
            return True

        self._stop_control_loop()
        if mode == "mit":
            ok = self._arm.mode_mit()
        elif mode == "pos_vel":
            ok = self._arm.mode_pos_vel()
            if self._enabled:
                self._start_pos_vel_loop()
        else:
            ok = self._arm.mode_vel()
        self.set_state_machine("IDLE")
        return bool(ok)

    def set_zero(self, joint_name: str = "") -> bool:
        with self._motor_lifecycle_lock:
            self._require_connected()
            if self._enabled or self._lifecycle_state != "CONNECTED_DISABLED":
                raise RuntimeError("set_zero requires CONNECTED_DISABLED state")
            self._stop_control_loop()
            normalized = str(joint_name).strip().lower()
            if normalized in ("gripper", "endjoint"):
                return self._set_gripper_zero()
            if normalized:
                ok = self._arm.set_zero_single(joint_name)
            else:
                self._arm.set_zero()
                ok = True
            if ok:
                self._validated_joint_feedback(expected_status=0)
            self._enabled = False
            self._set_lifecycle_state("CONNECTED_DISABLED")
            self.set_state_machine("IDLE")
            return bool(ok)

    def _set_gripper_zero(self) -> bool:
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        self._validated_gripper_status(expected_status=0)
        with self._gripper_lock:
            self._gripper_zero_error = "gripper zero verification pending"
        try:
            self._gripper_mot.set_zero_position()
            deadline = time.monotonic() + _G_ZERO_VERIFY_TIMEOUT_SEC
            consecutive = 0
            position = float("nan")
            while time.monotonic() < deadline:
                self.refresh_feedback_if_due(force=True)
                sample = self._verified_feedback_sample("gripper")
                position, _velocity, _torque, status = (
                    self._validated_gripper_feedback_values(sample.state)
                )
                if status != 0:
                    raise RuntimeError(
                        f"gripper status_code={status}, expected 0 after zero"
                    )
                consecutive = (
                    consecutive + 1
                    if abs(position) <= _GRIPPER_COORDINATES.coordinate_tolerance_rad
                    else 0
                )
                if consecutive >= _G_ZERO_VERIFY_SAMPLES:
                    with self._gripper_lock:
                        self._gripper_zero_error = None
                    self._enabled = False
                    self._set_lifecycle_state("CONNECTED_DISABLED")
                    self.set_state_machine("IDLE")
                    return True
                time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)
            raise TimeoutError(
                f"zero verification timed out: raw={position:.6f} rad; "
                f"need {_G_ZERO_VERIFY_SAMPLES} fresh disabled samples within "
                f"+/-{_GRIPPER_COORDINATES.coordinate_tolerance_rad:.6f} rad"
            )
        except Exception as exc:
            message = f"gripper set_zero failed: {exc}"
            with self._gripper_lock:
                self._gripper_zero_error = message
            raise RuntimeError(message) from exc

    def ensure_pos_vel_control(self) -> None:
        if self._mode_transition.in_progress:
            raise RuntimeError("mode transition in progress")
        self._require_enabled()
        if self._gravity_comp_active:
            self.stop_gravity_compensation()
        if self.mode != "pos_vel":
            validate_mode_transition(self.mode, "pos_vel", self._mode_transition_config)
            if self.mode == "mit":
                raise RuntimeError(
                    "MIT mode is not in gravity compensation; refusing abrupt POS_VEL switch"
                )
            self._stop_control_loop()
            self._arm.mode_pos_vel()
        if not self.control_loop_active:
            self._start_pos_vel_loop()
        else:
            self.hold_current_position()

    def send_joint_motor_cmd(self, joint_name: str, cmd) -> None:
        if self._mode_transition.in_progress:
            raise RuntimeError("mode transition in progress")
        if int(cmd.mode) == 2 and not self._mode_transition_config.allow_velocity_mode:
            raise ValueError("VEL mode is disabled")
        self._require_enabled()
        if joint_name not in self._arm._motor_map:
            raise KeyError(f"unknown joint: {joint_name}")

        motor = self._arm._motor_map[joint_name]
        joint_config = next(j for j in self._arm._joints if j.name == joint_name)
        state = self._verified_feedback_sample(joint_name).state
        motor_command = resolve_joint_motor_command(
            joint_name,
            cmd,
            feedback_state=state,
            config=joint_config,
            position_limits_rad=_JOINT_FEEDBACK_LIMITS_RAD[joint_name],
        )
        dispatch_joint_motor_command(
            motor,
            motor_command,
            joint_name=joint_name,
        )
        self.set_state_machine("LOWLEVEL_STREAMING")

    def start_gravity_compensation(self) -> None:
        if self._gravity_comp_active:
            return
        self._require_enabled()
        result = self._mode_transition.enter_gravity_compensation()
        if not result.success:
            raise RuntimeError(f"{result.stage}: {result.failure_reason}")

    def stop_gravity_compensation(self) -> None:
        if not self._gravity_comp_active:
            return
        result = self._mode_transition.exit_gravity_compensation()
        if not result.success:
            raise RuntimeError(f"{result.stage}: {result.failure_reason}")

    def feedback(self) -> FeedbackSample:
        if self.control_loop_active:
            positions, velocities, _effort = self.get_cached_joint_state()
        else:
            positions, velocities, _effort = self.get_joint_state()
        updated = self._feedback_coordinator.arm_updated_monotonic
        age_sec = float("inf") if updated is None else max(
            0.0, time.monotonic() - updated
        )
        return FeedbackSample(positions=positions, velocities=velocities, age_sec=age_sec)

    def gravity_torque(self, positions: np.ndarray) -> np.ndarray:
        torque = self._gravity_dynamics.gravity_torque(
            np.asarray(positions, dtype=np.float64)
        )
        return apply_gravity_compensation_tau_scale(torque)

    def preload_position_hold(self, target: np.ndarray) -> None:
        self._endpos_ctrl._q_target[:] = np.asarray(target, dtype=np.float64)

    def stop_control_loop(self) -> None:
        self._stop_control_loop()

    def switch_mode(self, mode: str, *, kp: float | None = None, kd: float | None = None) -> None:
        normalized = str(mode).strip().lower()
        if normalized == "mit":
            kp_values = np.full(self._arm.num_joints, float(kp or _GC_KP), dtype=np.float64)
            kd_values = np.full(self._arm.num_joints, float(kd or _GC_KD), dtype=np.float64)
            if not self._arm.mode_mit(kp=kp_values, kd=kd_values):
                raise RuntimeError("MIT mode switch failed")
        elif normalized == "pos_vel":
            if not self._arm.mode_pos_vel():
                raise RuntimeError("POS_VEL mode switch failed")
        else:
            raise ValueError(f"unsupported coordinated mode: {mode}")

    def send_mit(
        self,
        *,
        position: np.ndarray,
        kp: float,
        kd: float,
        torque: np.ndarray,
    ) -> None:
        self._arm.mit(
            pos=np.asarray(position, dtype=np.float64),
            vel=np.zeros(self._arm.num_joints, dtype=np.float64),
            kp=np.full(self._arm.num_joints, float(kp), dtype=np.float64),
            kd=np.full(self._arm.num_joints, float(kd), dtype=np.float64),
            tau=np.asarray(torque, dtype=np.float64),
        )

    def start_gravity_loop(self, target: np.ndarray) -> None:
        self._gravity_state.start(target)
        self._gravity_hardware_tick(self._arm, 1.0 / float(self._arm._rate))
        self._arm.start_control_loop(self._gravity_hardware_tick, rate=self._arm._rate)

    def finish_gravity_compensation(self) -> None:
        self._gravity_state.finish()

    def start_position_hold(
        self,
        target: np.ndarray,
        *,
        zero_velocity_limit: bool = False,
    ) -> None:
        self._start_pos_vel_loop(target=np.asarray(target, dtype=np.float64))
        if zero_velocity_limit:
            self._endpos_ctrl._vlim_override = np.zeros(
                self._arm.num_joints,
                dtype=np.float64,
            )

    def restore_position_velocity_limit(self) -> None:
        self._endpos_ctrl._vlim_override = None

    def disable_immediately(self) -> bool:
        """Best-effort emergency disable without claiming an unverified state."""
        with self._motor_lifecycle_lock:
            if not self._connected:
                self._enabled = False
                self._set_lifecycle_state("DISCONNECTED")
                return True
            was_enabled = self._enabled
            self._set_lifecycle_state("DISABLING")
            attempt = attempt_verified_disable(
                stop_control_loop=self._stop_control_loop,
                disable_all_motors=self._disable_all_motors,
                verify_disabled_feedback=self._verify_disabled_feedback,
            )
            if not attempt.verified:
                self._enabled = was_enabled
                message = "EMERGENCY_DISABLE_UNVERIFIED: " + attempt.error_detail
                if message not in self._error_codes:
                    self._error_codes.append(message)
                return False
            self._enabled = False
            self._state_machine = "IDLE"
            self._set_lifecycle_state("CONNECTED_DISABLED")
            return True

    def _on_mode_transition_stage(self, stage: str) -> None:
        if stage == "GRAVITY_COMP":
            self.set_state_machine("GRAVITY_COMP")
        elif stage in ("POS_VEL_HOLD", "TRANSITION_FAILED"):
            self.set_state_machine("IDLE")
        else:
            self.set_state_machine("MODE_TRANSITION")

    def gravity_compensation_active(self) -> bool:
        return self._gravity_state.active

    def gravity_compensation_target(self) -> np.ndarray | None:
        return self._gravity_state.target_copy()

    def _feedback_controller_groups(self):
        return build_controller_groups(
            self._arm,
            gripper_motor=self._gripper_mot,
            gripper_controller=self._gripper_ctrl,
        )

    @staticmethod
    def _validated_gripper_feedback_values(state) -> tuple[float, float, float, int]:
        return validated_gripper_feedback_values(
            state,
            coordinates=_GRIPPER_COORDINATES,
        )

    def _validate_feedback_state(self, label: str, state) -> None:
        validate_hardware_feedback_state(
            label,
            state,
            joint_position_limits_rad=_JOINT_FEEDBACK_LIMITS_RAD,
            gripper_coordinates=_GRIPPER_COORDINATES,
        )

    def _on_verified_feedback(
        self,
        label: str,
        state: object,
        _observed_at: float,
    ) -> None:
        if label == "gripper":
            position, velocity, torque, _status = self._validated_gripper_feedback_values(state)
            with self._gripper_lock:
                self._gripper_state.update_feedback(position, velocity, torque)

    def _verified_feedback_sample(self, label: str) -> VerifiedFeedbackSample:
        return self._feedback_coordinator.sample(label)

    def refresh_feedback_if_due(self, *, force: bool = False, now: float | None = None) -> bool:
        observed_at = time.monotonic() if now is None else float(now)
        control_thread = getattr(self._arm, "_ctrl_thread", None)
        if (
            self.control_loop_active
            and control_thread is not None
            and threading.current_thread() is not control_thread
        ):
            if force:
                raise RuntimeError(
                    "synchronous feedback refresh rejected while hardware loop owns bus"
                )
            return False
        return self._feedback_coordinator.refresh_if_due(
            force=force,
            now=observed_at,
        )

    def _arm_feedback_failure_reason(self, *, now: float | None = None) -> str | None:
        return self._feedback_coordinator.arm_failure_reason(now=now)

    def _gripper_feedback_failure_reason_locked(
        self,
        *,
        feedback_error: str | None,
        updated: float | None,
        now: float | None = None,
    ) -> str | None:
        zero_error = getattr(self, "_gripper_zero_error", None)
        if zero_error is not None:
            return zero_error
        if feedback_error is not None:
            return f"gripper feedback unavailable: {feedback_error}"
        current = time.monotonic() if now is None else float(now)
        age = float("inf") if updated is None else max(current - updated, 0.0)
        if age > self._feedback_stale_timeout_sec:
            return (
                f"gripper feedback stale: age={age:.3f}s "
                f"limit={self._feedback_stale_timeout_sec:.3f}s"
            )
        position = float(self._gripper_state.position)
        if not _GRIPPER_COORDINATES.accepts_feedback_angle(position):
            return (
                f"gripper coordinate invalid: raw={position:.6f} rad outside "
                f"[{_GRIPPER_COORDINATES.feedback_lower_rad:.6f}, "
                f"{_GRIPPER_COORDINATES.feedback_upper_rad:.6f}]"
            )
        return None

    def _gripper_feedback_failure_reason(self) -> str | None:
        feedback_error = self._feedback_coordinator.gripper_error
        updated = self._feedback_coordinator.gripper_updated_monotonic
        with self._gripper_lock:
            return self._gripper_feedback_failure_reason_locked(
                feedback_error=feedback_error,
                updated=updated,
            )

    def _validated_joint_feedback(
        self,
        *,
        expected_status: int | None = None,
        refresh: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[int]]:
        if refresh:
            self.refresh_feedback_if_due(force=True)
        failure = self._arm_feedback_failure_reason()
        if failure is not None:
            raise RuntimeError(failure)
        names = self.joint_names
        samples = [self._verified_feedback_sample(name) for name in names]
        positions: list[float] = []
        velocities: list[float] = []
        torques: list[float] = []
        statuses: list[int] = []
        for name, sample in zip(names, samples):
            self._validate_feedback_state(name, sample.state)
            status = int(sample.state.status_code)
            if expected_status is not None and status != expected_status:
                raise RuntimeError(
                    f"{name} status_code={status}, expected {expected_status}"
                )
            positions.append(float(sample.state.pos))
            velocities.append(float(sample.state.vel))
            torques.append(float(sample.state.torq))
            statuses.append(status)
        return (
            np.asarray(positions, dtype=np.float64),
            np.asarray(velocities, dtype=np.float64),
            np.asarray(torques, dtype=np.float64),
            statuses,
        )

    def _validated_gripper_status(self, expected_status: int | None = None) -> int | None:
        if self._gripper_mot is None:
            return None
        self.refresh_feedback_if_due(force=True)
        sample = self._verified_feedback_sample("gripper")
        _position, _velocity, _torque, status = self._validated_gripper_feedback_values(
            sample.state
        )
        if expected_status is not None and status != expected_status:
            raise RuntimeError(
                f"gripper status_code={status}, expected {expected_status}"
            )
        return status

    def _cached_gripper_status(self) -> int | None:
        if self._gripper_mot is None:
            return None
        sample = self._verified_feedback_sample("gripper")
        if sample.is_stale(time.monotonic(), self._feedback_stale_timeout_sec):
            raise RuntimeError("gripper feedback stale while verifying lifecycle state")
        _position, _velocity, _torque, status = self._validated_gripper_feedback_values(
            sample.state
        )
        return status

    def _disable_all_motors(self) -> None:
        errors: list[str] = []
        try:
            self._arm.disable()
        except Exception as exc:
            errors.append(f"arm disable: {exc}")
        if self._gripper_mot is not None and hasattr(self._gripper_mot, "disable"):
            try:
                self._gripper_mot.disable()
            except Exception as exc:
                errors.append(f"gripper disable: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def _verify_disabled_feedback(self) -> None:
        self._validated_joint_feedback(expected_status=0)
        self._validated_gripper_status(expected_status=0)

    def _refresh_arm_feedback(self) -> bool:
        return self.refresh_feedback_if_due(force=True)

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        if request:
            self.refresh_feedback_if_due(force=True)
        q, _velocity, _torque = self.get_cached_joint_state()
        ref = reference if reference is not None else self._gravity_state.last_position
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        return self._gravity_state.remember_position(q)

    def _gravity_comp_tick(self, arm, dt: float) -> None:
        del dt
        if not self._gravity_state.active or self._gravity_state.target is None:
            return

        q = self._read_gravity_comp_positions()
        _positions, qd, _torque = self.get_cached_joint_state()
        tau_g = self._gravity_dynamics.gravity_torque(q)
        tau_g = apply_gravity_compensation_tau_scale(tau_g)

        q_error = self._gravity_state.target - q
        integral = self._gravity_state.accumulate_error(q_error, template=q)

        linear_speed, angular_speed = self._gravity_dynamics.end_effector_speeds(
            q,
            qd,
        )

        self._gravity_state.observe_motion(
            q,
            moving=(
                linear_speed > _GC_VEL_THRESHOLD
                or angular_speed > _GC_W_VEL_THRESHOLD
            ),
        )

        arm.mit(
            pos=self._gravity_state.target,
            vel=np.zeros(arm.num_joints),
            kp=np.full(arm.num_joints, _GC_KP),
            kd=np.full(arm.num_joints, _GC_KD),
            tau=tau_g + integral,
            request_feedback=False,
        )

    def current_pose(self):
        from reBotArm_control_py.kinematics import compute_fk

        q, _, _ = self.get_joint_state()
        position, rotation, _ = compute_fk(self._endpos_ctrl._model, q)
        return fk_to_pose(position, rotation)

    def get_joint_status_codes(self) -> list[int]:
        if self._connected and self._arm_feedback_failure_reason() is not None:
            return [255] * len(self.joint_names)
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._verified_feedback_sample(name).state
                codes.append(int(st.status_code if st is not None else 255))
            except Exception:
                codes.append(255)
        return codes

    def init_gripper(self, cfg_path: str) -> None:
        adapter = GripperSdkAdapter(self._arm)
        self._gripper_cfg = adapter.load_config(cfg_path)
        controller = adapter.controller_for(self._gripper_cfg)
        self._gripper_mot = adapter.create_motor(controller, self._gripper_cfg)
        self._gripper_ctrl = controller
        adapter.share_controller_bus(controller, self._gripper_mot)
        # Connection only discovers hardware.  Mode selection, enabling, and
        # command-loop startup belong to the explicit enable transition.

    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        self._require_enabled()
        if not self.control_loop_active:
            raise RuntimeError("gripper command requires the unified hardware control loop")
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        gripper_failure = self._gripper_feedback_failure_reason()
        if gripper_failure is not None:
            raise RuntimeError(gripper_failure)
        position, effort_request = _GRIPPER_COORDINATES.validate_position_request(
            position_m,
            max_effort,
        )
        self._gripper_position.start(position, effort_request)

    def gripper_target_timeout_sec(self) -> float:
        with self._gripper_lock:
            return float(self._gripper_state.target_timeout_sec)

    @property
    def gripper_command_error(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_state.command_error

    def wait_gripper_target(self, timeout: float | None = None) -> bool:
        return self._gripper_position.wait(timeout)

    def set_gripper_position(self, position_m: float, max_effort: float = 0.0) -> tuple[bool, float]:
        self.set_gripper_target(position_m, max_effort)
        reached = self.wait_gripper_target()
        return reached, self.gripper_position_m()

    def grasp_gripper(
        self,
        close_force: float = _G_GRASP_CLOSE_FORCE_DEFAULT,
        hold_force: float = _G_GRASP_HOLD_FORCE_DEFAULT,
        close_timeout_sec: float = _G_GRASP_TIMEOUT,
        min_close_time_sec: float = _G_GRASP_MIN_CLOSE_TIME,
        velocity_threshold: float = _G_GRASP_VEL_THRESHOLD,
        min_closure_distance_m: float = _G_GRASP_MIN_CLOSURE_M,
        hold_timeout_sec: float | None = None,
    ) -> tuple[bool, bool, float, float, float, str]:
        self._require_enabled()
        if not self.control_loop_active:
            raise RuntimeError("gripper grasp requires the unified hardware control loop")
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        gripper_failure = self._gripper_feedback_failure_reason()
        if gripper_failure is not None:
            raise RuntimeError(gripper_failure)

        initial_sample = self._verified_feedback_sample("gripper")
        return self._gripper_grasp.execute(
            initial_sample,
            close_force=close_force,
            hold_force=hold_force,
            close_timeout_sec=close_timeout_sec,
            min_close_time_sec=min_close_time_sec,
            velocity_threshold=velocity_threshold,
            min_closure_distance_m=min_closure_distance_m,
            hold_timeout_sec=hold_timeout_sec,
        )

    def gripper_feedback_sample(self) -> VerifiedFeedbackSample:
        return self._verified_feedback_sample("gripper")

    def gripper_feedback_motion(self) -> tuple[float, float]:
        with self._gripper_lock:
            return self._gripper_state.velocity, self._gripper_state.torque

    def gripper_validated_feedback_values(
        self,
        state,
    ) -> tuple[float, float, float, int]:
        return self._validated_gripper_feedback_values(state)

    def gripper_command_canceled(self) -> bool:
        return self._gripper_command_cancel.is_set()

    def begin_gripper_position(
        self,
        *,
        start_angle: float,
        goal_angle: float,
        target_effort: float,
        now: float,
        timeout_sec: float,
    ) -> None:
        with self._gripper_lock:
            self._gripper_command_cancel.clear()
            self._gripper_state.start_position(
                start_angle=start_angle,
                goal_angle=goal_angle,
                target_effort=target_effort,
                now=now,
                timeout_sec=timeout_sec,
            )
        self._require_gripper_control_loop()

    def gripper_position_progress(self) -> GripperPositionProgress:
        with self._gripper_lock:
            return GripperPositionProgress(
                goal_angle_rad=self._gripper_state.goal_angle,
                position_angle_rad=self._gripper_state.position,
                deadline=self._gripper_state.target_deadline,
                active=self._gripper_state.active,
                result=self._gripper_state.position_result,
                canceled=self._gripper_command_cancel.is_set(),
            )

    def begin_gripper_grasp(self, close_force: float, hold_force: float) -> None:
        with self._gripper_lock:
            self._gripper_command_cancel.clear()
            self._gripper_state.start_grasp(
                close_force=close_force,
                hold_force=hold_force,
            )
        self._require_gripper_control_loop()

    def begin_gripper_hold(self, hold_force: float, deadline: float) -> None:
        with self._gripper_lock:
            self._gripper_state.start_hold(
                angle=self._gripper_state.position,
                force=hold_force,
                deadline=deadline,
            )

    def stop_gripper_motion(self, reason: str = "gripper command canceled") -> None:
        """Cancel the current task and queue a zero-torque command for the bus owner."""
        self._gripper_command_cancel.set()
        with self._gripper_lock:
            self._gripper_state.request_stop(reason)

    def cancel_gripper_position_command(self, reason: str = "position command canceled") -> bool:
        with self._gripper_lock:
            if not self._gripper_state.can_cancel_position():
                return False
        self.stop_gripper_motion(reason)
        return True

    def release_grasp_hold(self, reason: str = "external release") -> bool:
        with self._gripper_lock:
            if not self._gripper_state.can_release_grasp():
                return False
            self._gripper_state.hold_release_reason = str(reason)
        self.stop_gripper_motion(f"grasp release: {reason}")
        return True

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        if self._gripper_mot is None:
            return (
                self._gripper_state.position,
                self._gripper_state.velocity,
                self._gripper_state.torque,
                255,
            )
        try:
            sample = self._verified_feedback_sample("gripper")
            position, velocity, torque, status = self._validated_gripper_feedback_values(
                sample.state
            )
            if self._gripper_feedback_failure_reason() is not None:
                status = 255
            return position, velocity, torque, status
        except Exception:
            return (
                self._gripper_state.position,
                self._gripper_state.velocity,
                self._gripper_state.torque,
                255,
            )

    def gripper_position_m(self) -> float:
        with self._gripper_lock:
            position = float(self._gripper_state.position)
            zero_error = getattr(self, "_gripper_zero_error", None)
        if zero_error is not None:
            return float("nan")
        return _GRIPPER_COORDINATES.angle_to_opening(position)

    def gripper_reached_target(self) -> bool:
        return self._gripper_position.reached_target()

    def send_gripper_motor_cmd(self, cmd) -> None:
        self._require_enabled()
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        gripper_failure = self._gripper_feedback_failure_reason()
        if gripper_failure is not None:
            raise RuntimeError(gripper_failure)
        state = self._verified_feedback_sample("gripper").state
        motor_command = resolve_gripper_motor_command(
            cmd,
            feedback_state=state,
            config=self._gripper_cfg,
            open_soft_limit_rad=_GRIPPER_COORDINATES.open_soft_limit_rad,
            torque_limit_nm=_G_TAU_MAX,
        )
        dispatch_gripper_motor_command(self._gripper_mot, motor_command)
        with self._gripper_lock:
            self._gripper_state.set_idle()

    def _start_pos_vel_loop(self, target: np.ndarray | None = None) -> None:
        if self.control_loop_active:
            return
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.array(target, dtype=np.float64)
        self._arm.start_control_loop(self._endpos_hardware_tick)
        self._endpos_ctrl._running = True

    def _endpos_hardware_tick(self, arm, dt: float) -> None:
        self._hardware_control_tick(arm, dt, self._endpos_ctrl._loop_cb)

    def _gravity_hardware_tick(self, arm, dt: float) -> None:
        self._hardware_control_tick(arm, dt, self._gravity_comp_tick)

    def _hardware_control_tick(self, arm, dt: float, arm_callback) -> None:
        """Serialize arm commands, shared-bus feedback, and gripper commands."""
        try:
            self.refresh_feedback_if_due()
        except Exception:
            failure = self._arm_feedback_failure_reason()
            if failure is None:
                raise
            self._protective_disable_from_hardware_loop(failure)
            return
        failure = self._arm_feedback_failure_reason()
        if failure is not None:
            self._protective_disable_from_hardware_loop(failure)
            return
        try:
            arm_callback(arm, dt)
        except Exception:
            failure = self._arm_feedback_failure_reason()
            if failure is None:
                raise
            self._protective_disable_from_hardware_loop(failure)
            return
        self._gripper_tick()

    def _protective_disable_from_hardware_loop(self, reason: str) -> None:
        """Fail closed without joining the hardware thread from itself."""
        self._arm._running = False
        self._endpos_ctrl._running = False
        self._endpos_ctrl._stop_send.set()
        self._endpos_ctrl._moving = False
        self._gravity_state.deactivate()
        with self._gripper_lock:
            self._gripper_state.set_idle()
        self._state_machine = "IDLE"
        self._set_lifecycle_state("DISABLING")
        errors = disable_unique_controller_buses(
            getattr(self._arm, "_ctrl_map", {}).values()
        )
        if errors:
            self._error_codes.append(
                "FEEDBACK_PROTECTIVE_DISABLE_FAILED: " + "; ".join(errors)
            )
        # Do not claim disabled from command success alone.  A later external
        # cache refresh finalises CONNECTED_DISABLED only after every new
        # sequence reports status_code=0.
        if "FEEDBACK_PROTECTIVE_DISABLE" not in self._error_codes:
            self._error_codes.append("FEEDBACK_PROTECTIVE_DISABLE")
        _LOG.error("protective disable requested from hardware loop: %s", reason)

    def _stop_control_loop(self) -> None:
        self._arm.stop_control_loop()
        self._endpos_ctrl._running = False

    def _gripper_safe_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau_ff: float = 0.0,
        tau_limit: float = _G_TAU_MAX,
    ) -> None:
        if self._gripper_mot is None or self._gripper_ctrl is None:
            return
        send_safe_gripper_mit(
            self._gripper_mot,
            position_rad=pos,
            velocity_rad_s=vel,
            kp=kp,
            kd=kd,
            torque_feedforward_nm=tau_ff,
            torque_limit_nm=tau_limit,
            current_position_rad=self._gripper_state.position,
            current_velocity_rad_s=self._gripper_state.velocity,
            open_soft_limit_rad=_GRIPPER_COORDINATES.open_soft_limit_rad,
            maximum_torque_nm=_G_TAU_MAX,
        )

    def _gripper_tick(self) -> None:
        with self._gripper_lock:
            snapshot = self._gripper_state.snapshot()
            if snapshot.neutral_pending is not None:
                decision = decide_gripper_tick(
                    snapshot,
                    now=time.monotonic(),
                    config=self._gripper_motion_policy_config(),
                )
                self._apply_neutral_decision_locked(decision)
                return
        if not snapshot.active:
            return
        self._verified_feedback_sample("gripper")
        gripper_failure = self._gripper_feedback_failure_reason()
        if gripper_failure is not None:
            self.stop_gripper_motion(gripper_failure)
            return
        now = time.monotonic()
        with self._gripper_lock:
            decision = decide_gripper_tick(
                self._gripper_state.snapshot(),
                now=now,
                config=self._gripper_motion_policy_config(),
            )
            self._gripper_state.apply_tick_decision(decision, now=now)
            if decision.action == "queue_neutral":
                return
        if decision.action == "cancel":
            self.stop_gripper_motion(decision.reason)
            return
        if decision.action == "mit":
            self._gripper_safe_mit(
                decision.position_rad,
                0.0,
                decision.kp,
                decision.kd,
                decision.torque_ff_nm,
                tau_limit=decision.torque_limit_nm,
            )

    def _gripper_motion_policy_config(self) -> GripperMotionPolicyConfig:
        return GripperMotionPolicyConfig(
            arrive_tolerance_rad=_G_ARRIVE_TOL,
            position_max_speed_rad_s=self._gripper_position_max_speed_rad_s,
            move_kp=_G_KP_MOVE,
            move_kd=_G_KD_MOVE,
            grasp_close_kp=_G_GRASP_CLOSE_KP,
            grasp_close_kd=_G_GRASP_CLOSE_KD,
            grasp_hold_kp=_G_GRASP_HOLD_KP,
            grasp_hold_kd=_G_GRASP_HOLD_KD,
            default_torque_limit_nm=_G_TAU_MAX,
        )

    def _apply_neutral_decision_locked(self, decision: GripperTickDecision) -> None:
        self._gripper_safe_mit(
            decision.position_rad,
            0.0,
            decision.kp,
            decision.kd,
            decision.torque_ff_nm,
            tau_limit=decision.torque_limit_nm,
        )
        self._gripper_state.complete_neutral(decision)

    def _require_gripper_control_loop(self) -> None:
        if not self.control_loop_active:
            raise RuntimeError("gripper command requires the unified hardware control loop")
