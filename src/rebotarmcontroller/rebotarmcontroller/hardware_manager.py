from __future__ import annotations

import threading
import time
import sys
import logging
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml

from .command_arbiter import CommandArbiter
from .conversions import fk_to_pose
from .feedback_sequence import VerifiedFeedbackSample, sequence_advanced, validate_sequence
from .gripper_safety import is_gripper_contact_sample
from .mode_transition import ModeTransitionCoordinator
from .mode_transition_policy import (
    FeedbackSample,
    ModeTransitionConfig,
    validate_mode_transition,
)

_LOG = logging.getLogger(__name__)

_G_MAX_DIST_M = 0.09
_G_VERIFIED_OPEN_LIMIT_M = 0.085
_G_ANGLE_OPEN = -5.0
_G_COORDINATE_TOL_RAD = 2.0 * 25.0 / 65535.0
# One millimetre at the closed end is accepted only when validating feedback.
# It does not move the zero, enlarge the requested range, or change command clamping.
_G_CLOSED_FEEDBACK_TOL_RAD = 0.001 * abs(_G_ANGLE_OPEN) / _G_MAX_DIST_M
_G_ZERO_VERIFY_TIMEOUT_SEC = 0.5
_G_ZERO_VERIFY_SAMPLES = 3
_G_OPEN_SOFT_LIMIT = -4.9
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
_G_CTRL_RATE = 500.0
_HARDWARE_FEEDBACK_RATE_HZ = 50.0
_HARDWARE_FEEDBACK_RATE_MIN_HZ = 20.0
_HARDWARE_FEEDBACK_RATE_MAX_HZ = 100.0
_FEEDBACK_REFRESH_RETRIES = 3
_FEEDBACK_RETRY_INTERVAL_SEC = 0.005
_FEEDBACK_STALE_TIMEOUT_SEC = 0.15
_FEEDBACK_STALE_TIMEOUT_MAX_SEC = 2.0
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

_LIFECYCLE_STATES = {
    "DISCONNECTED",
    "CONNECTED_DISABLED",
    "ENABLING",
    "ENABLED_HOLD",
    "TRAJECTORY_RUNNING",
    "DISABLING",
}


def apply_gravity_compensation_tau_scale(tau: np.ndarray) -> np.ndarray:
    scaled = np.array(tau, dtype=np.float64, copy=True)
    if scaled.shape == _GC_TAU_SCALE.shape:
        scaled *= _GC_TAU_SCALE
    return scaled


class HardwareManager:
    """Owns the single RobotArm instance used by the ROS driver."""

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
        feedback_rate = float(hardware_feedback_rate_hz)
        if not np.isfinite(feedback_rate) or not (
            _HARDWARE_FEEDBACK_RATE_MIN_HZ
            <= feedback_rate
            <= _HARDWARE_FEEDBACK_RATE_MAX_HZ
        ):
            raise ValueError(
                "hardware_feedback_rate_hz must be finite and within "
                f"[{_HARDWARE_FEEDBACK_RATE_MIN_HZ:g}, "
                f"{_HARDWARE_FEEDBACK_RATE_MAX_HZ:g}] Hz"
            )
        stale_timeout = float(feedback_stale_timeout_sec)
        if not np.isfinite(stale_timeout) or not (
            0.05 <= stale_timeout <= _FEEDBACK_STALE_TIMEOUT_MAX_SEC
        ):
            raise ValueError(
                "feedback_stale_timeout_sec must be finite and within "
                f"[0.05, {_FEEDBACK_STALE_TIMEOUT_MAX_SEC:g}] s"
            )
        self._hardware_feedback_period_sec = 1.0 / feedback_rate
        self._feedback_stale_timeout_sec = stale_timeout
        torque_cap = float(gripper_position_torque_cap_nm)
        if not np.isfinite(torque_cap) or not 0.05 <= torque_cap <= _G_TAU_MAX:
            raise ValueError("gripper_position_torque_cap_nm must be within [0.05, 1.5]")
        position_speed = float(gripper_position_max_speed_rad_s)
        if not np.isfinite(position_speed) or not 0.05 <= position_speed <= 3.0:
            raise ValueError(
                "gripper_position_max_speed_rad_s must be within [0.05, 3.0]"
            )
        timeout_margin = float(gripper_position_timeout_margin_sec)
        if not np.isfinite(timeout_margin) or not 0.1 <= timeout_margin <= 10.0:
            raise ValueError(
                "gripper_position_timeout_margin_sec must be within [0.1, 10.0]"
            )
        hold_timeout = float(grasp_hold_timeout_sec)
        if not np.isfinite(hold_timeout) or not 0.1 <= hold_timeout <= _G_GRASP_HOLD_TIMEOUT_MAX_SEC:
            raise ValueError("grasp_hold_timeout_sec must be within [0.1, 120.0]")
        self._gripper_position_torque_cap_nm = torque_cap
        self._gripper_position_max_speed_rad_s = position_speed
        self._gripper_position_timeout_margin_sec = timeout_margin
        self._grasp_hold_timeout_sec = hold_timeout
        contact_torque = float(gripper_contact_torque_min_nm)
        if not np.isfinite(contact_torque) or not 0.0 <= contact_torque <= _G_TAU_MAX:
            raise ValueError("gripper_contact_torque_min_nm must be within [0.0, 1.5]")
        self._gripper_contact_torque_min_nm = contact_torque
        self._sdk_root = self._ensure_rebot_sdk_in_syspath()

        from reBotArm_control_py.actuator import RobotArm
        from reBotArm_control_py.controllers import ArmEndPos
        from reBotArm_control_py.kinematics import load_robot_model
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        import pinocchio as pin

        cfg_path = Path(arm_cfg).expanduser() if arm_cfg else self.default_arm_cfg()
        cfg_path = self._arm_cfg_with_channel(cfg_path, channel)
        self._arm = RobotArm(cfg_path=str(cfg_path))
        self._gc_model = load_robot_model()
        self._gc_data = self._gc_model.createData()
        self._gc_ee_frame_id = self._gc_model.getFrameId(_GC_EE_FRAME)
        self._gc_compute_generalized_gravity = compute_generalized_gravity
        self._gc_pin = pin

        self._gripper_cfg_path = (
            Path(gripper_cfg).expanduser() if gripper_cfg else self.default_gripper_cfg()
        )
        self._gripper_cfg = None
        self._gripper_mot = None
        self._gripper_ctrl = None
        self._gripper_target_angle = 0.0
        self._gripper_goal_angle = 0.0
        self._gripper_target_effort = _G_DEFAULT_FORCE
        self._gripper_close_force = _G_GRASP_CLOSE_FORCE_DEFAULT
        self._gripper_hold_force = _G_GRASP_HOLD_FORCE_DEFAULT
        self._gripper_hold_angle = 0.0
        self._gripper_hold_deadline: float | None = None
        self._gripper_hold_release_reason: str | None = None
        self._gripper_mode = "idle"
        self._gripper_active = False
        self._gripper_pos = 0.0
        self._gripper_vel = 0.0
        self._gripper_torque = 0.0
        self._gripper_loop_stop = threading.Event()
        self._gripper_command_cancel = threading.Event()
        self._gripper_command_error: str | None = None
        self._gripper_position_result = "idle"
        self._gripper_target_timeout_sec = 0.0
        self._gripper_target_deadline_monotonic: float | None = None
        self._gripper_last_tick_monotonic: float | None = None
        self._gripper_neutral_pending: tuple[float, str, bool] | None = None
        self._gripper_loop_thread: threading.Thread | None = None
        self._gripper_loop_running = False
        self._gripper_lock = threading.RLock()

        self._feedback_lock = threading.RLock()
        self._verified_feedback_by_label: dict[str, VerifiedFeedbackSample] = {}
        self._feedback_request_baseline_by_label: dict[str, int] = {}
        self._feedback_request_deadline_by_label: dict[str, float] = {}
        self._feedback_error_by_label: dict[str, str] = {}
        self._feedback_next_refresh_monotonic: float | None = None
        self._arm_feedback_updated_monotonic: float | None = None
        self._arm_feedback_error: str | None = "arm feedback not received"
        self._gripper_feedback_updated_monotonic: float | None = None
        self._gripper_feedback_error: str | None = "gripper feedback not received"
        self._gripper_zero_error: str | None = None
        self._motor_lifecycle_lock = threading.RLock()

        self._endpos_ctrl = ArmEndPos(self._arm)
        self._connected = False
        self._enabled = False
        self._lifecycle_state = "DISCONNECTED"
        self._state_machine = "IDLE"
        self.command_arbiter = CommandArbiter()
        self._error_codes: list[str] = []
        self._gravity_comp_active = False
        self._gravity_comp_q_target: np.ndarray | None = None
        self._gravity_comp_integral: np.ndarray | None = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last: np.ndarray | None = None

        self._mode_transition_config = mode_transition_config or ModeTransitionConfig()
        self._mode_transition = ModeTransitionCoordinator(
            self,
            self._mode_transition_config,
            control_period_sec=1.0 / float(getattr(self._arm, "_rate", 500.0)),
            on_stage=self._on_mode_transition_stage,
        )

        self._patch_arm_bus_lock()

    def default_arm_cfg(self) -> Path:
        return self._sdk_root / "config" / "arm.yaml"

    def default_gripper_cfg(self) -> Path:
        return self._sdk_root / "config" / "gripper.yaml"

    @staticmethod
    def _workspace_root() -> Path:
        return Path(__file__).resolve().parents[3]

    @classmethod
    def _sdk_candidates(cls) -> list[Path]:
        workspace = cls._workspace_root()
        return [
            workspace / "third_party" / "reBotArm_control_py",
            workspace / "third_party" / "reBotArm_control_py-main",
            workspace / "sdk" / "reBotArm_control_py",
            Path.cwd() / "third_party" / "reBotArm_control_py",
            Path.cwd() / "sdk" / "reBotArm_control_py",
            Path.home() / "robotarm_ros2" / "third_party" / "reBotArm_control_py",
            Path.home() / "robotarm_ros2" / "sdk" / "reBotArm_control_py",
            Path.home() / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
        ]

    @classmethod
    def _ensure_rebot_sdk_in_syspath(cls) -> Path:
        for root in cls._sdk_candidates():
            if (root / "reBotArm_control_py").is_dir():
                root_str = str(root)
                if root_str not in sys.path:
                    sys.path.insert(0, root_str)
                return root
        candidates = "\n".join(f"  - {path}" for path in cls._sdk_candidates())
        raise FileNotFoundError(
            "Cannot find reBotArm_control_py. Clone it into one of:\n"
            f"{candidates}"
        )

    @staticmethod
    def _arm_cfg_with_channel(cfg_path: Path, channel: str) -> Path:
        normalized_channel = str(channel or "").strip()
        if not normalized_channel or normalized_channel.lower() == "auto":
            return cfg_path
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        data["channel"] = normalized_channel
        tmp_dir = Path("/tmp") / "rebotarm_ros2"
        tmp_dir.mkdir(parents=True, exist_ok=True)
        tmp_path = tmp_dir / "arm_channel_override.yaml"
        with open(tmp_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return tmp_path

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
        return self._enabled

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def control_loop_active(self) -> bool:
        return bool(self._arm.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return self._gripper_mot is not None

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def lifecycle_state(self) -> str:
        return self._lifecycle_state

    @property
    def ready_for_motion(self) -> bool:
        """Return whether new arm or gripper motion goals may be accepted."""
        return bool(
            self._connected
            and self._enabled
            and self._lifecycle_state in ("ENABLED_HOLD", "TRAJECTORY_RUNNING")
        )

    @property
    def gripper_active(self) -> bool:
        with self._gripper_lock:
            return bool(self._gripper_active)

    @property
    def gripper_mode(self) -> str:
        with self._gripper_lock:
            return str(self._gripper_mode)

    @property
    def error_codes(self) -> list[str]:
        codes = list(self._error_codes)
        if not getattr(self, "_connected", False):
            return codes
        arm_failure = self._arm_feedback_failure_reason()
        if arm_failure is not None:
            codes.append(f"ARM_FEEDBACK: {arm_failure}")
        if self._gripper_mot is not None:
            with self._gripper_lock:
                gripper_failure = self._gripper_feedback_failure_reason_locked()
            if gripper_failure is not None:
                codes.append(f"GRIPPER_FEEDBACK: {gripper_failure}")
        return codes

    def set_state_machine(self, state: str) -> None:
        if state not in (
            "IDLE",
            "TRAJ_RUNNING",
            "LOWLEVEL_STREAMING",
            "GRAVITY_COMP",
            "MODE_TRANSITION",
        ):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state
        lifecycle_can_move = bool(
            self._connected
            and self._enabled
            and self._lifecycle_state not in ("DISABLING", "DISCONNECTED")
        )
        if state == "TRAJ_RUNNING" and lifecycle_can_move:
            self._set_lifecycle_state("TRAJECTORY_RUNNING")
        elif state == "IDLE" and lifecycle_can_move:
            self._set_lifecycle_state("ENABLED_HOLD")

    def _set_lifecycle_state(self, state: str) -> None:
        if state not in _LIFECYCLE_STATES:
            raise ValueError(f"unsupported lifecycle state: {state}")
        self._lifecycle_state = state

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("hardware is not connected")

    def _require_enabled(self) -> None:
        self._require_connected()
        if not self._enabled:
            raise RuntimeError("hardware is disabled; call explicit enable first")

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
        errors: list[str] = []
        try:
            self._stop_control_loop()
        except Exception as exc:
            errors.append(f"stop control loop: {exc}")
        try:
            self._disable_all_motors()
        except Exception as exc:
            errors.append(f"disable command: {exc}")
        if not errors:
            try:
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
            except Exception as exc:
                errors.append(f"disabled feedback: {exc}")
        self._state_machine = "IDLE"
        if errors:
            self._enabled = True
            self._error_codes.append("ENABLE_ROLLBACK_FAILED")
            self._set_lifecycle_state("DISABLING")
            return "; ".join(errors)
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
                self._validated_joint_feedback(expected_status=0)
                self._validated_gripper_status(expected_status=0)
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
        self._stop_gripper_loop()
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
                    if abs(position) <= _G_COORDINATE_TOL_RAD
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
                f"+/-{_G_COORDINATE_TOL_RAD:.6f} rad"
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

        mot = self._arm._motor_map[joint_name]
        jc = next(j for j in self._arm._joints if j.name == joint_name)
        state = self._verified_feedback_sample(joint_name).state

        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(jc.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(jc.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(jc.vlim)

        values = {"pos": pos, "vel": vel, "kp": kp, "kd": kd, "tau": tau, "vlim": vlim}
        invalid = [name for name, value in values.items() if not np.isfinite(value)]
        if invalid:
            raise ValueError("joint motor command contains non-finite " + ", ".join(invalid))
        if kp < 0.0 or kd < 0.0 or vlim <= 0.0:
            raise ValueError("joint motor kp/kd must be non-negative and vlim must be positive")
        lower, upper = _JOINT_FEEDBACK_LIMITS_RAD[joint_name]
        if cmd.use_pos and not lower <= pos <= upper:
            raise ValueError(
                f"{joint_name} position command {pos:.6f} outside [{lower:.6f}, {upper:.6f}]"
            )

        if int(cmd.mode) == 0:
            mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(mot, "send_vel"):
                raise RuntimeError(f"{joint_name} does not support send_vel")
            mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
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
        updated = self._arm_feedback_updated_monotonic
        age_sec = float("inf") if updated is None else max(
            0.0, time.monotonic() - updated
        )
        return FeedbackSample(positions=positions, velocities=velocities, age_sec=age_sec)

    def gravity_torque(self, positions: np.ndarray) -> np.ndarray:
        torque = self._gc_compute_generalized_gravity(q=np.asarray(positions, dtype=np.float64))
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
        self._gravity_comp_q_target = np.asarray(target, dtype=np.float64).copy()
        self._gravity_comp_q_last = self._gravity_comp_q_target.copy()
        self._gravity_comp_integral = np.zeros_like(self._gravity_comp_q_target)
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_active = True
        self._gravity_hardware_tick(self._arm, 1.0 / float(self._arm._rate))
        self._arm.start_control_loop(self._gravity_hardware_tick, rate=self._arm._rate)

    def finish_gravity_compensation(self) -> None:
        self._gravity_comp_active = False
        self._gravity_comp_q_target = None
        self._gravity_comp_integral = None
        self._gravity_comp_lock_counter = 0
        self._gravity_comp_q_last = None

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
            errors: list[str] = []
            try:
                self._stop_control_loop()
            except Exception as exc:
                errors.append(f"stop control loop: {exc}")
            try:
                self._disable_all_motors()
            except Exception as exc:
                errors.append(f"disable command: {exc}")
            if not errors:
                try:
                    self._validated_joint_feedback(expected_status=0)
                    self._validated_gripper_status(expected_status=0)
                except Exception as exc:
                    errors.append(f"disabled feedback: {exc}")
            if errors:
                self._enabled = was_enabled
                message = "EMERGENCY_DISABLE_UNVERIFIED: " + "; ".join(errors)
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
        return self._gravity_comp_active

    def gravity_compensation_target(self) -> np.ndarray | None:
        if self._gravity_comp_q_target is None:
            return None
        return self._gravity_comp_q_target.copy()

    def _feedback_controller_groups(self):
        groups: list[tuple[object, list[tuple[str, object]]]] = []

        def add(controller, label: str, motor) -> None:
            for existing, entries in groups:
                if existing is controller:
                    entries.append((label, motor))
                    return
            groups.append((controller, [(label, motor)]))

        controller_map = getattr(self._arm, "_ctrl_map", {})
        motor_map = getattr(self._arm, "_motor_map", {})
        for joint in getattr(self._arm, "_joints", []):
            label = str(joint.name)
            controller = controller_map.get(getattr(joint, "vendor", None))
            motor = motor_map.get(label)
            if controller is None or motor is None:
                raise RuntimeError(f"{label} feedback hardware unavailable")
            add(controller, label, motor)
        if self._gripper_mot is not None:
            if self._gripper_ctrl is None:
                raise RuntimeError("gripper feedback controller unavailable")
            add(self._gripper_ctrl, "gripper", self._gripper_mot)
        if not groups:
            raise RuntimeError("hardware feedback controller map unavailable")
        return groups

    @staticmethod
    def _state_with_sequence(label: str, motor) -> tuple[object, int]:
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

    @staticmethod
    def _validated_gripper_feedback_values(state) -> tuple[float, float, float, int]:
        if state is None:
            raise RuntimeError("gripper feedback unavailable")
        values = (float(state.pos), float(state.vel), float(state.torq))
        if not all(np.isfinite(value) for value in values):
            raise RuntimeError("gripper feedback contains non-finite values")
        position, velocity, torque = values
        if not (_G_ANGLE_OPEN - _G_COORDINATE_TOL_RAD <= position <= _G_CLOSED_FEEDBACK_TOL_RAD):
            raise RuntimeError(
                f"gripper coordinate invalid: {position:.6f} rad outside feedback range "
                f"[{_G_ANGLE_OPEN:.6f}, {_G_CLOSED_FEEDBACK_TOL_RAD:.6f}]"
            )
        return position, velocity, torque, int(state.status_code)

    def _validate_feedback_state(self, label: str, state) -> None:
        if label == "gripper":
            self._validated_gripper_feedback_values(state)
            return
        if state is None:
            raise RuntimeError(f"{label} feedback unavailable")
        values = (float(state.pos), float(state.vel), float(state.torq))
        if not all(np.isfinite(value) for value in values):
            raise RuntimeError(f"{label} feedback contains non-finite values")
        limits = _JOINT_FEEDBACK_LIMITS_RAD.get(label)
        if limits is None:
            raise RuntimeError(f"no feedback limit configured for {label}")
        if not limits[0] <= values[0] <= limits[1]:
            raise RuntimeError(
                f"{label} position {values[0]:.6f} rad outside feedback range "
                f"[{limits[0]:.6f}, {limits[1]:.6f}]"
            )

    def _record_verified_feedback(
        self, label: str, state, sequence: int, observed_at: float
    ) -> None:
        self._validate_feedback_state(label, state)
        sample = VerifiedFeedbackSample(state, sequence, observed_at)
        self._verified_feedback_by_label[label] = sample
        self._feedback_error_by_label.pop(label, None)
        if label == "gripper":
            position, velocity, torque, _status = self._validated_gripper_feedback_values(state)
            with self._gripper_lock:
                self._gripper_pos = position
                self._gripper_vel = velocity
                self._gripper_torque = torque
                self._gripper_feedback_updated_monotonic = observed_at
                self._gripper_feedback_error = None

    def _verified_feedback_sample(self, label: str) -> VerifiedFeedbackSample:
        with self._feedback_lock:
            sample = self._verified_feedback_by_label.get(label)
        if sample is None:
            raise RuntimeError(f"{label} verified feedback unavailable")
        return sample

    def _feedback_response_window_sec(self) -> float:
        return max(
            self._hardware_feedback_period_sec * _FEEDBACK_REFRESH_RETRIES,
            _FEEDBACK_RETRY_INTERVAL_SEC * _FEEDBACK_REFRESH_RETRIES,
        )

    def _ensure_feedback_request_state(self) -> None:
        if not hasattr(self, "_feedback_request_baseline_by_label"):
            self._feedback_request_baseline_by_label = {}
        if not hasattr(self, "_feedback_request_deadline_by_label"):
            self._feedback_request_deadline_by_label = {}

    def _inspect_pending_feedback(
        self,
        observations: dict[str, tuple[object, int]],
        *,
        observed_at: float,
    ) -> None:
        self._ensure_feedback_request_state()
        for label, baseline in list(
            self._feedback_request_baseline_by_label.items()
        ):
            observation = observations.get(label)
            if observation is None:
                continue
            state, sequence = observation
            if sequence_advanced(sequence, baseline):
                try:
                    self._record_verified_feedback(
                        label,
                        state,
                        sequence,
                        observed_at,
                    )
                except Exception as exc:
                    self._feedback_error_by_label[label] = (
                        f"{label} feedback invalid: {exc}"
                    )
                self._feedback_request_baseline_by_label.pop(label, None)
                self._feedback_request_deadline_by_label.pop(label, None)
                continue
            deadline = self._feedback_request_deadline_by_label[label]
            if observed_at >= deadline:
                self._feedback_error_by_label[label] = (
                    f"{label} feedback deadline expired: sequence did not advance "
                    f"beyond baseline={baseline}"
                )
                self._feedback_request_baseline_by_label.pop(label, None)
                self._feedback_request_deadline_by_label.pop(label, None)

    def _read_feedback_observations(
        self,
        groups,
    ) -> dict[str, tuple[object, int]]:
        observations: dict[str, tuple[object, int]] = {}
        for _controller, entries in groups:
            for label, motor in entries:
                observations[label] = self._state_with_sequence(label, motor)
        return observations

    def _refresh_feedback_batch(
        self,
        *,
        observed_at: float,
        inspect_after_poll: bool = False,
    ) -> None:
        self._ensure_feedback_request_state()
        groups = self._feedback_controller_groups()
        observations: dict[str, tuple[object, int]] = {}
        group_errors: list[str] = []
        readable_groups = []
        for controller, entries in groups:
            try:
                for label, motor in entries:
                    observations[label] = self._state_with_sequence(label, motor)
                readable_groups.append((controller, entries))
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = f"controller={type(controller).__name__} motors={labels}: {exc}"
                group_errors.append(message)
                for label, _motor in entries:
                    self._feedback_error_by_label[label] = (
                        f"shared feedback batch failed before request: {message}"
                    )

        self._inspect_pending_feedback(observations, observed_at=observed_at)
        response_window = self._feedback_response_window_sec()
        for _controller, entries in readable_groups:
            for label, _motor in entries:
                if label not in self._feedback_request_baseline_by_label:
                    _state, sequence = observations[label]
                    self._feedback_request_baseline_by_label[label] = sequence
                    self._feedback_request_deadline_by_label[label] = (
                        observed_at + response_window
                    )

        successful_groups = []
        for controller, entries in readable_groups:
            lock = getattr(controller, "_bus_lock", None)

            def transaction() -> None:
                for _label, motor in entries:
                    motor.request_feedback()
                controller.poll_feedback_once()

            try:
                if lock is None:
                    transaction()
                else:
                    with lock:
                        transaction()
                successful_groups.append((controller, entries))
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = f"controller={type(controller).__name__} motors={labels}: {exc}"
                group_errors.append(message)
                for label, _motor in entries:
                    self._feedback_error_by_label[label] = (
                        f"shared feedback batch failed: {message}"
                    )
                    self._feedback_request_baseline_by_label.pop(label, None)
                    self._feedback_request_deadline_by_label.pop(label, None)

        if inspect_after_poll:
            completed_at = time.monotonic()
            observations = {}
            for controller, entries in successful_groups:
                try:
                    for label, motor in entries:
                        observations[label] = self._state_with_sequence(label, motor)
                except Exception as exc:
                    labels = ",".join(label for label, _motor in entries)
                    message = (
                        f"controller={type(controller).__name__} motors={labels}: {exc}"
                    )
                    group_errors.append(message)
                    for label, _motor in entries:
                        self._feedback_error_by_label[label] = message
            self._inspect_pending_feedback(
                observations,
                observed_at=completed_at,
            )
        self._sync_feedback_health()
        if group_errors:
            raise RuntimeError("shared feedback batch failed: " + "; ".join(group_errors))

    def _force_feedback_refresh(self) -> None:
        self._ensure_feedback_request_state()
        groups = self._feedback_controller_groups()
        initial: dict[str, tuple[object, int]] = {}
        initial_errors: list[str] = []
        for controller, entries in groups:
            try:
                for label, motor in entries:
                    initial[label] = self._state_with_sequence(label, motor)
            except Exception as exc:
                labels = ",".join(label for label, _motor in entries)
                message = f"controller={type(controller).__name__} motors={labels}: {exc}"
                initial_errors.append(message)
                for label, _motor in entries:
                    self._feedback_error_by_label[label] = message
        if initial_errors:
            self._sync_feedback_health()
            raise RuntimeError(
                "forced feedback baseline failed before request: "
                + "; ".join(initial_errors)
            )

        required_baselines = {
            label: sequence for label, (_state, sequence) in initial.items()
        }
        prior_samples = {
            label: self._verified_feedback_by_label.get(label)
            for label in required_baselines
        }

        def forced_sample_satisfies(label: str, baseline: int) -> bool:
            sample = self._verified_feedback_by_label.get(label)
            return bool(
                label not in self._feedback_error_by_label
                and sample is not None
                and sample is not prior_samples[label]
                and sequence_advanced(sample.sequence, baseline)
            )

        for label in required_baselines:
            self._feedback_request_baseline_by_label.pop(label, None)
            self._feedback_request_deadline_by_label.pop(label, None)

        last_error: Exception | None = None
        for attempt in range(_FEEDBACK_REFRESH_RETRIES):
            attempt_error: Exception | None = None
            try:
                self._refresh_feedback_batch(
                    observed_at=time.monotonic(),
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
            if attempt + 1 < _FEEDBACK_REFRESH_RETRIES:
                time.sleep(_FEEDBACK_RETRY_INTERVAL_SEC)

        missing: list[str] = []
        for label, baseline in required_baselines.items():
            if not forced_sample_satisfies(label, baseline):
                reason = self._feedback_error_by_label.get(label)
                if reason is None:
                    reason = (
                        f"{label} feedback timeout: sequence did not advance "
                        f"beyond baseline={baseline}"
                    )
                self._feedback_error_by_label[label] = reason
                missing.append(reason)
            self._feedback_request_baseline_by_label.pop(label, None)
            self._feedback_request_deadline_by_label.pop(label, None)
        self._sync_feedback_health()
        if last_error is not None:
            raise last_error
        raise RuntimeError("fresh hardware feedback unavailable: " + "; ".join(missing))

    def _sync_feedback_health(self) -> None:
        labels = list(self.joint_names)
        errors = [self._feedback_error_by_label[x] for x in labels if x in self._feedback_error_by_label]
        missing = [x for x in labels if x not in self._verified_feedback_by_label]
        if errors:
            self._arm_feedback_error = "; ".join(errors)
        elif missing:
            self._arm_feedback_error = "verified feedback pending: " + ",".join(missing)
        else:
            self._arm_feedback_error = None
            self._arm_feedback_updated_monotonic = min(
                self._verified_feedback_by_label[x].observed_at for x in labels
            )
        if self._gripper_mot is not None:
            self._gripper_feedback_error = self._feedback_error_by_label.get("gripper")
            if "gripper" not in self._verified_feedback_by_label:
                self._gripper_feedback_error = "gripper feedback not received"

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
        with self._feedback_lock:
            due = self._feedback_next_refresh_monotonic
            if not force and due is not None and observed_at < due:
                return False
            self._feedback_next_refresh_monotonic = (
                observed_at + self._hardware_feedback_period_sec
            )
            try:
                if force:
                    self._force_feedback_refresh()
                else:
                    self._refresh_feedback_batch(observed_at=observed_at)
                return True
            except Exception:
                if force:
                    raise
                return False

    def _arm_feedback_failure_reason(self, *, now: float | None = None) -> str | None:
        if self._arm_feedback_error:
            return f"arm feedback unavailable: {self._arm_feedback_error}"
        current = time.monotonic() if now is None else float(now)
        updated = self._arm_feedback_updated_monotonic
        age = float("inf") if updated is None else max(current - updated, 0.0)
        if age > self._feedback_stale_timeout_sec:
            return (
                f"arm feedback stale: age={age:.3f}s "
                f"limit={self._feedback_stale_timeout_sec:.3f}s"
            )
        return None

    def _gripper_feedback_failure_reason_locked(
        self,
        *,
        now: float | None = None,
    ) -> str | None:
        zero_error = getattr(self, "_gripper_zero_error", None)
        if zero_error is not None:
            return zero_error
        feedback_error = getattr(self, "_gripper_feedback_error", None)
        if feedback_error is not None:
            return f"gripper feedback unavailable: {feedback_error}"
        current = time.monotonic() if now is None else float(now)
        updated = getattr(self, "_gripper_feedback_updated_monotonic", None)
        age = float("inf") if updated is None else max(current - updated, 0.0)
        if age > self._feedback_stale_timeout_sec:
            return (
                f"gripper feedback stale: age={age:.3f}s "
                f"limit={self._feedback_stale_timeout_sec:.3f}s"
            )
        position = float(self._gripper_pos)
        if not np.isfinite(position) or not (
            _G_ANGLE_OPEN - _G_COORDINATE_TOL_RAD
            <= position
            <= _G_CLOSED_FEEDBACK_TOL_RAD
        ):
            return (
                f"gripper coordinate invalid: raw={position:.6f} rad outside "
                f"[{_G_ANGLE_OPEN - _G_COORDINATE_TOL_RAD:.6f}, "
                f"{_G_CLOSED_FEEDBACK_TOL_RAD:.6f}]"
            )
        return None

    def _gripper_feedback_failure_reason(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_feedback_failure_reason_locked()

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
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy()

    def _gravity_comp_tick(self, arm, dt: float) -> None:
        del dt
        if not self._gravity_comp_active or self._gravity_comp_q_target is None:
            return

        q = self._read_gravity_comp_positions()
        _positions, qd, _torque = self.get_cached_joint_state()
        tau_g = self._gc_compute_generalized_gravity(q=q)
        tau_g = apply_gravity_compensation_tau_scale(tau_g)

        q_error = self._gravity_comp_q_target - q
        if self._gravity_comp_integral is None:
            self._gravity_comp_integral = np.zeros_like(q)
        self._gravity_comp_integral += q_error * 1.0
        np.clip(self._gravity_comp_integral, -0.5, 0.5, out=self._gravity_comp_integral)

        self._gc_pin.computeJointJacobians(self._gc_model, self._gc_data, q)
        self._gc_pin.updateFramePlacements(self._gc_model, self._gc_data)
        jacobian = self._gc_pin.getFrameJacobian(
            self._gc_model,
            self._gc_data,
            self._gc_ee_frame_id,
            self._gc_pin.ReferenceFrame.WORLD,
        )
        spatial_velocity = jacobian @ qd
        linear_speed = float(np.linalg.norm(spatial_velocity[:3]))
        angular_speed = float(np.linalg.norm(spatial_velocity[3:]))

        if linear_speed > _GC_VEL_THRESHOLD or angular_speed > _GC_W_VEL_THRESHOLD:
            self._gravity_comp_q_target = q.copy()
            self._gravity_comp_lock_counter = 0
            self._gravity_comp_integral *= 0.9
        else:
            self._gravity_comp_lock_counter += 1

        arm.mit(
            pos=self._gravity_comp_q_target,
            vel=np.zeros(arm.num_joints),
            kp=np.full(arm.num_joints, _GC_KP),
            kd=np.full(arm.num_joints, _GC_KD),
            tau=tau_g + self._gravity_comp_integral,
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
        from reBotArm_control_py.actuator.gripper import load_cfg as load_gripper_cfg

        gcfg = load_gripper_cfg(cfg_path)
        gc = gcfg["gripper"]
        self._gripper_cfg = gc

        vendor = gc.vendor
        if vendor not in self._arm._ctrl_map:
            raise RuntimeError(
                f"gripper vendor={vendor!r} cannot share the arm Controller"
            )
        ctrl = self._arm._ctrl_map[vendor]

        if vendor == "damiao":
            self._gripper_mot = ctrl.add_damiao_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "myactuator":
            self._gripper_mot = ctrl.add_myactuator_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "robstride":
            self._gripper_mot = ctrl.add_robstride_motor(gc.motor_id, gc.feedback_id, gc.model)
        else:
            raise ValueError(f"unsupported gripper vendor: {vendor!r}")

        self._gripper_ctrl = ctrl

        self._patch_controller_bus(ctrl)
        self._wrap_motor_bus(self._gripper_mot, ctrl._bus_lock)
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
        position = float(position_m)
        effort_request = float(max_effort)
        if not np.isfinite(position) or not np.isfinite(effort_request):
            raise ValueError("gripper position and effort must be finite")
        if not 0.0 <= position <= _G_VERIFIED_OPEN_LIMIT_M:
            raise ValueError(
                "gripper position must be within "
                f"[0.0, {_G_VERIFIED_OPEN_LIMIT_M:g}] m"
            )
        sample = self._verified_feedback_sample("gripper")
        if sample.is_stale(time.monotonic(), self._feedback_stale_timeout_sec):
            raise RuntimeError("gripper feedback is stale before position command")
        start_angle, _velocity, _torque, status = self._validated_gripper_feedback_values(
            sample.state
        )
        if status != 1:
            raise RuntimeError(f"gripper status_code={status}, expected 1")
        target = max((position / _G_MAX_DIST_M) * _G_ANGLE_OPEN, _G_OPEN_SOFT_LIMIT)
        effort = _G_DEFAULT_FORCE if effort_request <= 0.0 else effort_request
        now = time.monotonic()
        dynamic_timeout = (
            abs(target - start_angle) / self._gripper_position_max_speed_rad_s
            + self._gripper_position_timeout_margin_sec
        )
        with self._gripper_lock:
            self._gripper_command_cancel.clear()
            self._gripper_target_angle = start_angle
            self._gripper_goal_angle = target
            self._gripper_target_effort = float(
                np.clip(effort, 0.05, self._gripper_position_torque_cap_nm)
            )
            self._gripper_mode = "position"
            self._gripper_active = True
            self._gripper_position_result = "active"
            self._gripper_command_error = None
            self._gripper_last_tick_monotonic = now
            self._gripper_target_timeout_sec = dynamic_timeout
            self._gripper_target_deadline_monotonic = now + dynamic_timeout
            self._gripper_neutral_pending = None
        self._start_gripper_loop()

    def gripper_target_timeout_sec(self) -> float:
        with self._gripper_lock:
            return float(self._gripper_target_timeout_sec)

    @property
    def gripper_command_error(self) -> str | None:
        with self._gripper_lock:
            return self._gripper_command_error

    def wait_gripper_target(self, timeout: float | None = None) -> bool:
        with self._gripper_lock:
            owned_goal = self._gripper_goal_angle
            deadline = self._gripper_target_deadline_monotonic
        if deadline is None:
            return False
        if timeout is not None:
            explicit = time.monotonic() + max(float(timeout), 0.0)
            deadline = min(deadline, explicit)
        while time.monotonic() < deadline:
            with self._gripper_lock:
                if self._gripper_goal_angle != owned_goal:
                    return False
                if not self._gripper_active:
                    return self._gripper_position_result == "succeeded"
                if self._gripper_command_cancel.is_set():
                    return False
            time.sleep(0.02)
        self.cancel_gripper_position_command(
            f"gripper target timeout: goal={owned_goal:.6f}rad"
        )
        return False

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
        if initial_sample.is_stale(time.monotonic(), self._feedback_stale_timeout_sec):
            raise RuntimeError("gripper feedback is stale before grasp command")

        close_effort = float(np.clip(close_force, 0.05, _G_GRASP_CLOSE_FORCE_MAX))
        hold_effort = float(np.clip(hold_force, 0.05, _G_TAU_MAX))
        numeric_inputs = (
            close_force,
            hold_force,
            close_timeout_sec,
            min_close_time_sec,
            velocity_threshold,
            min_closure_distance_m,
        )
        if any(not np.isfinite(float(value)) for value in numeric_inputs):
            raise ValueError("gripper grasp parameters must be finite")
        timeout = max(float(close_timeout_sec), 0.1)
        min_time = max(float(min_close_time_sec), 0.0)
        velocity_limit = max(float(velocity_threshold), 0.0)
        min_closure = max(float(min_closure_distance_m), 0.0)
        start_position_m = self.gripper_position_m()
        start = time.monotonic()
        stable_contact_samples = 0
        last_sequence = initial_sample.sequence
        requested_hold = self._grasp_hold_timeout_sec if hold_timeout_sec is None else float(
            hold_timeout_sec
        )
        if not np.isfinite(requested_hold):
            raise ValueError("gripper hold timeout must be finite")
        hold_timeout = float(
            np.clip(requested_hold, 0.1, _G_GRASP_HOLD_TIMEOUT_MAX_SEC)
        )

        with self._gripper_lock:
            self._gripper_command_cancel.clear()
            self._gripper_close_force = close_effort
            self._gripper_hold_force = hold_effort
            self._gripper_hold_deadline = None
            self._gripper_hold_release_reason = None
            self._gripper_command_error = None
            self._gripper_neutral_pending = None
            self._gripper_mode = "grasp_closing"
            self._gripper_active = True
        self._start_gripper_loop()

        while time.monotonic() - start < timeout:
            if self._gripper_command_cancel.is_set():
                self.stop_gripper_motion()
                return (
                    False,
                    False,
                    0.0,
                    self.gripper_position_m(),
                    hold_effort,
                    "grasp canceled",
                )
            elapsed = time.monotonic() - start
            try:
                sample = self._verified_feedback_sample("gripper")
            except Exception as exc:
                self.stop_gripper_motion(str(exc))
                return False, False, 0.0, float("nan"), hold_effort, str(exc)
            if sample.is_stale(time.monotonic(), self._feedback_stale_timeout_sec):
                self.stop_gripper_motion("gripper feedback stale during grasp")
                return (
                    False,
                    False,
                    0.0,
                    self.gripper_position_m(),
                    hold_effort,
                    "gripper feedback stale during grasp",
                )
            if sample.sequence == last_sequence:
                time.sleep(0.005)
                continue
            last_sequence = sample.sequence
            reached_position_m = self.gripper_position_m()
            closure_m = max(start_position_m - reached_position_m, 0.0)
            contact_sample = elapsed >= min_time and is_gripper_contact_sample(
                opening_m=reached_position_m,
                closure_m=closure_m,
                velocity_rad_s=self._gripper_vel,
                torque_nm=self._gripper_torque,
                min_opening_m=_G_GRASP_EMPTY_CLOSE_THRESHOLD_M,
                min_closure_m=min_closure,
                max_velocity_rad_s=velocity_limit,
                min_torque_nm=self._gripper_contact_torque_min_nm,
            )
            stable_contact_samples = stable_contact_samples + 1 if contact_sample else 0
            if stable_contact_samples >= _G_GRASP_CONTACT_STABLE_SAMPLES:
                with self._gripper_lock:
                    self._gripper_hold_angle = float(self._gripper_pos)
                    self._gripper_hold_force = hold_effort
                    self._gripper_hold_deadline = time.monotonic() + hold_timeout
                    self._gripper_mode = "grasp_holding"
                    self._gripper_active = True
                contact_position_m = self.gripper_position_m()
                return (
                    True,
                    True,
                    contact_position_m,
                    contact_position_m,
                    hold_effort,
                    f"contact detected; hold bounded to {hold_timeout:g} s",
                )
            time.sleep(0.01)

        self.stop_gripper_motion("grasp close timeout before contact")
        return (
            False,
            False,
            0.0,
            self.gripper_position_m(),
            hold_effort,
            "grasp close timeout before contact",
        )

    def stop_gripper_motion(self, reason: str = "gripper command canceled") -> None:
        """Cancel the current task and queue a zero-torque command for the bus owner."""
        self._gripper_command_cancel.set()
        with self._gripper_lock:
            self._gripper_command_error = str(reason)
            self._gripper_position_result = "failed"
            self._gripper_hold_deadline = None
            self._gripper_neutral_pending = (float(self._gripper_pos), str(reason), False)
            self._gripper_mode = "neutral_pending"
            self._gripper_active = True

    def cancel_gripper_position_command(self, reason: str = "position command canceled") -> bool:
        with self._gripper_lock:
            if not self._gripper_active or self._gripper_mode != "position":
                return False
        self.stop_gripper_motion(reason)
        return True

    def release_grasp_hold(self, reason: str = "external release") -> bool:
        with self._gripper_lock:
            if not self._gripper_active or self._gripper_mode not in (
                "grasp_closing",
                "grasp_holding",
            ):
                return False
            self._gripper_hold_release_reason = str(reason)
        self.stop_gripper_motion(f"grasp release: {reason}")
        return True

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        if self._gripper_mot is None:
            return self._gripper_pos, self._gripper_vel, self._gripper_torque, 255
        try:
            sample = self._verified_feedback_sample("gripper")
            position, velocity, torque, status = self._validated_gripper_feedback_values(
                sample.state
            )
            if self._gripper_feedback_failure_reason() is not None:
                status = 255
            return position, velocity, torque, status
        except Exception:
            return self._gripper_pos, self._gripper_vel, self._gripper_torque, 255

    def gripper_position_m(self) -> float:
        with self._gripper_lock:
            position = float(self._gripper_pos)
            zero_error = getattr(self, "_gripper_zero_error", None)
        if zero_error is not None:
            return float("nan")
        if not np.isfinite(position) or not (
            _G_ANGLE_OPEN - _G_COORDINATE_TOL_RAD
            <= position
            <= _G_CLOSED_FEEDBACK_TOL_RAD
        ):
            return float("nan")
        distance = (position / _G_ANGLE_OPEN) * _G_MAX_DIST_M
        return float(np.clip(distance, 0.0, _G_MAX_DIST_M))

    def gripper_reached_target(self) -> bool:
        with self._gripper_lock:
            if self._gripper_position_result == "succeeded":
                return True
            if self._gripper_position_result == "failed" or not self._gripper_active:
                return False
            target = self._gripper_goal_angle
            return abs(self._gripper_pos - target) < _G_ARRIVE_TOL

    def send_gripper_motor_cmd(self, cmd) -> None:
        self._require_enabled()
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        gripper_failure = self._gripper_feedback_failure_reason()
        if gripper_failure is not None:
            raise RuntimeError(gripper_failure)
        state = self._verified_feedback_sample("gripper").state
        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(self._gripper_cfg.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(self._gripper_cfg.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(self._gripper_cfg.vlim)

        values = (pos, vel, kp, kd, tau, vlim)
        if not all(np.isfinite(value) for value in values):
            raise ValueError("gripper motor command values must be finite")
        if kp < 0.0 or kd < 0.0 or vlim <= 0.0:
            raise ValueError("gripper kp/kd must be non-negative and vlim must be positive")
        if cmd.use_pos and not _G_OPEN_SOFT_LIMIT <= pos <= 0.0:
            raise ValueError("gripper raw position command outside calibrated range")
        if cmd.use_tau and abs(tau) > _G_TAU_MAX:
            raise ValueError(f"gripper torque command exceeds {_G_TAU_MAX:g} N.m")

        if int(cmd.mode) == 0:
            self._gripper_mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            self._gripper_mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(self._gripper_mot, "send_vel"):
                raise RuntimeError("gripper does not support send_vel")
            self._gripper_mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        with self._gripper_lock:
            self._gripper_active = False
            self._gripper_mode = "idle"

    def _patch_arm_bus_lock(self) -> None:
        for ctrl in self._arm._ctrl_map.values():
            self._patch_controller_bus(ctrl)

        if not hasattr(self._arm, "_bus_lock_patched"):
            for jc in self._arm._joints:
                mot = self._arm._motor_map[jc.name]
                ctrl = self._arm._ctrl_map[jc.vendor]
                self._wrap_motor_bus(mot, ctrl._bus_lock)
            self._arm._bus_lock_patched = True

    @staticmethod
    def _patch_controller_bus(ctrl) -> None:
        if not hasattr(ctrl, "_bus_lock"):
            ctrl._bus_lock = threading.RLock()
        if hasattr(ctrl, "_bus_lock_patched"):
            return
        lock = ctrl._bus_lock

        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in ("poll_feedback_once", "enable_all", "disable_all"):
            if hasattr(ctrl, attr):
                wrapped = _wrap(getattr(ctrl, attr))
                wrapped._rebotarm_locked = True
                setattr(ctrl, attr, wrapped)
        ctrl._bus_lock_patched = True

    @staticmethod
    def _wrap_motor_bus(mot, lock) -> None:
        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in (
            "send_pos_vel",
            "send_mit",
            "send_vel",
            "request_feedback",
            "enable",
            "disable",
            "ensure_mode",
            "write_register_f32",
            "set_zero_position",
        ):
            if hasattr(mot, attr) and not hasattr(getattr(mot, attr), "_rebotarm_locked"):
                wrapped = _wrap(getattr(mot, attr))
                wrapped._rebotarm_locked = True
                setattr(mot, attr, wrapped)

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
        self._gravity_comp_active = False
        with self._gripper_lock:
            self._gripper_active = False
            self._gripper_mode = "idle"
        self._state_machine = "IDLE"
        self._set_lifecycle_state("DISABLING")
        errors: list[str] = []
        controllers: list[object] = []
        for controller in getattr(self._arm, "_ctrl_map", {}).values():
            if all(controller is not existing for existing in controllers):
                controllers.append(controller)
        for controller in controllers:
            try:
                controller.disable_all()
            except Exception as exc:
                errors.append(f"{type(controller).__name__}: {exc}")
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
        pos_cmd = float(np.clip(pos, _G_OPEN_SOFT_LIMIT, 0.0))
        pos_term = kp * (pos_cmd - self._gripper_pos) + kd * (-self._gripper_vel)
        limit = float(np.clip(abs(tau_limit), 0.05, _G_TAU_MAX))
        tau_safe = float(np.clip(pos_term + tau_ff, -limit, limit)) - pos_term
        try:
            self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
        except Exception as exc:
            raise RuntimeError(f"gripper MIT command failed: {exc}") from exc

    def _gripper_tick(self) -> None:
        with self._gripper_lock:
            pending = self._gripper_neutral_pending
            if pending is not None:
                angle, _reason, marks_success = pending
                self._gripper_safe_mit(angle, 0.0, 0.0, 0.0, 0.0, tau_limit=0.05)
                self._gripper_neutral_pending = None
                self._gripper_active = False
                self._gripper_mode = "idle"
                if marks_success:
                    self._gripper_position_result = "succeeded"
                    self._gripper_command_error = None
                return
            target = self._gripper_target_angle
            goal = self._gripper_goal_angle
            effort = self._gripper_target_effort
            close_force = self._gripper_close_force
            hold_force = self._gripper_hold_force
            hold_angle = self._gripper_hold_angle
            mode = self._gripper_mode
            active = self._gripper_active
        if not active:
            return
        sample = self._verified_feedback_sample("gripper")
        gripper_failure = self._gripper_feedback_failure_reason()
        if gripper_failure is not None:
            self.stop_gripper_motion(gripper_failure)
            return
        if mode == "grasp_closing":
            self._gripper_safe_mit(0.0, 0.0, _G_GRASP_CLOSE_KP, _G_GRASP_CLOSE_KD, close_force)
        elif mode == "grasp_holding":
            if self._gripper_hold_deadline is not None and time.monotonic() >= self._gripper_hold_deadline:
                self._gripper_hold_release_reason = "hold timeout"
                self.stop_gripper_motion("grasp release: hold timeout")
                return
            self._gripper_safe_mit(hold_angle, 0.0, _G_GRASP_HOLD_KP, _G_GRASP_HOLD_KD, hold_force)
        elif mode == "position":
            if abs(self._gripper_pos - goal) < _G_ARRIVE_TOL:
                with self._gripper_lock:
                    self._gripper_neutral_pending = (
                        float(self._gripper_pos),
                        "position target reached",
                        True,
                    )
                    self._gripper_mode = "neutral_pending"
                return
            now = time.monotonic()
            elapsed = max(now - (self._gripper_last_tick_monotonic or now), 0.0)
            max_step = self._gripper_position_max_speed_rad_s * elapsed
            remaining = goal - target
            if abs(remaining) <= max_step:
                target = goal
            elif max_step > 0.0:
                target += float(np.copysign(max_step, remaining))
            with self._gripper_lock:
                self._gripper_target_angle = target
                self._gripper_last_tick_monotonic = now
            if (
                self._gripper_target_deadline_monotonic is not None
                and now >= self._gripper_target_deadline_monotonic
            ):
                self.stop_gripper_motion("gripper dynamic position timeout")
                return
            tau_ff = effort if abs(target) < 1e-6 else 0.0
            self._gripper_safe_mit(
                target,
                0.0,
                _G_KP_MOVE,
                _G_KD_MOVE,
                tau_ff,
                tau_limit=effort,
            )

    def _gripper_loop(self) -> None:
        dt = 1.0 / _G_CTRL_RATE
        last = time.perf_counter()
        while not self._gripper_loop_stop.is_set():
            now = time.perf_counter()
            if now - last >= dt:
                last += dt
                self._gripper_tick()
            else:
                time.sleep(1e-4)

    def _start_gripper_loop(self) -> None:
        if not self.control_loop_active:
            raise RuntimeError("gripper command requires the unified hardware control loop")

    def _stop_gripper_loop(self) -> None:
        self._gripper_loop_running = False
