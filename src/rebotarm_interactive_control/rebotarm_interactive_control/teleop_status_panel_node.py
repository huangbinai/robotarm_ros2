from __future__ import annotations

import json
import math
import threading
import time
from contextlib import suppress
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import SetTeachRecordPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_command_api import (
    arm_command_is_replay_locked,
    arm_command_timeout_sec,
    normalize_arm_command,
    should_stop_trajectory_before_arm_command,
    status_state,
)
from .parameter_helpers import build_joint_limits
from .parameter_helpers import sensor_qos_kwargs
from .status_panel_http import create_status_panel_server
from .status_panel_page import HTML_PAGE
from .status_panel_state import TeleopStatusStore
from .teleop_core import validate_web_keyboard_command
from .teach_recording import (
    ReplayStartBand,
    build_replay_start_soft_points,
    compute_auto_align_duration,
    estimate_teach_replay,
    inspect_teach_record,
    list_teach_record_files,
    load_teach_samples,
    normalize_teach_replay_settings,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    retime_teach_samples,
    teach_record_info_to_dict,
    teach_trajectory_preview_to_dict,
    validate_teach_dry_run_request,
    validate_teach_replay_execute_request,
    validate_teach_replay_stop_request,
    write_prepared_teach_record,
)
from .trajectory_safety_monitor import evaluate_replay_tracking
from .moveit_planner import MoveItMotionPlanner
from .web_robot_assets import (
    DEFAULT_GRIPPER_LIMITS_M,
    load_gripper_limits,
    load_moveit_velocity_limits,
    load_urdf_joint_limits,
    merge_velocity_limits,
    merge_joint_limits,
)
from .web_execute import (
    WebExecuteDecision,
    WebGripperDecision,
    interpolate_joint_points,
    validate_web_gripper_request,
    validate_web_execute_request,
)


def _set_duration(duration_msg, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


def _is_number_like(value) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def _number_or_default(value, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not _is_number_like(number) or not math.isfinite(number):
        return float(default)
    return float(number)


def _select_collision_samples(samples, *, max_samples: int) -> list[tuple[int, object]]:
    if not samples:
        return []
    limit = max(int(max_samples), 1)
    if len(samples) <= limit:
        return list(enumerate(samples))
    if limit == 1:
        return [(0, samples[0])]
    indices = sorted(
        {
            round(index * (len(samples) - 1) / (limit - 1))
            for index in range(limit)
        }
    )
    return [(index, samples[index]) for index in indices]


def _decision_response(decision: WebExecuteDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "max_delta": float(decision.max_delta),
        "max_delta_limit": float(decision.max_delta_limit),
        "duration": float(decision.duration),
    }


def _gripper_decision_response(decision: WebGripperDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "position": float(decision.position),
        "max_effort": float(decision.max_effort),
    }


def _keyboard_decision_response(decision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "key": str(decision.key),
        "joint_name": str(decision.joint_name),
        "step_rad": float(decision.step_rad),
        "duration": float(decision.duration),
        "max_joint_speed_rad_s": float(decision.max_joint_speed_rad_s),
    }




class TeleopStatusPanelNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_status_panel_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8088)
        self.declare_parameter("sse_rate_hz", 10.0)
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("direct_threshold", 0.01)
        self.declare_parameter("align_threshold", 0.25)
        self.declare_parameter("align_duration", 3.0)
        self.declare_parameter("align_duration_auto", True)
        self.declare_parameter("align_target_speed_rad_s", 0.15)
        self.declare_parameter("align_min_duration", 3.0)
        self.declare_parameter("align_max_duration", 10.0)
        self.declare_parameter("align_steps", 30)
        self.declare_parameter("replay_speed", 1.0)
        self.declare_parameter("green_jump_rad", 0.03)
        self.declare_parameter("yellow_jump_rad", 0.05)
        self.declare_parameter("yellow_max_speed", 0.6)
        self.declare_parameter("max_replay_velocity_rad_s", 3.0)
        self.declare_parameter(
            "max_replay_velocity_rad_s_by_joint",
            [3.0, 3.0, 3.0, 1.8, 1.8, 1.8],
        )
        self.declare_parameter("max_replay_acceleration_rad_s2", 5.0)
        self.declare_parameter("max_replay_jerk_rad_s3", 20.0)
        self.declare_parameter("large_motion_span_rad", 0.8)
        self.declare_parameter("large_motion_total_rad", 2.5)
        self.declare_parameter("large_motion_max_speed", 1.0)
        self.declare_parameter("start_hold_sec", 0.8)
        self.declare_parameter("soft_start_duration", 1.0)
        self.declare_parameter("soft_start_steps", 30)
        self.declare_parameter("first_hold_sec", 0.3)
        self.declare_parameter("final_hold_sec", 1.0)
        self.declare_parameter("initial_replay_delay_sec", 0.2)
        self.declare_parameter("use_moveit_start_align", True)
        self.declare_parameter("moveit_start_skip_threshold", 0.005)
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        self.declare_parameter("moveit_planner_id", "")
        self.declare_parameter("moveit_planning_time", 3.0)
        self.declare_parameter("moveit_num_planning_attempts", 3)
        self.declare_parameter("moveit_joint_goal_tolerance", 0.005)
        self.declare_parameter("moveit_velocity_scaling", 0.1)
        self.declare_parameter("moveit_acceleration_scaling", 0.1)
        self.declare_parameter("collision_check_enabled", True)
        self.declare_parameter("collision_check_service", "/check_state_validity")
        self.declare_parameter("collision_check_max_samples", 80)
        self.declare_parameter("collision_check_timeout_sec", 2.0)
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("smoothing_window", 7)
        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("filter_cutoff_hz", 5.0)
        self.declare_parameter("filter_sample_rate_hz", 150.0)
        self.declare_parameter("resample_enabled", True)
        self.declare_parameter("resample_rate_hz", 150.0)
        self.declare_parameter("time_parameterization_method", "auto")
        self.declare_parameter("max_prepared_jump_rad", 0.02)
        self.declare_parameter("replay_monitor_enabled", True)
        self.declare_parameter("replay_monitor_period_sec", 0.05)
        self.declare_parameter("replay_monitor_start_grace_sec", 1.0)
        self.declare_parameter("replay_monitor_violation_grace_sec", 0.30)
        self.declare_parameter("max_tracking_error_rad", 0.25)
        self.declare_parameter("max_live_velocity_rad_s", 3.0)
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("panel_mode", "control")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "joint_lower_limits",
            [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        self.declare_parameter("web_execute_enabled", False)
        self.declare_parameter("web_execute_max_delta_rad", 1.5)
        self.declare_parameter("web_execute_min_duration", 1.0)
        self.declare_parameter("web_execute_max_duration", 8.0)
        self.declare_parameter("web_execute_max_joint_speed_rad_s", 1.5)
        self.declare_parameter("web_keyboard_default_step_rad", 0.02)
        self.declare_parameter("web_keyboard_min_step_rad", 0.005)
        self.declare_parameter("web_keyboard_max_step_rad", 0.10)
        self.declare_parameter("web_keyboard_default_duration", 0.2)
        self.declare_parameter("web_keyboard_min_duration", 0.1)
        self.declare_parameter("web_keyboard_max_duration", 2.0)
        self.declare_parameter("web_keyboard_default_speed_rad_s", 0.5)
        self.declare_parameter("gripper_lower_limit_m", DEFAULT_GRIPPER_LIMITS_M[0])
        self.declare_parameter("gripper_upper_limit_m", DEFAULT_GRIPPER_LIMITS_M[1])
        self.declare_parameter("web_gripper_max_effort", 0.3)
        self.declare_parameter("web_gripper_max_effort_limit", 1.5)
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        bringup_share = Path(get_package_share_directory("rebotarm_bringup"))
        self._urdf_path = bringup_share / "description" / "urdf" / "reBot-DevArm_fixend.urdf"
        self._mesh_dir = bringup_share / "description" / "meshes"
        lower = tuple(float(v) for v in self.get_parameter("joint_lower_limits").value)
        upper = tuple(float(v) for v in self.get_parameter("joint_upper_limits").value)
        fallback_limits = build_joint_limits(
            joint_names=self._joint_names,
            lower_limits=lower,
            upper_limits=upper,
        )
        try:
            urdf_limits = load_urdf_joint_limits(self._urdf_path, self._joint_names)
        except Exception as exc:
            self.get_logger().warn(f"failed to load URDF joint limits, using parameter limits: {exc}")
            urdf_limits = {}
        self._joint_limits = merge_joint_limits(
            joint_names=self._joint_names,
            fallback_limits=fallback_limits,
            preferred_limits=urdf_limits,
        )
        moveit_share = Path(get_package_share_directory("rebotarm_moveit_config"))
        moveit_velocity_limits = load_moveit_velocity_limits(
            moveit_share / "config" / "joint_limits.yaml",
            self._joint_names,
        )
        self._joint_velocity_limits = merge_velocity_limits(
            joint_names=self._joint_names,
            default_limit=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            preferred_limits=moveit_velocity_limits,
        )
        gripper_lower = float(self.get_parameter("gripper_lower_limit_m").value)
        gripper_upper = float(self.get_parameter("gripper_upper_limit_m").value)
        if gripper_upper < gripper_lower:
            gripper_lower, gripper_upper = gripper_upper, gripper_lower
        self._gripper_limits = (gripper_lower, gripper_upper)
        self._use_hardware = bool(self.get_parameter("use_hardware").value)
        self._sim_gripper_position = gripper_lower
        self._store = TeleopStatusStore()
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        self._moveit_planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id="end_link",
            frame_id="base_link",
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=str(self.get_parameter("moveit_planning_pipeline").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=0.005,
            goal_orientation_tolerance=0.02,
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
        )
        self._gripper_action_client = ActionClient(
            self,
            GripperCommand,
            f"/{self._arm_namespace}/gripper/command",
        )
        self._gravity_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/start",
        )
        self._gravity_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/stop",
        )
        self._teach_record_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/start",
        )
        self._teach_record_set_path_client = self.create_client(
            SetTeachRecordPath,
            f"/{self._arm_namespace}/teleop/teach_record/set_path",
        )
        self._teach_record_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/stop",
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
        )
        self._arm_enable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/enable",
        )
        self._arm_disable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/disable",
        )
        self._arm_safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
        )
        self._execute_lock = threading.Lock()
        self._execute_goal_handle = None
        self._web_keyboard_lock = threading.Lock()
        self._web_keyboard_enabled = False
        self._web_keyboard_step_rad = float(self.get_parameter("web_keyboard_default_step_rad").value)
        self._web_keyboard_duration = float(self.get_parameter("web_keyboard_default_duration").value)
        self._web_keyboard_speed = float(self.get_parameter("web_keyboard_default_speed_rad_s").value)
        self._teach_replay_lock = threading.Lock()
        self._teach_replay_goal_handle = None
        self._active_teach_replay_trajectory: JointTrajectory | None = None
        self._active_teach_replay_started_at: float | None = None
        self._teach_replay_tracking_violation_since: float | None = None
        self._teach_replay_monitor_stop_requested = False
        self._last_teach_dry_run: dict | None = None
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        arm_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_subscription(ArmStatus, f"/{self._arm_namespace}/arm_status", self._on_arm_status, arm_status_qos)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/status", lambda msg: self._on_status("status", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/recording_status", lambda msg: self._on_status("recording", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/replay_status", lambda msg: self._on_status("replay", msg), 10)
        for joint_name in self._joint_names:
            self.create_subscription(
                JointMotorState,
                f"/{self._arm_namespace}/joints/{joint_name}/state",
                self._on_motor_state,
                sensor_qos,
            )
        self.create_subscription(
            JointMotorState,
            f"/{self._arm_namespace}/gripper/state",
            self._on_gripper_state,
            sensor_qos,
        )
        self._sim_gripper_state_pub = None
        if not self._use_hardware:
            self._sim_gripper_state_pub = self.create_publisher(
                JointMotorState,
                f"/{self._arm_namespace}/gripper/state",
                sensor_qos,
            )
            self.create_timer(0.1, self._publish_simulated_gripper_state)
        self.create_timer(1.0, self._update_gravity_comp_status)
        self.create_timer(
            max(float(self.get_parameter("replay_monitor_period_sec").value), 0.02),
            self._check_active_replay_tracking,
        )
        self._server = self._make_server()
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.get_logger().info(f"teleop status panel available at http://{host}:{port}/")

    def _make_server(self):
        interval = 1.0 / max(float(self.get_parameter("sse_rate_hz").value), 1.0)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        return create_status_panel_server(
            host=host,
            port=port,
            node=self,
            store=self._store,
            html_page=HTML_PAGE,
            urdf_path=self._urdf_path,
            mesh_dir=self._mesh_dir,
            sse_interval_sec=interval,
        )

    def _panel_config(self) -> dict:
        return {
            "joint_names": list(self._joint_names),
            "joint_limits": {
                name: [float(lower), float(upper)]
                for name, (lower, upper) in self._joint_limits.items()
            },
            "joint_velocity_limits": {
                name: float(limit)
                for name, limit in self._joint_velocity_limits.items()
            },
            "gripper_limits": [float(self._gripper_limits[0]), float(self._gripper_limits[1])],
            "web_execute": {
                "enabled": bool(self.get_parameter("web_execute_enabled").value),
                "max_delta_rad": float(self.get_parameter("web_execute_max_delta_rad").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
                "min_duration": float(self.get_parameter("web_execute_min_duration").value),
                "max_duration": float(self.get_parameter("web_execute_max_duration").value),
            },
            "web_keyboard": {
                "step_rad": float(self.get_parameter("web_keyboard_default_step_rad").value),
                "min_step_rad": float(self.get_parameter("web_keyboard_min_step_rad").value),
                "max_step_rad": float(self.get_parameter("web_keyboard_max_step_rad").value),
                "duration": float(self.get_parameter("web_keyboard_default_duration").value),
                "min_duration": float(self.get_parameter("web_keyboard_min_duration").value),
                "max_duration": float(self.get_parameter("web_keyboard_max_duration").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_keyboard_default_speed_rad_s").value),
            },
            "web_gripper": {
                "max_effort": float(self.get_parameter("web_gripper_max_effort").value),
                "max_effort_limit": float(self.get_parameter("web_gripper_max_effort_limit").value),
            },
            "teach": {
                "record_path": str(self.get_parameter("record_path").value),
                "direct_threshold": float(self.get_parameter("direct_threshold").value),
                "align_threshold": float(self.get_parameter("align_threshold").value),
                "align_duration": float(self.get_parameter("align_duration").value),
                "align_duration_auto": bool(self.get_parameter("align_duration_auto").value),
                "align_target_speed_rad_s": float(self.get_parameter("align_target_speed_rad_s").value),
                "align_min_duration": float(self.get_parameter("align_min_duration").value),
                "align_max_duration": float(self.get_parameter("align_max_duration").value),
                "align_steps": int(self.get_parameter("align_steps").value),
                "replay_speed": float(self.get_parameter("replay_speed").value),
                "green_jump_rad": float(self.get_parameter("green_jump_rad").value),
                "yellow_jump_rad": float(self.get_parameter("yellow_jump_rad").value),
                "yellow_max_speed": float(self.get_parameter("yellow_max_speed").value),
                "max_replay_velocity_rad_s": float(self.get_parameter("max_replay_velocity_rad_s").value),
                "max_replay_velocity_rad_s_by_joint": [
                    float(value)
                    for value in self.get_parameter("max_replay_velocity_rad_s_by_joint").value
                ],
                "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
                "large_motion_span_rad": float(self.get_parameter("large_motion_span_rad").value),
                "large_motion_total_rad": float(self.get_parameter("large_motion_total_rad").value),
                "large_motion_max_speed": float(self.get_parameter("large_motion_max_speed").value),
                "start_hold_sec": float(self.get_parameter("start_hold_sec").value),
                "soft_start_duration": float(self.get_parameter("soft_start_duration").value),
                "soft_start_steps": int(self.get_parameter("soft_start_steps").value),
                "first_hold_sec": float(self.get_parameter("first_hold_sec").value),
                "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
                "use_moveit_start_align": bool(self.get_parameter("use_moveit_start_align").value),
                "moveit_start_skip_threshold": float(self.get_parameter("moveit_start_skip_threshold").value),
                "collision_check_enabled": bool(self.get_parameter("collision_check_enabled").value),
                "collision_check_max_samples": int(self.get_parameter("collision_check_max_samples").value),
                "smoothing_enabled": bool(self.get_parameter("smoothing_enabled").value),
                "smoothing_window": int(self.get_parameter("smoothing_window").value),
                "filter_enabled": bool(self.get_parameter("filter_enabled").value),
                "filter_cutoff_hz": float(self.get_parameter("filter_cutoff_hz").value),
                "filter_sample_rate_hz": float(self.get_parameter("filter_sample_rate_hz").value),
                "resample_enabled": bool(self.get_parameter("resample_enabled").value),
                "resample_rate_hz": float(self.get_parameter("resample_rate_hz").value),
                "time_parameterization_method": str(self.get_parameter("time_parameterization_method").value),
                "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
                "use_hardware": bool(self.get_parameter("use_hardware").value) if self.has_parameter("use_hardware") else False,
            },
            "panel_mode": str(self.get_parameter("panel_mode").value),
        }

    def _teach_record_info(self, record_path: str | None = None) -> dict:
        snapshot = self._store.snapshot()
        path = record_path or str(self.get_parameter("record_path").value)
        for key in ("recording", "replay"):
            value = snapshot.teleop.get(key)
            if record_path is None and isinstance(value, dict) and value.get("record_path"):
                path = str(value["record_path"])
                break
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if name in self._joint_names and "position" in data
        }
        info = inspect_teach_record(
            path,
            current_positions=current_positions if current_positions else None,
            direct_threshold=float(self.get_parameter("direct_threshold").value),
            align_threshold=float(self.get_parameter("align_threshold").value),
        )
        payload = teach_record_info_to_dict(info)
        if (
            str(payload.get("start_band", "")).lower() == ReplayStartBand.REJECT.value
            and bool(self.get_parameter("use_moveit_start_align").value)
            and _is_number_like(payload.get("max_error"))
        ):
            payload["start_band"] = ReplayStartBand.MOVEIT_ALIGN.value
            payload["message"] = "start error requires MoveIt start alignment"
        payload["direct_threshold"] = float(self.get_parameter("direct_threshold").value)
        payload["align_threshold"] = float(self.get_parameter("align_threshold").value)
        return self._compact_replay_payload(payload)

    @staticmethod
    def _compact_list(items, *, limit: int = 12) -> list:
        values = list(items) if isinstance(items, (list, tuple)) else []
        return values[: max(int(limit), 0)]

    @classmethod
    def _compact_quality_payload(cls, quality: dict, *, limit: int = 12) -> dict:
        compact = dict(quality)
        if isinstance(compact.get("events"), list):
            compact["events_total"] = len(compact["events"])
            compact["events"] = cls._compact_list(compact["events"], limit=limit)
            compact["events_truncated"] = compact["events_total"] > len(compact["events"])
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        return compact

    @classmethod
    def _compact_replay_payload(cls, payload: dict, *, limit: int = 12) -> dict:
        compact = dict(payload)
        for key in (
            "quality",
            "before_quality",
            "after_quality",
            "raw_quality",
            "filtered_quality",
            "retimed_quality",
        ):
            if isinstance(compact.get(key), dict):
                compact[key] = cls._compact_quality_payload(compact[key], limit=limit)
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        if isinstance(compact.get("prepared_replay"), dict):
            compact["prepared_replay"] = cls._compact_replay_payload(compact["prepared_replay"], limit=limit)
        return compact

    def _max_replay_velocity_limits(self, joint_names: tuple[str, ...]):
        scalar_limit = float(self.get_parameter("max_replay_velocity_rad_s").value)
        values = self.get_parameter("max_replay_velocity_rad_s_by_joint").value
        if isinstance(values, (list, tuple)) and len(values) == len(joint_names):
            return tuple(float(value) for value in values)
        return scalar_limit

    def _prepare_teach_replay_samples(self, samples, settings: dict[str, float | int] | None = None):
        replay_speed = float(settings["replay_speed"]) if settings else float(self.get_parameter("replay_speed").value)
        return prepare_teach_replay_samples(
            samples,
            smoothing_enabled=bool(self.get_parameter("smoothing_enabled").value),
            smoothing_window=int(self.get_parameter("smoothing_window").value),
            filter_enabled=bool(self.get_parameter("filter_enabled").value),
            filter_cutoff_hz=float(self.get_parameter("filter_cutoff_hz").value),
            filter_sample_rate_hz=float(self.get_parameter("filter_sample_rate_hz").value),
            resample_enabled=bool(self.get_parameter("resample_enabled").value),
            resample_rate_hz=float(self.get_parameter("resample_rate_hz").value),
            retime_enabled=True,
            replay_speed=replay_speed,
            max_velocity_rad_s=self._max_replay_velocity_limits(tuple(samples[0].joint_names) if samples else ()),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            time_parameterization_method=str(self.get_parameter("time_parameterization_method").value),
            large_motion_span_rad=float(self.get_parameter("large_motion_span_rad").value),
            large_motion_total_rad=float(self.get_parameter("large_motion_total_rad").value),
            large_motion_max_speed=float(self.get_parameter("large_motion_max_speed").value),
        )

    def _moveit_align_summary(self, info_payload: dict, samples=None, *, plan: bool = False) -> dict:
        enabled = bool(self.get_parameter("use_moveit_start_align").value)
        max_error = info_payload.get("max_error")
        threshold = float(self.get_parameter("moveit_start_skip_threshold").value)
        if not enabled:
            return {"state": "disabled", "message": "MoveIt start alignment disabled"}
        if not _is_number_like(max_error):
            return {"state": "unknown", "message": "current start error unavailable"}
        if float(max_error) < threshold:
            return {
                "state": "skipped",
                "message": "already near teach start; MoveIt alignment not required",
                "max_error": float(max_error),
                "skip_threshold": threshold,
            }
        available = False
        try:
            available = bool(self._moveit_planner._client.service_is_ready())  # noqa: SLF001
            if not available:
                available = bool(self._moveit_planner._client.wait_for_service(timeout_sec=0.0))  # noqa: SLF001
        except Exception:
            available = False
        if not available:
            return {
                "state": "unavailable",
                "message": "MoveIt planning service unavailable",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": str(self.get_parameter("moveit_planning_service").value),
            }
        if not plan:
            return {
                "state": "ready",
                "message": "MoveIt planning service ready",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": str(self.get_parameter("moveit_planning_service").value),
            }
        if not samples:
            return {
                "state": "unknown",
                "message": "no teach samples for MoveIt start alignment precheck",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": str(self.get_parameter("moveit_planning_service").value),
            }
        first = samples[0]
        result = self._moveit_planner.plan_joint_positions(
            joint_names=tuple(first.joint_names),
            target_positions=tuple(first.positions),
            tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
            velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
            acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
        )
        points = len(getattr(result.trajectory, "points", [])) if result.trajectory is not None else 0
        return {
            "state": "planned" if result.success else "failed",
            "message": result.message,
            "max_error": float(max_error),
            "skip_threshold": threshold,
            "service": str(self.get_parameter("moveit_planning_service").value),
            "points": points,
        }

    def _collision_precheck(self, samples) -> dict:
        if not samples:
            return self._collision_precheck_positions((), [])
        first = samples[0]
        positions = [tuple(sample.positions) for sample in samples]
        return self._collision_precheck_positions(tuple(first.joint_names), positions)

    def _collision_precheck_trajectory(self, trajectory: JointTrajectory) -> dict:
        positions = [
            tuple(point.positions)
            for point in getattr(trajectory, "points", [])
            if getattr(point, "positions", None)
        ]
        return self._collision_precheck_positions(tuple(trajectory.joint_names), positions)

    def _collision_precheck_positions(self, joint_names: tuple[str, ...], positions_list: list[tuple[float, ...]]) -> dict:
        if not bool(self.get_parameter("collision_check_enabled").value):
            return {"state": "disabled", "message": "collision precheck disabled"}
        if not joint_names or not positions_list:
            return {"state": "unknown", "message": "no trajectory samples to check"}
        try:
            available = bool(self._state_validity_client.service_is_ready())
            if not available:
                available = bool(self._state_validity_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            available = False
        if not available:
            return {
                "state": "unknown",
                "message": "MoveIt state validity service unavailable",
                "service": str(self.get_parameter("collision_check_service").value),
                "checked_samples": 0,
        }
        max_samples = max(int(self.get_parameter("collision_check_max_samples").value), 1)
        timeout_sec = max(float(self.get_parameter("collision_check_timeout_sec").value), 0.1)
        selected = _select_collision_samples(positions_list, max_samples=max_samples)
        collisions = []
        checked = 0
        deadline = time.monotonic() + timeout_sec
        for sample_index, positions in selected:
            if time.monotonic() >= deadline:
                return {
                    "state": "unknown",
                    "message": "collision precheck timed out",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            request = GetStateValidity.Request()
            request.group_name = str(self.get_parameter("collision_group_name").value)
            request.robot_state.joint_state.name = list(joint_names)
            request.robot_state.joint_state.position = [float(v) for v in positions]
            future = self._state_validity_client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                break
            try:
                response = future.result()
            except Exception as exc:
                return {
                    "state": "unknown",
                    "message": f"collision precheck failed: {exc}",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            checked += 1
            if not bool(getattr(response, "valid", False)):
                contacts = []
                for contact in list(getattr(response, "contacts", []))[:5]:
                    contacts.append(
                        {
                            "body_1": str(getattr(contact, "contact_body_1", "")),
                            "body_2": str(getattr(contact, "contact_body_2", "")),
                        }
                    )
                collisions.append({"sample": sample_index, "contacts": contacts})
                break
        if collisions:
            return {
                "state": "collision",
                "message": "collision detected in teach trajectory",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": collisions,
            }
        if checked < len(selected):
            return {
                "state": "unknown",
                "message": "collision precheck incomplete",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": [],
            }
        return {
            "state": "pass",
            "message": "no collision detected in sampled teach trajectory",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": [],
        }

    def _teach_trajectory(self, record_path: str | None = None, max_points: int = 500) -> dict:
        path = record_path or str(self._teach_record_info(None).get("path", self.get_parameter("record_path").value))
        try:
            samples = load_teach_samples(path)
        except Exception as exc:
            return {
                "accepted": False,
                "message": f"failed to load teach trajectory: {exc}",
                "path": str(path),
                "points": [],
            }
        prepared = self._prepare_teach_replay_samples(samples)
        prepared_path = write_prepared_teach_record(path, prepared)
        preview_samples = load_teach_samples(prepared_path)
        payload = teach_trajectory_preview_to_dict(preview_samples, max_points=max_points)
        payload["prepared_replay"] = prepared_teach_replay_to_dict(prepared)
        payload["collision_precheck"] = self._collision_precheck(preview_samples)
        payload["accepted"] = True
        payload["curve_source"] = "prepared"
        payload["path"] = str(prepared_path)
        payload["raw_record_path"] = str(path)
        payload["prepared_record_path"] = str(prepared_path)
        payload["info"] = self._teach_record_info(str(path))
        return payload

    def _teach_records(self) -> dict:
        record_path = Path(str(self.get_parameter("record_path").value))
        directory = record_path.parent if str(record_path.parent) else Path("teleop_records")
        records = list_teach_record_files(directory)
        return {
            "directory": str(directory),
            "default_record_path": str(record_path),
            "records": records,
        }

    def _handle_teach_dry_run(self, payload: dict) -> dict:
        record_path = payload.get("record_path")
        info_payload = self._teach_record_info(str(record_path) if record_path else None)
        settings = self._teach_replay_settings_from_payload(
            payload,
            max_error=info_payload.get("max_error"),
        )
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        decision = validate_teach_dry_run_request(str(info_payload.get("start_band", "")))
        prepared_payload = {}
        prepared_record_path = ""
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        samples_for_precheck = []
        trajectory_points = 0
        try:
            samples_for_precheck = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(samples_for_precheck, settings)
            prepared_record_path = str(write_prepared_teach_record(str(info_payload.get("path", "")), prepared))
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            moveit_align = self._moveit_align_summary(info_payload, samples_for_precheck, plan=decision.accepted)
            if decision.accepted and str(moveit_align.get("state", "")).lower() not in ("failed", "unavailable", "unknown"):
                trajectory = self._build_teach_replay_trajectory(
                    samples_for_precheck,
                    str(info_payload.get("start_band", "")),
                    settings,
                )
                trajectory_points = len(trajectory.points)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
        except Exception as exc:
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        prepared_quality = prepared_payload.get("after_quality") if isinstance(prepared_payload.get("after_quality"), dict) else {}
        gate_blocked = (
            str(moveit_align.get("state", "")).lower() in ("failed", "unavailable")
            or str(collision_precheck.get("state", "")).lower() in ("collision", "unknown")
        )
        estimate = estimate_teach_replay(
            samples=int(info_payload.get("samples") or 0),
            record_duration_sec=float(info_payload.get("duration_sec") or 0.0),
            start_band=str(info_payload.get("start_band", "")),
            replay_speed=float(settings["replay_speed"]),
            align_duration=float(settings["align_duration"]),
            align_steps=int(settings["align_steps"]),
            final_hold_sec=float(settings["final_hold_sec"]),
        )
        result = {
            "accepted": bool(decision.accepted) and not gate_blocked,
            "state": "blocked" if decision.accepted and gate_blocked else decision.state,
            "message": (
                f"{decision.message}; MoveIt/collision precheck blocked real replay"
                if decision.accepted and gate_blocked
                else decision.message
            ),
            "record_path": str(info_payload.get("path", "")),
            "prepared_record_path": prepared_record_path,
            "start_band": str(info_payload.get("start_band", "")),
            "max_error": info_payload.get("max_error"),
            "worst_joint": str(info_payload.get("worst_joint", "")),
            "samples": int(info_payload.get("samples") or 0),
            "trajectory_points": int(trajectory_points or prepared_payload.get("prepared_samples") or estimate["trajectory_points"]),
            "estimated_duration_sec": float(estimate["estimated_duration_sec"]),
            "settings": settings,
            "quality": quality,
            "risk_level": str(quality.get("risk_level", "unknown")),
            "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
            "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
            "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
            "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
            "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
            "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
            "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
            "prepared_replay": prepared_payload,
            "moveit_start_align": moveit_align,
            "collision_precheck": collision_precheck,
            "target_runtime": "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation",
            "dry_run": True,
        }
        result = self._compact_replay_payload(result)
        self._last_teach_dry_run = result if result["accepted"] else None
        self._store.update_teleop_status("replay", result)
        return result

    def _handle_teach_replay_execute(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web teach replay disabled; launch with web_execute_enabled:=true"
            self._store.update_teleop_status("replay", {"state": "blocked", "message": message})
            return {"accepted": False, "state": "blocked", "message": message}
        record_path = payload.get("record_path")
        info_payload = self._teach_record_info(str(record_path) if record_path else None)
        settings = self._teach_replay_settings_from_payload(
            payload,
            max_error=info_payload.get("max_error"),
        )
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        prepared_payload = {}
        prepared_quality = {}
        prepared_record_path = ""
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        trajectory = None
        try:
            source_samples = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(source_samples, settings)
            prepared_record_path = str(write_prepared_teach_record(str(info_payload.get("path", "")), prepared))
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            prepared_quality = prepared_payload.get("after_quality") if isinstance(prepared_payload.get("after_quality"), dict) else {}
            moveit_align = self._moveit_align_summary(info_payload, source_samples, plan=False)
        except Exception as exc:
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        token = self._last_teach_dry_run or {}
        dry_run_passed = (
            bool(token.get("accepted"))
            and str(token.get("record_path", "")) == str(info_payload.get("path", ""))
            and str(token.get("prepared_risk_level", "")) == str(prepared_quality.get("risk_level", ""))
            and token.get("settings") == settings
        )
        decision = validate_teach_replay_execute_request(
            str(info_payload.get("start_band", "")),
            dry_run_passed=dry_run_passed,
            risk_level=str(quality.get("risk_level", "unknown")),
            prepared_risk_level=str(prepared_quality.get("risk_level", "")) or None,
            prepared_max_jump_rad=prepared_quality.get("max_jump_rad"),
            max_prepared_jump_rad=float(self.get_parameter("max_prepared_jump_rad").value),
            retimed_max_acceleration_rad_s2=prepared_quality.get("max_acceleration_rad_s2"),
            max_replay_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            retimed_max_jerk_rad_s3=prepared_quality.get("max_jerk_rad_s3"),
            max_replay_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            replay_speed=float(settings["replay_speed"]),
            yellow_max_speed=float(self.get_parameter("yellow_max_speed").value),
        )
        moveit_state = str(moveit_align.get("state", "")).lower()
        if decision.accepted and moveit_state in ("failed", "unavailable", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"MoveIt start alignment not ready: {moveit_align.get('message', moveit_state)}",
            )
        if decision.accepted:
            try:
                samples = load_teach_samples(str(info_payload["path"]))
                if not samples:
                    raise ValueError("record contains no samples")
                trajectory = self._build_teach_replay_trajectory(samples, str(info_payload.get("start_band", "")), settings)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
            except Exception as exc:
                collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        precheck_state = str(collision_precheck.get("state", "")).lower()
        if decision.accepted and precheck_state in ("collision", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"collision precheck blocked replay: {collision_precheck.get('message', precheck_state)}",
            )
        if not decision.accepted:
            result = {
                "accepted": False,
                "state": decision.state,
                "message": decision.message,
                "record_path": str(info_payload.get("path", "")),
                "prepared_record_path": prepared_record_path,
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "quality": quality,
                "risk_level": str(quality.get("risk_level", "unknown")),
                "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
                "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
                "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
                "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
                "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
                "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
                "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
                "prepared_replay": prepared_payload,
                "moveit_start_align": moveit_align,
                "collision_precheck": collision_precheck,
                "target_runtime": "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation",
                "dry_run": False,
            }
            result = self._compact_replay_payload(result)
            self._store.update_teleop_status("replay", result)
            return result
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("replay", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}
        if trajectory is None:
            result = {
                "accepted": False,
                "state": "blocked",
                "message": "failed to build replay trajectory",
                "record_path": str(info_payload.get("path", "")),
                "prepared_record_path": prepared_record_path,
                "start_band": str(info_payload.get("start_band", "")),
                "moveit_start_align": moveit_align,
                "collision_precheck": collision_precheck,
                "prepared_replay": prepared_payload,
                "dry_run": False,
            }
            result = self._compact_replay_payload(result)
            self._store.update_teleop_status("replay", result)
            return result
        prepared_payload = getattr(self, "_last_teach_prepared_payload", prepared_payload)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_teach_replay_goal_response(fut, info_payload, len(trajectory.points), trajectory))
        result = {
            "accepted": True,
            "state": "replaying",
            "message": decision.message,
            "record_path": str(info_payload.get("path", "")),
            "prepared_record_path": prepared_record_path,
            "start_band": str(info_payload.get("start_band", "")),
            "max_error": info_payload.get("max_error"),
            "worst_joint": str(info_payload.get("worst_joint", "")),
            "samples": int(info_payload.get("samples") or 0),
            "trajectory_points": len(trajectory.points),
            "estimated_duration_sec": float(estimate_teach_replay(
                samples=int(info_payload.get("samples") or 0),
                record_duration_sec=float(info_payload.get("duration_sec") or 0.0),
                start_band=str(info_payload.get("start_band", "")),
                replay_speed=float(settings["replay_speed"]),
                align_duration=float(settings["align_duration"]),
                align_steps=int(settings["align_steps"]),
                final_hold_sec=float(settings["final_hold_sec"]),
            )["estimated_duration_sec"]),
            "settings": settings,
            "quality": quality,
            "risk_level": str(quality.get("risk_level", "unknown")),
            "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
            "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
            "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
            "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
            "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
            "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
            "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
            "prepared_replay": prepared_payload,
            "moveit_start_align": moveit_align,
            "collision_precheck": collision_precheck,
            "target_runtime": "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation",
            "dry_run": False,
        }
        result = self._compact_replay_payload(result)
        self._store.update_teleop_status("replay", result)
        return result

    def _handle_teach_replay_stop(self) -> dict:
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
        decision = validate_teach_replay_stop_request(goal_handle is not None)
        if not decision.accepted:
            stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.8)
            if stop_requested:
                result = {
                    "accepted": True,
                    "state": "stop_requested",
                    "message": "controller trajectory_stop requested",
                }
            else:
                result = {"accepted": False, "state": decision.state, "message": decision.message}
            self._store.update_teleop_status("replay", result)
            return result
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_teach_replay_cancel_response)
            self._request_controller_trajectory_stop(timeout_sec=0.2)
        except Exception as exc:
            result = {"accepted": False, "state": "failed", "message": f"failed to request teach replay cancel: {exc}"}
            self._store.update_teleop_status("replay", result)
            return result
        result = {"accepted": True, "state": decision.state, "message": decision.message}
        self._store.update_teleop_status("replay", result)
        return result

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        try:
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
                return False
            self._trajectory_stop_client.call_async(Trigger.Request())
            return True
        except Exception:
            return False

    def _call_trigger_service(self, client, *, timeout_sec: float) -> tuple[bool, str]:
        try:
            if not client.wait_for_service(timeout_sec=min(timeout_sec, 0.5)):
                return False, "service unavailable"
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + max(float(timeout_sec), 0.1)
            while time.monotonic() < deadline:
                if future.done():
                    response = future.result()
                    return bool(getattr(response, "success", False)), str(getattr(response, "message", ""))
                time.sleep(0.02)
            return False, "service timeout"
        except Exception as exc:
            return False, str(exc)

    def _handle_arm_service_command(self, command: str) -> dict:
        command = normalize_arm_command(command) or ""
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web arm command disabled; launch with web_execute_enabled:=true"
            result = {"accepted": False, "state": "blocked", "command": command, "message": message}
            self._store.update_teleop_status("arm_command", result)
            return result
        replay_state = status_state(self._store.snapshot().teleop.get("replay", {}))
        if arm_command_is_replay_locked(replay_state):
            message = "arm command blocked during teach replay"
            result = {"accepted": False, "state": "blocked", "command": command, "message": message}
            self._store.update_teleop_status("arm_command", result)
            return result
        clients = {
            "safe_home": self._arm_safe_home_client,
            "enable": self._arm_enable_client,
            "disable": self._arm_disable_client,
        }
        if command not in clients:
            result = {"accepted": False, "state": "rejected", "command": command, "message": "unknown arm command"}
            self._store.update_teleop_status("arm_command", result)
            return result
        stop_requested = False
        stop_message = ""
        if should_stop_trajectory_before_arm_command(command):
            stop_requested, stop_message = self._call_trigger_service(
                self._trajectory_stop_client,
                timeout_sec=2.0,
            )
            with self._execute_lock:
                self._execute_goal_handle = None
        timeout_sec = arm_command_timeout_sec(command)
        ok, message = self._call_trigger_service(clients[command], timeout_sec=timeout_sec)
        if stop_message:
            message = f"{message}; trajectory_stop: {stop_message}" if message else f"trajectory_stop: {stop_message}"
        result = {
            "accepted": ok,
            "state": "done" if ok else "failed",
            "command": command,
            "message": message or ("done" if ok else "failed"),
            "trajectory_stop_requested": stop_requested,
        }
        if ok:
            arm = self._store.snapshot().arm
            enabled = bool(arm.get("enabled", False))
            mode = str(arm.get("mode", ""))
            if command == "disable":
                enabled = False
            elif command in ("enable", "safe_home"):
                enabled = True
            if command == "safe_home":
                mode = mode or "pos_vel"
            self._store.update_arm_status(
                mode=mode,
                enabled=enabled,
                state_machine="IDLE",
                error_codes=tuple(str(v) for v in arm.get("error_codes", [])),
            )
        self._store.update_teleop_status("arm_command", result)
        return result

    def _handle_teach_record_start(self, payload: dict | None = None) -> dict:
        payload = payload or {}
        requested_path = str(payload.get("record_path", "")).strip()
        set_path_ok = True
        set_path_message = ""
        normalized_path = ""
        if requested_path:
            try:
                if not self._teach_record_set_path_client.wait_for_service(timeout_sec=0.5):
                    set_path_ok = False
                    set_path_message = "record path service unavailable"
                else:
                    request = SetTeachRecordPath.Request()
                    request.record_path = requested_path
                    future = self._teach_record_set_path_client.call_async(request)
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline and not future.done():
                        time.sleep(0.02)
                    if future.done():
                        response = future.result()
                        set_path_ok = bool(getattr(response, "success", False))
                        set_path_message = str(getattr(response, "message", ""))
                        normalized_path = str(getattr(response, "normalized_path", ""))
                    else:
                        set_path_ok = False
                        set_path_message = "record path service timeout"
            except Exception as exc:
                set_path_ok = False
                set_path_message = str(exc)
        if not set_path_ok:
            result = {
                "accepted": False,
                "state": "blocked",
                "message": f"record path: {set_path_message}",
                "record_path": normalized_path or requested_path,
            }
            self._store.update_teleop_status("recording", result)
            return result
        gravity_ok, gravity_message = self._call_trigger_service(
            self._gravity_start_client,
            timeout_sec=2.0,
        )
        record_ok, record_message = self._call_trigger_service(
            self._teach_record_start_client,
            timeout_sec=2.0,
        )
        accepted = record_ok and (gravity_ok or "already" in gravity_message.lower())
        result = {
            "accepted": accepted,
            "state": "starting" if accepted else "blocked",
            "message": f"gravity: {gravity_message or gravity_ok}; record: {record_message or record_ok}",
            "gravity_started": gravity_ok,
            "record_started": record_ok,
            "record_path": normalized_path or requested_path,
        }
        self._store.update_teleop_status("recording", result)
        return result

    def _handle_teach_record_stop(self) -> dict:
        record_ok, record_message = self._call_trigger_service(
            self._teach_record_stop_client,
            timeout_sec=2.0,
        )
        gravity_ok, gravity_message = self._call_trigger_service(
            self._gravity_stop_client,
            timeout_sec=2.0,
        )
        accepted = record_ok
        result = {
            "accepted": accepted,
            "state": "stopped" if accepted else "failed",
            "message": f"record: {record_message or record_ok}; gravity: {gravity_message or gravity_ok}",
            "record_stopped": record_ok,
            "gravity_stopped": gravity_ok,
        }
        self._store.update_teleop_status("recording", result)
        return result

    def _auto_align_duration_from_error(self, max_error: float | None) -> float:
        if bool(self.get_parameter("align_duration_auto").value):
            return compute_auto_align_duration(
                max_error,
                target_speed_rad_s=float(self.get_parameter("align_target_speed_rad_s").value),
                min_duration_sec=float(self.get_parameter("align_min_duration").value),
                max_duration_sec=float(self.get_parameter("align_max_duration").value),
            )
        return float(self.get_parameter("align_duration").value)

    def _teach_replay_settings_from_payload(
        self,
        payload: dict,
        *,
        max_error: float | None = None,
    ) -> dict[str, float | int]:
        values = payload.get("settings") if isinstance(payload.get("settings"), dict) else {}
        align_duration = self._auto_align_duration_from_error(max_error)
        if not bool(self.get_parameter("align_duration_auto").value):
            align_duration = float(values.get("align_duration", align_duration))
        return normalize_teach_replay_settings(
            replay_speed=float(values.get("replay_speed", self.get_parameter("replay_speed").value)),
            align_duration=align_duration,
            align_steps=int(values.get("align_steps", self.get_parameter("align_steps").value)),
            final_hold_sec=1.0,
        )

    def _build_teach_replay_trajectory(self, samples, start_band: str, settings: dict[str, float | int]) -> JointTrajectory:
        prepared = self._prepare_teach_replay_samples(samples, settings)
        self._last_teach_prepared_payload = prepared_teach_replay_to_dict(prepared)
        replay_samples = prepared.samples
        first = replay_samples[0]
        trajectory = JointTrajectory()
        trajectory.joint_names = list(first.joint_names)
        snapshot = self._store.snapshot()
        current_map = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        current_positions = tuple(current_map.get(name, start) for name, start in zip(first.joint_names, first.positions))
        if bool(self.get_parameter("use_moveit_start_align").value):
            elapsed = self._append_moveit_start_alignment(
                trajectory,
                current_positions=current_positions,
                first_positions=first.positions,
            )
        else:
            start_points = build_replay_start_soft_points(
                current_positions=current_positions,
                first_positions=first.positions,
                start_band=start_band,
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                soft_start_duration=float(self.get_parameter("soft_start_duration").value),
                soft_start_steps=int(self.get_parameter("soft_start_steps").value),
                align_duration=float(settings["align_duration"]),
                align_steps=int(settings["align_steps"]),
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
            )
            for start_point in start_points:
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in start_point.positions]
                point.velocities = [0.0 for _ in start_point.positions]
                _set_duration(point.time_from_start, start_point.time_from_start)
                trajectory.points.append(point)
            elapsed = start_points[-1].time_from_start if start_points else 0.0
        speed = max(float(prepared.effective_replay_speed), 0.01)
        prepared_quality = prepared.after_quality
        if str(prepared_quality.risk_level) == "yellow":
            speed = min(speed, float(self.get_parameter("yellow_max_speed").value))
        if prepared.retimed_points:
            initial_delay = max(float(self.get_parameter("initial_replay_delay_sec").value), 0.0)
            for retimed in prepared.retimed_points:
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in retimed.positions]
                point.velocities = [float(v) for v in retimed.velocities] if retimed.velocities else [0.0 for _ in point.positions]
                _set_duration(point.time_from_start, elapsed + initial_delay + float(retimed.time_from_start))
                trajectory.points.append(point)
        else:
            for retimed in retime_teach_samples(
                replay_samples,
                replay_speed=speed,
                max_velocity_rad_s=self._max_replay_velocity_limits(tuple(replay_samples[0].joint_names)),
                max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
                initial_delay_sec=float(self.get_parameter("initial_replay_delay_sec").value),
                boundary_zero_velocity=True,
            ):
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in retimed.positions]
                if retimed.velocities:
                    point.velocities = [float(v) for v in retimed.velocities]
                _set_duration(point.time_from_start, elapsed + retimed.time_from_start)
                trajectory.points.append(point)
        self._append_final_hold(trajectory, final_hold_sec=float(settings["final_hold_sec"]))
        return trajectory

    def _append_final_hold(self, trajectory: JointTrajectory, *, final_hold_sec: float) -> None:
        final_hold = max(float(final_hold_sec), 0.0)
        if final_hold <= 0.0 or not trajectory.points:
            return
        last_point = trajectory.points[-1]
        last_time = float(last_point.time_from_start.sec) + float(last_point.time_from_start.nanosec) * 1e-9
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in last_point.positions]
        hold_point.velocities = [0.0 for _ in hold_point.positions]
        _set_duration(hold_point.time_from_start, last_time + final_hold)
        trajectory.points.append(hold_point)

    def _append_moveit_start_alignment(
        self,
        trajectory: JointTrajectory,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        elapsed = max(float(self.get_parameter("start_hold_sec").value), 0.0)
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in current_positions]
        hold_point.velocities = [0.0 for _ in current_positions]
        _set_duration(hold_point.time_from_start, elapsed)
        trajectory.points.append(hold_point)
        max_error = max(
            (abs(float(a) - float(b)) for a, b in zip(current_positions, first_positions)),
            default=0.0,
        )
        if max_error >= float(self.get_parameter("moveit_start_skip_threshold").value):
            plan = self._moveit_planner.plan_joint_positions(
                joint_names=tuple(trajectory.joint_names),
                target_positions=first_positions,
                tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            )
            if not plan.success or plan.trajectory is None:
                raise ValueError(f"moveit start alignment failed: {plan.message}")
            source_names = list(getattr(plan.trajectory, "joint_names", []))
            index_by_name = {name: index for index, name in enumerate(source_names)}
            missing = [name for name in trajectory.joint_names if name not in index_by_name]
            if missing:
                raise ValueError(f"moveit start alignment missing joints: {', '.join(missing)}")
            for source_point in getattr(plan.trajectory, "points", []):
                source_time = float(source_point.time_from_start.sec) + float(source_point.time_from_start.nanosec) * 1e-9
                point = JointTrajectoryPoint()
                point.positions = [
                    float(source_point.positions[index_by_name[name]])
                    for name in trajectory.joint_names
                ]
                if getattr(source_point, "velocities", None):
                    point.velocities = [
                        float(source_point.velocities[index_by_name[name]])
                        for name in trajectory.joint_names
                    ]
                _set_duration(point.time_from_start, elapsed + source_time)
                trajectory.points.append(point)
            if trajectory.points:
                last = trajectory.points[-1].time_from_start
                elapsed = float(last.sec) + float(last.nanosec) * 1e-9
        first_hold = max(float(self.get_parameter("first_hold_sec").value), 0.0)
        if first_hold > 0.0:
            elapsed += first_hold
            first_point = JointTrajectoryPoint()
            first_point.positions = [float(v) for v in first_positions]
            first_point.velocities = [0.0 for _ in first_positions]
            _set_duration(first_point.time_from_start, elapsed)
            trajectory.points.append(first_point)
        return elapsed

    def _on_teach_replay_goal_response(self, future, info_payload: dict, points: int, trajectory: JointTrajectory) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status("replay", {"state": "rejected", "message": "teach replay goal rejected"})
            return
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = goal_handle
            self._active_teach_replay_trajectory = trajectory
            self._active_teach_replay_started_at = time.monotonic()
            self._teach_replay_tracking_violation_since = None
            self._teach_replay_monitor_stop_requested = False
        self._store.update_teleop_status(
            "replay",
            {
                "state": "replaying",
                "message": "teach replay goal accepted",
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "runtime_monitor": {
                    "enabled": bool(self.get_parameter("replay_monitor_enabled").value),
                    "max_tracking_error_rad": float(self.get_parameter("max_tracking_error_rad").value),
                    "max_live_velocity_rad_s": float(self.get_parameter("max_live_velocity_rad_s").value),
                },
                "dry_run": False,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_teach_replay_result(fut, info_payload, points))

    def _on_teach_replay_cancel_response(self, future) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "teach replay cancel accepted"
            if goals_canceling
            else "teach replay already finished before cancel"
        )
        self._store.update_teleop_status("replay", {"state": state, "message": message})
        if not goals_canceling:
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
                self._active_teach_replay_trajectory = None
                self._active_teach_replay_started_at = None
                self._teach_replay_tracking_violation_since = None
                self._teach_replay_monitor_stop_requested = False

    def _on_teach_replay_result(self, future, info_payload: dict, points: int) -> None:
        previous_replay = self._store.snapshot().teleop.get("replay", {})
        with self._teach_replay_lock:
            monitor_stop_requested = self._teach_replay_monitor_stop_requested
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
                self._active_teach_replay_trajectory = None
                self._active_teach_replay_started_at = None
                self._teach_replay_tracking_violation_since = None
                self._teach_replay_monitor_stop_requested = False
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "safety_stop" if monitor_stop_requested else "canceled"
        else:
            state = "failed"
        message = f"teach replay result status={status}, error_code={error_code}: {error_string}"
        runtime_monitor = previous_replay.get("runtime_monitor") if isinstance(previous_replay, dict) else None
        if status == 5 and monitor_stop_requested:
            previous_message = str(previous_replay.get("message", "")) if isinstance(previous_replay, dict) else ""
            message = (
                f"action canceled after runtime monitor stop: {previous_message}"
                if previous_message
                else "action canceled after runtime monitor stop"
            )
        self._store.update_teleop_status(
            "replay",
            {
                "state": state,
                "message": message,
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "runtime_monitor": runtime_monitor,
                "dry_run": False,
            },
        )
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = None
            self._active_teach_replay_trajectory = None
            self._active_teach_replay_started_at = None
            self._teach_replay_tracking_violation_since = None
            self._teach_replay_monitor_stop_requested = False

    def _check_active_replay_tracking(self) -> None:
        if not bool(self.get_parameter("replay_monitor_enabled").value):
            return
        snapshot = self._store.snapshot()
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
            trajectory = self._active_teach_replay_trajectory
            started_at = self._active_teach_replay_started_at
            stop_requested = self._teach_replay_monitor_stop_requested
        if goal_handle is None or trajectory is None or started_at is None or stop_requested:
            return
        elapsed = time.monotonic() - started_at
        if elapsed < float(self.get_parameter("replay_monitor_start_grace_sec").value):
            return
        joint_names = tuple(snapshot.joints.keys())
        positions = tuple(float(item.get("position", 0.0)) for item in snapshot.joints.values())
        velocities = tuple(float(item.get("velocity", 0.0)) for item in snapshot.joints.values())
        result = evaluate_replay_tracking(
            trajectory,
            joint_names=joint_names,
            positions=positions,
            velocities=velocities,
            elapsed_sec=elapsed,
            max_tracking_error_rad=float(self.get_parameter("max_tracking_error_rad").value),
            max_live_velocity_rad_s=float(self.get_parameter("max_live_velocity_rad_s").value),
        )
        if result.ok:
            with self._teach_replay_lock:
                self._teach_replay_tracking_violation_since = None
            return
        now = time.monotonic()
        with self._teach_replay_lock:
            if self._teach_replay_tracking_violation_since is None:
                self._teach_replay_tracking_violation_since = now
                return
            if now - self._teach_replay_tracking_violation_since < float(self.get_parameter("replay_monitor_violation_grace_sec").value):
                return
            self._teach_replay_monitor_stop_requested = True
        self._request_controller_trajectory_stop(timeout_sec=0.2)
        with suppress(Exception):
            goal_handle.cancel_goal_async()
        self._store.update_teleop_status(
            "replay",
            {
                "state": "safety_stop",
                "message": f"runtime replay monitor stopped trajectory: {result.message}",
                "runtime_monitor": {
                    "reason": result.reason,
                    "worst_joint": result.worst_joint,
                    "max_tracking_error_rad": result.max_tracking_error_rad,
                    "max_live_velocity_rad_s": result.max_live_velocity_rad_s,
                    "tracking_error": result.reason == "tracking_error",
                    "live_velocity": result.reason == "live_velocity",
                },
                "dry_run": False,
            },
        )

    def _handle_keyboard_enable(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web keyboard disabled; launch with web_execute_enabled:=true"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "blocked", "message": message})
            return {"accepted": False, "message": message}
        if str(self.get_parameter("panel_mode").value).lower() == "check":
            message = "web keyboard blocked: check mode is read-only"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "blocked", "message": message})
            return {"accepted": False, "message": message}
        step = _number_or_default(payload.get("step_rad"), self._web_keyboard_step_rad)
        duration = _number_or_default(payload.get("duration"), self._web_keyboard_duration)
        speed = _number_or_default(payload.get("max_joint_speed_rad_s"), self._web_keyboard_speed)
        with self._web_keyboard_lock:
            self._web_keyboard_step_rad = min(
                max(step, float(self.get_parameter("web_keyboard_min_step_rad").value)),
                float(self.get_parameter("web_keyboard_max_step_rad").value),
            )
            self._web_keyboard_duration = min(
                max(duration, float(self.get_parameter("web_keyboard_min_duration").value)),
                float(self.get_parameter("web_keyboard_max_duration").value),
            )
            self._web_keyboard_speed = min(
                max(speed, 0.05),
                float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            )
            self._web_keyboard_enabled = True
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "ready",
            "message": "web keyboard teleop enabled",
            "step_rad": self._web_keyboard_step_rad,
            "duration": self._web_keyboard_duration,
            "max_joint_speed_rad_s": self._web_keyboard_speed,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_disable(self) -> dict:
        with self._web_keyboard_lock:
            self._web_keyboard_enabled = False
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.2)
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "disabled",
            "message": "web keyboard teleop disabled; trajectory_stop requested" if stop_requested else "web keyboard teleop disabled",
            "trajectory_stop_requested": stop_requested,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_key(self, payload: dict) -> dict:
        with self._web_keyboard_lock:
            enabled = self._web_keyboard_enabled
            step_rad = self._web_keyboard_step_rad
            duration = self._web_keyboard_duration
            speed = self._web_keyboard_speed
        request_payload = dict(payload)
        request_payload.setdefault("step_rad", step_rad)
        request_payload.setdefault("duration", duration)
        request_payload.setdefault("max_joint_speed_rad_s", speed)
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        decision = validate_web_keyboard_command(
            request_payload,
            enabled=enabled,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            default_step_rad=float(self.get_parameter("web_keyboard_default_step_rad").value),
            min_step_rad=float(self.get_parameter("web_keyboard_min_step_rad").value),
            max_step_rad=float(self.get_parameter("web_keyboard_max_step_rad").value),
            default_duration=float(self.get_parameter("web_keyboard_default_duration").value),
            min_duration=float(self.get_parameter("web_keyboard_min_duration").value),
            max_duration=float(self.get_parameter("web_keyboard_max_duration").value),
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status(
                "status",
                {"source": "web_keyboard", "state": "rejected", "message": decision.message, "last_key": payload.get("key")},
            )
            return _keyboard_decision_response(decision)
        if not self._action_client.wait_for_server(timeout_sec=0.05):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "unavailable", "message": message, "last_key": decision.key})
            return {"accepted": False, "message": message}
        trajectory = JointTrajectory()
        trajectory.joint_names = list(decision.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in decision.positions]
        _set_duration(point.time_from_start, decision.duration)
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_keyboard_goal_response(fut, decision))
        result = _keyboard_decision_response(decision)
        self._store.update_teleop_status(
            "status",
            {
                **result,
                "source": "web_keyboard",
                "state": "active",
                "last_key": decision.key,
            },
        )
        return result

    def _on_keyboard_goal_response(self, future, decision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "failed", "message": str(exc), "last_key": decision.key})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "rejected", "message": "keyboard trajectory goal rejected", "last_key": decision.key})
            return
        self._store.update_teleop_status(
            "status",
            {
                "source": "web_keyboard",
                "state": "accepted",
                "message": decision.message,
                "last_key": decision.key,
                "joint_name": decision.joint_name,
                "step_rad": decision.step_rad,
                "duration": decision.duration,
                "max_joint_speed_rad_s": decision.max_joint_speed_rad_s,
            },
        )

    def _handle_execute_preview(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            return {
                "accepted": False,
                "message": "web execute disabled; launch with web_execute_enabled:=true",
            }
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        decision = validate_web_execute_request(
            payload,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            max_delta_rad=float(self.get_parameter("web_execute_max_delta_rad").value),
            min_duration=float(self.get_parameter("web_execute_min_duration").value),
            max_duration=float(self.get_parameter("web_execute_max_duration").value),
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status("web_execute", {"state": "rejected", "message": decision.message})
            return _decision_response(decision)
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("web_execute", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}

        trajectory = JointTrajectory()
        trajectory.joint_names = list(decision.joint_names)
        current = tuple(current_positions[name] for name in decision.joint_names)
        for elapsed, positions in interpolate_joint_points(
            current=current,
            target=decision.positions,
            duration=decision.duration,
        ):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            _set_duration(point.time_from_start, elapsed)
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_execute_goal_response(fut, decision))
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "active",
                "message": decision.message,
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
                "points": len(trajectory.points),
            },
        )
        return _decision_response(decision)

    def _handle_stop_execute(self) -> dict:
        with self._execute_lock:
            goal_handle = self._execute_goal_handle
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.2)
        if goal_handle is None:
            message = (
                "no active web execute goal; controller trajectory_stop requested"
                if stop_requested
                else "no active web execute goal; controller trajectory_stop unavailable"
            )
            state = "cancel_requested" if stop_requested else "idle"
            self._store.update_teleop_status(
                "web_execute",
                {
                    "state": state,
                    "message": message,
                    "trajectory_stop_requested": stop_requested,
                },
            )
            return {
                "accepted": bool(stop_requested),
                "state": state,
                "message": message,
                "trajectory_stop_requested": stop_requested,
            }
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_execute_cancel_response)
        except Exception as exc:
            message = f"failed to request trajectory cancel: {exc}"
            if stop_requested:
                message = f"{message}; controller trajectory_stop requested"
                self._store.update_teleop_status(
                    "web_execute",
                    {
                        "state": "cancel_requested",
                        "message": message,
                        "trajectory_stop_requested": True,
                    },
                )
                return {"accepted": True, "message": message, "trajectory_stop_requested": True}
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": message})
            return {"accepted": False, "message": message, "trajectory_stop_requested": False}
        message = (
            "trajectory cancel requested; controller trajectory_stop requested"
            if stop_requested
            else "trajectory cancel requested; controller trajectory_stop unavailable"
        )
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "cancel_requested",
                "message": message,
                "trajectory_stop_requested": stop_requested,
            },
        )
        with self._execute_lock:
            self._execute_goal_handle = None
        return {"accepted": True, "message": message, "trajectory_stop_requested": stop_requested}

    def _handle_set_gripper(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            return {
                "accepted": False,
                "message": "web gripper disabled; launch with web_execute_enabled:=true",
            }
        decision = validate_web_gripper_request(
            payload,
            gripper_limits=self._gripper_limits,
            default_max_effort=float(self.get_parameter("web_gripper_max_effort").value),
            max_effort_limit=float(self.get_parameter("web_gripper_max_effort_limit").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status("web_gripper", {"state": "rejected", "message": decision.message})
            return _gripper_decision_response(decision)
        if not self._use_hardware:
            self._sim_gripper_position = float(decision.position)
            self._publish_simulated_gripper_state()
            self._store.update_teleop_status(
                "web_gripper",
                {
                    "state": "done",
                    "message": f"simulated gripper position={decision.position:.4f} m",
                    "position": decision.position,
                    "max_effort": decision.max_effort,
                    "simulated": True,
                },
            )
            return _gripper_decision_response(decision)
        if not self._gripper_action_client.wait_for_server(timeout_sec=0.1):
            message = "gripper command action unavailable"
            self._store.update_teleop_status("web_gripper", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}

        goal = GripperCommand.Goal()
        goal.command.position = float(decision.position)
        goal.command.max_effort = float(decision.max_effort)
        future = self._gripper_action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_gripper_goal_response(fut, decision))
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "active",
                "message": decision.message,
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
        )
        return _gripper_decision_response(decision)

    def _on_gripper_goal_response(self, future, decision: WebGripperDecision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_gripper",
                {"state": "rejected", "message": "gripper goal rejected"},
            )
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "accepted",
                "message": "gripper goal accepted by controller",
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_gripper_result(fut, decision))

    def _on_gripper_result(self, future, decision: WebGripperDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            reached = bool(getattr(result, "reached_goal", False))
            position = float(getattr(result, "position", decision.position))
            effort = float(getattr(result, "effort", 0.0))
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "done" if reached else "failed",
                "message": f"gripper result reached={reached}",
                "position": position,
                "max_effort": decision.max_effort,
                "effort": effort,
            },
        )

    def _publish_simulated_gripper_state(self) -> None:
        if self._sim_gripper_state_pub is None:
            return
        msg = JointMotorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = "gripper"
        msg.position = float(self._sim_gripper_position)
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.status_code = 1
        self._sim_gripper_state_pub.publish(msg)

    def _on_execute_cancel_response(self, future) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "trajectory cancel accepted"
            if goals_canceling
            else "trajectory already finished before cancel"
        )
        self._store.update_teleop_status("web_execute", {"state": state, "message": message})

    def _on_execute_goal_response(self, future, decision: WebExecuteDecision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_execute",
                {"state": "rejected", "message": "trajectory goal rejected"},
            )
            return
        with self._execute_lock:
            self._execute_goal_handle = goal_handle
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "accepted",
                "message": "trajectory goal accepted by controller",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_execute_result(fut, decision))

    def _on_execute_result(self, future, decision: WebExecuteDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            status = int(result_response.status)
            error_code = int(getattr(result, "error_code", 0))
            error_string = str(getattr(result, "error_string", ""))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            with self._execute_lock:
                self._execute_goal_handle = None
            return
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        with self._execute_lock:
            self._execute_goal_handle = None
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": state,
                "message": f"trajectory result status={status}, error_code={error_code}: {error_string}",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._store.update_joint_state(
            names=tuple(str(v) for v in msg.name),
            positions=tuple(float(v) for v in msg.position),
            velocities=tuple(float(v) for v in msg.velocity),
            efforts=tuple(float(v) for v in msg.effort),
        )

    def _on_motor_state(self, msg: JointMotorState) -> None:
        self._store.update_motor_state(
            joint_name=str(msg.joint_name),
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )

    def _on_gripper_state(self, msg: JointMotorState) -> None:
        joint_name = str(msg.joint_name).strip() or "gripper"
        self._store.update_motor_state(
            joint_name=joint_name,
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )
        if joint_name != "gripper":
            self._store.update_motor_state(
                joint_name="gripper",
                position=float(msg.position),
                velocity=float(msg.velocity),
                torque=float(msg.torque),
                status_code=int(msg.status_code),
            )

    def _on_arm_status(self, msg: ArmStatus) -> None:
        self._store.update_arm_status(
            mode=str(msg.mode),
            enabled=bool(msg.enabled),
            state_machine=str(msg.state_machine),
            error_codes=tuple(str(v) for v in msg.error_codes),
        )
        self._update_gravity_comp_status()

    def _on_status(self, key: str, msg: String) -> None:
        try:
            value = json.loads(msg.data)
        except Exception:
            value = msg.data
        self._store.update_teleop_status(key, value)

    def _update_gravity_comp_status(self) -> None:
        snapshot = self._store.snapshot()
        arm_state = str(snapshot.arm.get("state_machine", ""))
        try:
            start_available = bool(self._gravity_start_client.service_is_ready())
            if not start_available:
                start_available = bool(self._gravity_start_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            start_available = False
        try:
            stop_available = bool(self._gravity_stop_client.service_is_ready())
            if not stop_available:
                stop_available = bool(self._gravity_stop_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            stop_available = False
        active = arm_state == "GRAVITY_COMP"
        if active:
            state = "active"
            message = "gravity compensation is active"
        elif start_available:
            state = "ready"
            message = "gravity compensation start service is available"
        else:
            state = "unavailable"
            message = "gravity compensation services unavailable"
        recording = snapshot.teleop.get("recording")
        require_gravity = bool(recording.get("require_gravity_comp")) if isinstance(recording, dict) else True
        self._store.update_teleop_status(
            "gravity_comp",
            {
                "state": state,
                "message": message,
                "arm_state": arm_state,
                "start_service_available": start_available,
                "stop_service_available": stop_available,
                "active": active,
                "ready_for_teach_recording": (not require_gravity) or active,
                "recording_requires_gravity_comp": require_gravity,
            },
        )

    def destroy_node(self) -> bool:
        try:
            self._server.shutdown()
            self._server.server_close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopStatusPanelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()

