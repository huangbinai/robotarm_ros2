from __future__ import annotations

import json
import signal
import time
from contextlib import suppress
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarm_motion.collision_precheck import (
    CollisionPrechecker,
)
from rebotarm_motion.moveit_planner import MoveItMotionPlanner
from rebotarm_motion.teach_replay_start_alignment import (
    MoveItStartAligner,
)
from rebotarm_motion.teach_replay_start_align_precheck import (
    MoveItStartAlignPrechecker,
)
from rebotarm_motion.trajectory_safety_monitor import evaluate_replay_tracking

from .parameter_helpers import sensor_qos_kwargs
from .teach_recording import (
    ReplayStartBand,
    analyze_teach_trajectory,
    classify_replay_start,
    compute_auto_align_duration,
    load_teach_samples,
    prepared_teach_replay_to_dict,
    teach_trajectory_quality_to_dict,
)
from .teach_replay_parameter_adapter import TeachReplayParameterAdapter
from .teach_replay_parameters import declare_teach_replay_parameters
from .teach_replay_trajectory_builder import TeachReplayTrajectoryBuilder
from .teach_replay_workflow import (
    PreparedReplayRecord,
    TeachReplayWorkflow,
)


class TeachReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("teach_replay_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("dry_run", True)
        declare_teach_replay_parameters(
            self.declare_parameter,
            speed_parameter_name="speed",
        )
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._record_path = Path(str(self.get_parameter("record_path").value))
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._teach_replay_config = TeachReplayParameterAdapter(self.get_parameter)
        self._latest_joint_state: JointState | None = None
        self._started = False
        self._samples = []
        self._start_band = ""
        self._max_error: float | None = None
        self._per_joint_error: tuple[float, ...] = ()
        self._trajectory_points = 0
        self._quality = None
        self._prepared_replay = None
        self._prepared_record: PreparedReplayRecord | None = None
        self._prepared_record_path: Path | None = None
        self._goal_handle = None
        self._active_replay_trajectory: JointTrajectory | None = None
        self._active_replay_started_at: float | None = None
        self._tracking_violation_since: float | None = None
        self._monitor_stop_requested = False
        self._stop_requested = False
        self._stop_reason = ""
        self._moveit_align_message = ""
        self._collision_precheck = {"state": "not_run", "message": "collision precheck not run"}
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
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
        self._teach_replay_workflow = TeachReplayWorkflow(
            trajectory_builder=TeachReplayTrajectoryBuilder(
                trajectory_factory=JointTrajectory,
                trajectory_point_factory=JointTrajectoryPoint,
            ),
            moveit_start_aligner=MoveItStartAligner(
                planner=self._moveit_planner,
                trajectory_point_factory=JointTrajectoryPoint,
                message_sink=self._set_moveit_align_message,
            ),
            moveit_start_align_prechecker=MoveItStartAlignPrechecker(
                planner=self._moveit_planner,
                service_client=self._moveit_planner._client,  # noqa: SLF001
            ),
            collision_prechecker=CollisionPrechecker(
                client=self._state_validity_client,
                request_factory=GetStateValidity.Request,
            ),
        )
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/replay_status",
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_timer(0.2, self._maybe_start)
        self.create_timer(
            max(float(self.get_parameter("replay_monitor_period_sec").value), 0.02),
            self._check_active_replay_tracking,
        )
        self._publish_status("ready", f"waiting to replay {self._record_path}")

    def _max_replay_velocity_limits(self, joint_names: tuple[str, ...]):
        return self._teach_replay_config.velocity_limits(joint_names)

    def _preparation_config(
        self,
        joint_names: tuple[str, ...],
    ):
        return self._teach_replay_config.preparation(
            joint_names,
            replay_speed=float(self.get_parameter("speed").value),
        )

    def _trajectory_config(
        self,
        joint_names: tuple[str, ...],
    ):
        return self._teach_replay_config.trajectory(joint_names)

    def _alignment_config(self):
        return self._teach_replay_config.alignment()

    def _collision_config(self):
        group_name = str(self.get_parameter("collision_group_name").value)
        defaults: tuple[tuple[str, float], ...] = ()
        if group_name == "arm_with_gripper":
            latest_positions = {}
            if self._latest_joint_state is not None:
                latest_positions = {
                    str(name): float(self._latest_joint_state.position[index])
                    for index, name in enumerate(self._latest_joint_state.name)
                    if index < len(self._latest_joint_state.position)
                }
            defaults = (
                ("left_finger_joint", latest_positions.get("left_finger_joint", 0.0)),
                ("right_finger_joint", latest_positions.get("right_finger_joint", -0.0)),
            )
        return self._teach_replay_config.collision(
            default_joint_positions=defaults,
        )

    def _set_moveit_align_message(self, message: str) -> None:
        self._moveit_align_message = str(message)

    def _auto_align_duration_for_positions(
        self,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        if not bool(self.get_parameter("align_duration_auto").value):
            return float(self.get_parameter("align_duration").value)
        max_error = max(
            (abs(float(current) - float(first)) for current, first in zip(current_positions, first_positions)),
            default=0.0,
        )
        return compute_auto_align_duration(
            max_error,
            target_speed_rad_s=float(self.get_parameter("align_target_speed_rad_s").value),
            min_duration_sec=float(self.get_parameter("align_min_duration").value),
            max_duration_sec=float(self.get_parameter("align_max_duration").value),
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _maybe_start(self) -> None:
        if self._started:
            return
        if self._latest_joint_state is None:
            self._publish_status("waiting", "waiting for current joint_states")
            return
        self._started = True
        try:
            self._samples = load_teach_samples(self._record_path)
        except Exception as exc:
            self._publish_status("failed", f"failed to load record: {exc}")
            return
        if not self._samples:
            self._publish_status("failed", "record contains no samples")
            return

        first = self._samples[0]
        current_map = {
            str(name): float(pos)
            for name, pos in zip(self._latest_joint_state.name, self._latest_joint_state.position)
        }
        missing = [name for name in first.joint_names if name not in current_map]
        if missing:
            self._publish_status("failed", f"current joint_state missing: {', '.join(missing)}")
            return
        current = tuple(current_map[name] for name in first.joint_names)
        decision = classify_replay_start(
            current_positions=current,
            start_positions=first.positions,
            direct_threshold=float(self.get_parameter("direct_threshold").value),
            align_threshold=float(self.get_parameter("align_threshold").value),
        )
        start_band = decision.band
        if decision.band == ReplayStartBand.REJECT and bool(self.get_parameter("use_moveit_start_align").value):
            start_band = ReplayStartBand.MOVEIT_ALIGN
        if decision.band == ReplayStartBand.REJECT:
            if start_band != ReplayStartBand.MOVEIT_ALIGN:
                self._start_band = str(decision.band.value)
                self._max_error = decision.max_error
                self._per_joint_error = decision.per_joint_error
                self._publish_status("rejected", decision.message, max_error=decision.max_error)
                return

        self._start_band = str(start_band.value)
        if start_band == ReplayStartBand.MOVEIT_ALIGN:
            self.get_logger().warn(
                "start error exceeds joint-space align threshold; using MoveIt start alignment"
            )
        else:
            self._start_band = str(decision.band.value)
            self._max_error = decision.max_error
            self._per_joint_error = decision.per_joint_error
        self._max_error = decision.max_error
        self._per_joint_error = decision.per_joint_error
        self._prepared_record = self._teach_replay_workflow.prepare_loaded_record(
            self._record_path,
            self._samples,
            config=self._preparation_config(tuple(first.joint_names)),
        )
        self._samples = self._prepared_record.source_samples
        self._prepared_replay = self._prepared_record.prepared
        self._prepared_record_path = Path(self._prepared_record.prepared_path)
        self._quality = analyze_teach_trajectory(
            self._samples,
            green_jump_rad=float(self.get_parameter("green_jump_rad").value),
            yellow_jump_rad=float(self.get_parameter("yellow_jump_rad").value),
            max_velocity_rad_s=self._max_replay_velocity_limits(
                tuple(first.joint_names)
            ),
            max_acceleration_rad_s2=float(
                self.get_parameter("max_replay_acceleration_rad_s2").value
            ),
            max_jerk_rad_s3=float(
                self.get_parameter("max_replay_jerk_rad_s3").value
            ),
        )
        speed = float(self.get_parameter("speed").value)
        yellow_max_speed = float(self.get_parameter("yellow_max_speed").value)
        replay_quality = self._prepared_replay.after_quality
        max_prepared_jump_rad = float(self.get_parameter("max_prepared_jump_rad").value)
        max_replay_acceleration = float(self.get_parameter("max_replay_acceleration_rad_s2").value)
        max_replay_jerk = float(self.get_parameter("max_replay_jerk_rad_s3").value)
        if not self._dry_run and not replay_quality.allow_real_replay:
            self._publish_status(
                "rejected",
                replay_quality.replay_policy,
                max_error=decision.max_error,
            )
            return
        if self._dry_run and not replay_quality.allow_real_replay:
            self._publish_status(
                "dry_run",
                (
                    "validated replay but real execution is blocked; "
                    f"quality={self._quality.risk_level}->{self._prepared_replay.after_quality.risk_level}"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._dry_run and float(replay_quality.max_jump_rad) > max_prepared_jump_rad:
            self._publish_status(
                "rejected",
                (
                    "prepared replay jump is still too large: "
                    f"{float(replay_quality.max_jump_rad):.4f} rad > {max_prepared_jump_rad:.4f} rad"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._dry_run and float(replay_quality.max_acceleration_rad_s2) > max_replay_acceleration:
            self._publish_status(
                "rejected",
                (
                    "retimed replay acceleration is still too large: "
                    f"{float(replay_quality.max_acceleration_rad_s2):.4f} rad/s^2 > "
                    f"{max_replay_acceleration:.4f} rad/s^2"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._dry_run and float(replay_quality.max_jerk_rad_s3) > max_replay_jerk:
            self._publish_status(
                "rejected",
                (
                    "retimed replay jerk is still too large: "
                    f"{float(replay_quality.max_jerk_rad_s3):.4f} rad/s^3 > "
                    f"{max_replay_jerk:.4f} rad/s^3"
                ),
                max_error=decision.max_error,
            )
            return
        effective_speed = (
            float(self._prepared_replay.effective_replay_speed)
            if self._prepared_replay is not None
            else speed
        )
        if not self._dry_run and replay_quality.risk_level == "yellow" and effective_speed > yellow_max_speed:
            self._publish_status(
                "rejected",
                f"prepared yellow replay speed must be <= {yellow_max_speed:.2f}",
                max_error=decision.max_error,
            )
            return
        try:
            trajectory = self._build_trajectory(current, start_band)
        except RuntimeError as exc:
            self._publish_status("blocked", str(exc), max_error=decision.max_error)
            return
        self._trajectory_points = len(trajectory.points)
        self._collision_precheck = self._check_trajectory_collision(trajectory)
        collision_state = str(self._collision_precheck.get("state", "")).lower()
        if collision_state in ("collision", "unknown") and not self._dry_run:
            self._publish_status(
                "blocked",
                f"collision precheck blocked replay: {self._collision_precheck.get('message', collision_state)}",
                max_error=decision.max_error,
            )
            return
        if self._dry_run:
            dry_state = "dry_run" if collision_state not in ("collision", "unknown") else "blocked"
            collision_suffix = (
                ""
                if dry_state == "dry_run"
                else f"; collision precheck blocked replay: {self._collision_precheck.get('message', collision_state)}"
            )
            self._publish_status(
                dry_state,
                (
                    f"validated replay with {len(trajectory.points)} trajectory points; "
                    f"quality={self._quality.risk_level}->{self._prepared_replay.after_quality.risk_level}"
                    f"{collision_suffix}"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self._publish_status("failed", "follow_joint_trajectory action unavailable")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_response(fut, trajectory))

    def _on_goal_response(self, future, trajectory: JointTrajectory) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status("failed", f"failed to send replay trajectory: {exc}")
            return
        if not goal_handle.accepted:
            self._publish_status("rejected", "replay trajectory goal rejected")
            return
        self._goal_handle = goal_handle
        self._active_replay_trajectory = trajectory
        self._active_replay_started_at = time.monotonic()
        self._tracking_violation_since = None
        self._monitor_stop_requested = False
        self._publish_status("replaying", "trajectory goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_replay_result)

    def _on_replay_result(self, future) -> None:
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._publish_status("failed", f"replay result retrieval failed: {exc}")
            self._goal_handle = None
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        self._publish_status(
            state,
            f"replay result status={status}, error_code={error_code}: {error_string}",
        )
        self._goal_handle = None
        self._active_replay_trajectory = None
        self._active_replay_started_at = None
        self._tracking_violation_since = None
        self._monitor_stop_requested = False

    def request_stop(self, reason: str) -> None:
        self._stop_requested = True
        self._stop_reason = reason

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    def cancel_active_goal(self, *, timeout_sec: float = 2.0) -> bool:
        goal_handle = self._goal_handle
        reason = self._stop_reason or "stop requested"
        self._publish_status("cancel_requested", f"{reason}; stopping replay trajectory")
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=min(timeout_sec, 0.8))
        cancel_requested = False
        if goal_handle is not None:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_requested = True
            except Exception as exc:
                self._publish_status("failed", f"failed to request replay cancel: {exc}")
                return stop_requested
            with suppress(Exception, KeyboardInterrupt):
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=timeout_sec)
        return stop_requested or cancel_requested

    def _check_active_replay_tracking(self) -> None:
        if not bool(self.get_parameter("replay_monitor_enabled").value):
            return
        if self._dry_run or self._goal_handle is None or self._active_replay_trajectory is None:
            return
        if self._latest_joint_state is None or self._active_replay_started_at is None:
            return
        now = time.monotonic()
        elapsed = now - self._active_replay_started_at
        if elapsed < float(self.get_parameter("replay_monitor_start_grace_sec").value):
            return
        result = evaluate_replay_tracking(
            self._active_replay_trajectory,
            joint_names=tuple(self._latest_joint_state.name),
            positions=tuple(float(v) for v in self._latest_joint_state.position),
            velocities=tuple(float(v) for v in self._latest_joint_state.velocity),
            elapsed_sec=elapsed,
            max_tracking_error_rad=float(self.get_parameter("max_tracking_error_rad").value),
            max_live_velocity_rad_s=float(self.get_parameter("max_live_velocity_rad_s").value),
        )
        if result.ok:
            self._tracking_violation_since = None
            return
        if self._tracking_violation_since is None:
            self._tracking_violation_since = now
            return
        if now - self._tracking_violation_since < float(self.get_parameter("replay_monitor_violation_grace_sec").value):
            return
        if self._monitor_stop_requested:
            return
        self._monitor_stop_requested = True
        self._publish_status(
            "safety_stop",
            f"runtime replay monitor stopped trajectory: {result.message}",
        )
        self._request_controller_trajectory_stop(timeout_sec=0.2)
        try:
            self._goal_handle.cancel_goal_async()
        except Exception as exc:
            self._publish_status("failed", f"failed to cancel after runtime monitor stop: {exc}")

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        try:
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=0.1):
                return False
            future = self._trajectory_stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if not future.done():
                return False
            response = future.result()
            return bool(response is not None and response.success)
        except Exception:
            return False

    def _build_trajectory(
        self,
        current_positions: tuple[float, ...],
        start_band: ReplayStartBand,
    ) -> JointTrajectory:
        if self._prepared_record is None:
            raise RuntimeError("prepared replay is unavailable")
        joint_names = tuple(self._prepared_record.prepared.samples[0].joint_names)
        current_by_name = {
            name: float(position)
            for name, position in zip(joint_names, current_positions)
        }
        settings = {
            "align_duration": self._auto_align_duration_for_positions(
                current_positions,
                tuple(self._prepared_record.prepared.samples[0].positions),
            ),
            "align_steps": int(self.get_parameter("align_steps").value),
            "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
        }
        try:
            return self._teach_replay_workflow.build_trajectory(
                self._prepared_record,
                current_positions=current_by_name,
                start_band=start_band.value,
                settings=settings,
                trajectory_config=self._trajectory_config(joint_names),
                alignment_config=self._alignment_config(),
            )
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

    def _check_trajectory_collision(self, trajectory: JointTrajectory) -> dict:
        result = self._teach_replay_workflow.check_trajectory(
            trajectory,
            config=self._collision_config(),
        )
        result.pop("added_default_joints", None)
        messages = {
            "no trajectory samples to check": "no trajectory points to collision check",
            "collision detected in teach trajectory": "collision detected in replay trajectory",
            "no collision detected in sampled teach trajectory": (
                "no collision detected in sampled replay trajectory"
            ),
        }
        result["message"] = messages.get(result.get("message"), result.get("message"))
        for collision in result.get("collisions", []):
            if "sample" in collision:
                collision["point"] = collision.pop("sample")
        return result

    def _publish_status(
        self,
        state: str,
        message: str,
        *,
        max_error: float | None = None,
    ) -> None:
        msg = String()
        payload = {
            "state": state,
            "message": message,
            "record_path": str(self._record_path),
            "dry_run": self._dry_run,
            "speed": float(self.get_parameter("speed").value),
            "samples": len(self._samples),
            "trajectory_points": self._trajectory_points,
            "start_band": self._start_band,
            "per_joint_error": list(self._per_joint_error),
            "direct_threshold": float(self.get_parameter("direct_threshold").value),
            "align_threshold": float(self.get_parameter("align_threshold").value),
            "start_hold_sec": float(self.get_parameter("start_hold_sec").value),
            "soft_start_duration": float(self.get_parameter("soft_start_duration").value),
            "soft_start_steps": int(self.get_parameter("soft_start_steps").value),
            "first_hold_sec": float(self.get_parameter("first_hold_sec").value),
            "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
            "use_moveit_start_align": bool(self.get_parameter("use_moveit_start_align").value),
            "moveit_start_align_message": self._moveit_align_message,
            "collision_precheck": self._collision_precheck,
            "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
            "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
            "filter_enabled": bool(self.get_parameter("filter_enabled").value),
            "filter_cutoff_hz": float(self.get_parameter("filter_cutoff_hz").value),
            "filter_sample_rate_hz": float(self.get_parameter("filter_sample_rate_hz").value),
            "resample_rate_hz": float(self.get_parameter("resample_rate_hz").value),
            "time_parameterization_method": str(self.get_parameter("time_parameterization_method").value),
        }
        if self._quality is not None:
            payload["quality"] = teach_trajectory_quality_to_dict(self._quality)
            payload["risk_level"] = self._quality.risk_level
        if self._prepared_replay is not None:
            payload["prepared_replay"] = prepared_teach_replay_to_dict(self._prepared_replay)
            payload["prepared_risk_level"] = self._prepared_replay.after_quality.risk_level
            payload["effective_risk_level"] = self._prepared_replay.after_quality.risk_level
        if self._prepared_record_path is not None:
            payload["prepared_record_path"] = str(self._prepared_record_path)
        if max_error is not None:
            payload["max_error"] = max_error
        elif self._max_error is not None:
            payload["max_error"] = self._max_error
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachReplayNode()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def _request_signal_stop(signum, _frame) -> None:
        node.request_stop(f"signal {int(signum)} received")

    signal.signal(signal.SIGINT, _request_signal_stop)
    signal.signal(signal.SIGTERM, _request_signal_stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.stop_requested:
            node.cancel_active_goal(timeout_sec=2.0)
    except KeyboardInterrupt:
        node.request_stop("KeyboardInterrupt received")
        node.cancel_active_goal(timeout_sec=2.0)
    finally:
        with suppress(Exception):
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()
