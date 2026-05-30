from __future__ import annotations

from copy import deepcopy
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspPlan
from rebotarm_msgs.srv import ExecutePose, SetGripper

from .grasp_preview_sender_node import (
    _transform_from_msg,
    apply_tcp_offset_to_pose,
    transform_pose_message,
)
from .gripper_policy import GripperPolicyConfig, resolve_gripper_command
from .retreat_policy import RetreatPolicyConfig
from .visual_grasp_pose_policy import BaseAxisGraspPolicyConfig, build_base_axis_grasp_targets
from .visual_grasp_sequence import (
    PoseTarget,
    VisualGraspSequenceConfig,
    VisualGraspStage,
    build_visual_grasp_sequence,
)


def pose_to_target(pose: Pose) -> PoseTarget:
    return PoseTarget(
        position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        orientation=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def target_to_pose_stamped(target: PoseTarget, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = float(target.position[0])
    msg.pose.position.y = float(target.position[1])
    msg.pose.position.z = float(target.position[2])
    msg.pose.orientation.x = float(target.orientation[0])
    msg.pose.orientation.y = float(target.orientation[1])
    msg.pose.orientation.z = float(target.orientation[2])
    msg.pose.orientation.w = float(target.orientation[3])
    return msg


class VisualGraspExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_visual_grasp_executor")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("input_topic", "/grasp/plan")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tcp_offset_xyz", [-0.04, 0.0, 0.0])
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.01, 0.0])
        self.declare_parameter("pregrasp_base_z_offset_m", 0.05)
        self.declare_parameter("grasp_base_z_offset_m", 0.0)
        self.declare_parameter("pose_policy", "base_axis")
        self.declare_parameter("fixed_grasp_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        self.declare_parameter("min_grasp_z_m", 0.12)
        self.declare_parameter("lift_z_m", 0.08)
        self.declare_parameter("open_before_approach", False)
        self.declare_parameter("open_position_m", 0.09)
        self.declare_parameter("close_position_m", 0.025)
        self.declare_parameter("close_max_effort", 0.3)
        self.declare_parameter("auto_gripper_width", True)
        self.declare_parameter("open_clearance_m", 0.02)
        self.declare_parameter("close_margin_m", 0.012)
        self.declare_parameter("min_open_position_m", 0.035)
        self.declare_parameter("max_open_position_m", 0.09)
        self.declare_parameter("min_close_position_m", 0.006)
        self.declare_parameter("max_close_position_m", 0.08)
        self.declare_parameter("auto_gripper_effort", True)
        self.declare_parameter("min_gripper_effort", 0.22)
        self.declare_parameter("max_gripper_effort", 0.60)
        self.declare_parameter("gripper_effort_per_width_m", 3.0)
        self.declare_parameter("gripper_effort_per_length_m", 0.3)
        self.declare_parameter("max_allowed_grasp_width_m", 0.085)
        self.declare_parameter("safe_retreat_enabled", True)
        self.declare_parameter("safe_retreat_min_lift_z_m", 0.24)
        self.declare_parameter("safe_retreat_distance_m", 0.06)
        self.declare_parameter("safe_retreat_axis_xyz", [-1.0, 0.0, 0.0])
        self.declare_parameter("safe_home_after_grasp", False)
        self.declare_parameter("service_timeout_sec", 20.0)
        self.declare_parameter("pregrasp_wait_sec", 0.5)
        self.declare_parameter("approach_wait_sec", 0.2)
        self.declare_parameter("execute_gripper", True)
        self.declare_parameter("gripper_wait_sec", 1.0)
        self.declare_parameter("lift_wait_sec", 0.5)
        self.declare_parameter("motion_result_timeout_sec", 45.0)
        self.declare_parameter("execution_mode", "plan_only")
        self.declare_parameter("move_velocity_scaling", 0.10)
        self.declare_parameter("approach_velocity_scaling", 0.04)
        self.declare_parameter("lift_velocity_scaling", 0.08)
        self.declare_parameter("acceleration_scaling", 0.08)
        self.declare_parameter("plan_only_stage_pause_sec", 3.0)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._tcp_offset_xyz = self._tuple3("tcp_offset_xyz")
        self._target_base_offset_xyz = self._tuple3("target_base_offset_xyz")
        self._pregrasp_base_z_offset_m = float(self.get_parameter("pregrasp_base_z_offset_m").value)
        self._grasp_base_z_offset_m = float(self.get_parameter("grasp_base_z_offset_m").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self._motion_result_timeout_sec = float(self.get_parameter("motion_result_timeout_sec").value)
        self._execution_mode = str(self.get_parameter("execution_mode").value).strip().lower()
        self._stage_waits = {
            "move_to_pregrasp": float(self.get_parameter("pregrasp_wait_sec").value),
            "approach_grasp": float(self.get_parameter("approach_wait_sec").value),
            "close_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "open_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "lift": float(self.get_parameter("lift_wait_sec").value),
            "safe_retreat": float(self.get_parameter("lift_wait_sec").value),
        }

        self._latest_plan: GraspPlan | None = None
        self._running = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._execute_pose_client = self.create_client(
            ExecutePose,
            f"/{self._arm_namespace}/motion_execution/execute_pose",
            callback_group=self._callback_group,
        )
        self._motion_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/motion_execution/stop",
            callback_group=self._callback_group,
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
            callback_group=self._callback_group,
        )
        self._safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
            callback_group=self._callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            f"/{self._arm_namespace}/gripper/set",
            callback_group=self._callback_group,
        )
        self.create_subscription(GraspPlan, self._input_topic, self._on_plan, 10, callback_group=self._callback_group)
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/execute", self._execute_visual_grasp, callback_group=self._callback_group)
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/stop", self._stop_visual_grasp, callback_group=self._callback_group)
        self.get_logger().info(
            "visual grasp executor ready: "
            f"input={self._input_topic}, namespace=/{self._arm_namespace}, target_frame={self._target_frame}, "
            "motion_execution=/motion_execution/execute_pose"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = self._tuple_n(name, 3)
        return (values[0], values[1], values[2])

    def _tuple_n(self, name: str, expected_len: int) -> tuple[float, ...]:
        values = list(self.get_parameter(name).value)
        if len(values) != expected_len:
            raise ValueError(f"{name} must contain exactly {expected_len} values")
        return tuple(float(value) for value in values)

    def _on_plan(self, plan: GraspPlan) -> None:
        if plan.valid:
            self._latest_plan = deepcopy(plan)

    def _execute_visual_grasp(self, _request, response):
        if self._running:
            response.success = False
            response.message = "visual grasp already running"
            return response
        if self._latest_plan is None:
            response.success = False
            response.message = "no valid grasp plan received"
            return response
        self._running = True
        try:
            stages = self._build_sequence_from_plan(self._latest_plan)
            for stage in stages:
                ok, message = self._run_stage(stage)
                if not ok:
                    self._request_stop()
                    response.success = False
                    response.message = f"{stage.name} failed: {message}"
                    return response
            response.success = True
            response.message = "visual grasp sequence finished"
            return response
        except Exception as exc:
            self._request_stop()
            response.success = False
            response.message = f"visual grasp failed: {exc}"
            return response
        finally:
            self._running = False

    def _stop_visual_grasp(self, _request, response):
        self._running = False
        self._request_stop()
        response.success = True
        response.message = "visual grasp stop requested"
        return response

    def _build_sequence_from_plan(self, plan: GraspPlan) -> list[VisualGraspStage]:
        pregrasp, grasp = self._build_motion_targets(plan)
        gripper_command = resolve_gripper_command(
            jaw_width_m=self._detected_jaw_width(plan),
            object_length_m=self._detected_object_length(plan),
            class_name=str(getattr(plan.candidate, "class_name", "") or ""),
            config=GripperPolicyConfig(
                auto_width=bool(self.get_parameter("auto_gripper_width").value),
                auto_effort=bool(self.get_parameter("auto_gripper_effort").value),
                default_open_width_m=float(self.get_parameter("open_position_m").value),
                default_close_width_m=float(self.get_parameter("close_position_m").value),
                default_max_effort=float(self.get_parameter("close_max_effort").value),
                open_clearance_m=float(self.get_parameter("open_clearance_m").value),
                close_margin_m=float(self.get_parameter("close_margin_m").value),
                min_open_width_m=float(self.get_parameter("min_open_position_m").value),
                max_open_width_m=float(self.get_parameter("max_open_position_m").value),
                min_close_width_m=float(self.get_parameter("min_close_position_m").value),
                max_close_width_m=float(self.get_parameter("max_close_position_m").value),
                min_effort=float(self.get_parameter("min_gripper_effort").value),
                max_effort=float(self.get_parameter("max_gripper_effort").value),
                effort_per_width_m=float(self.get_parameter("gripper_effort_per_width_m").value),
                effort_per_length_m=float(self.get_parameter("gripper_effort_per_length_m").value),
                max_allowed_width_m=float(self.get_parameter("max_allowed_grasp_width_m").value),
            ),
        )
        config = VisualGraspSequenceConfig(
            open_before_approach=bool(self.get_parameter("open_before_approach").value),
            open_position_m=float(self.get_parameter("open_position_m").value),
            close_position_m=float(self.get_parameter("close_position_m").value),
            close_max_effort=float(self.get_parameter("close_max_effort").value),
            lift_z_m=float(self.get_parameter("lift_z_m").value),
            min_grasp_z_m=float(self.get_parameter("min_grasp_z_m").value),
            auto_gripper_width=bool(self.get_parameter("auto_gripper_width").value),
            detected_jaw_width_m=self._detected_jaw_width(plan),
            open_clearance_m=float(self.get_parameter("open_clearance_m").value),
            close_margin_m=float(self.get_parameter("close_margin_m").value),
            min_open_position_m=float(self.get_parameter("min_open_position_m").value),
            max_open_position_m=float(self.get_parameter("max_open_position_m").value),
            min_close_position_m=float(self.get_parameter("min_close_position_m").value),
            max_close_position_m=float(self.get_parameter("max_close_position_m").value),
            gripper_command=gripper_command,
            retreat_policy=RetreatPolicyConfig(
                enabled=bool(self.get_parameter("safe_retreat_enabled").value),
                min_lift_z_m=float(self.get_parameter("safe_retreat_min_lift_z_m").value),
                retreat_distance_m=float(self.get_parameter("safe_retreat_distance_m").value),
                retreat_axis_xyz=self._tuple3("safe_retreat_axis_xyz"),
            ),
            include_safe_home=bool(self.get_parameter("safe_home_after_grasp").value),
        )
        return build_visual_grasp_sequence(pregrasp, grasp, config)


    def _build_motion_targets(self, plan: GraspPlan) -> tuple[PoseTarget, PoseTarget]:
        policy = str(self.get_parameter("pose_policy").value).strip().lower()
        if policy in ("visual_pose", "source_pose", "legacy"):
            return (
                self._convert_plan_pose(plan, plan.pregrasp_pose, self._pregrasp_base_z_offset_m),
                self._convert_plan_pose(plan, plan.grasp_pose, self._grasp_base_z_offset_m),
            )
        if policy != "base_axis":
            raise ValueError(f"unsupported pose_policy: {policy}")
        grasp_pose = self._transform_plan_pose_to_target_frame(plan, plan.grasp_pose)
        return build_base_axis_grasp_targets(
            grasp_position_xyz=(
                float(grasp_pose.position.x),
                float(grasp_pose.position.y),
                float(grasp_pose.position.z),
            ),
            config=BaseAxisGraspPolicyConfig(
                fixed_orientation_xyzw=self._tuple_n("fixed_grasp_orientation_xyzw", 4),
                approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
                pregrasp_distance_m=float(self.get_parameter("base_pregrasp_distance_m").value),
                tcp_offset_xyz=self._tcp_offset_xyz,
                target_base_offset_xyz=self._target_base_offset_xyz,
                pregrasp_z_offset_m=self._pregrasp_base_z_offset_m,
                grasp_z_offset_m=self._grasp_base_z_offset_m,
            ),
        )

    def _transform_plan_pose_to_target_frame(self, plan: GraspPlan, pose: Pose) -> Pose:
        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            converted = transform_pose_message(converted, _transform_from_msg(tf_msg))
        return converted

    def _convert_plan_pose(self, plan: GraspPlan, pose: Pose, base_z_offset_m: float) -> PoseTarget:
        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            converted = transform_pose_message(converted, _transform_from_msg(tf_msg))
        converted = apply_tcp_offset_to_pose(converted, self._tcp_offset_xyz)
        converted.position.x = round(float(converted.position.x) + self._target_base_offset_xyz[0], 6)
        converted.position.y = round(float(converted.position.y) + self._target_base_offset_xyz[1], 6)
        converted.position.z = round(float(converted.position.z) + self._target_base_offset_xyz[2] + base_z_offset_m, 6)
        return pose_to_target(converted)

    def _run_stage(self, stage: VisualGraspStage) -> tuple[bool, str]:
        self.get_logger().info(f"visual grasp stage: {stage.name}")
        if stage.kind == "move":
            if stage.pose is None:
                return False, "missing move pose"
            ok, message = self._call_execute_pose(stage)
        elif stage.kind == "gripper":
            if not self._execution_enabled():
                ok, message = True, "plan_only: gripper command skipped"
            elif self._gripper_execution_enabled():
                ok, message = self._call_gripper(stage)
            else:
                ok, message = True, "simulation: gripper command skipped"
        elif stage.kind == "safe_home":
            ok, message = self._call_safe_home()
        else:
            return False, f"unsupported stage kind: {stage.kind}"
        if not ok:
            return False, message
        wait_sec = self._stage_waits.get(stage.name, 0.0)
        if not self._execution_enabled() and stage.kind == "move":
            wait_sec = max(wait_sec, float(self.get_parameter("plan_only_stage_pause_sec").value))
        time.sleep(max(0.0, wait_sec))
        if not self._running:
            return False, "stopped"
        return True, message

    def _call_execute_pose(self, stage: VisualGraspStage) -> tuple[bool, str]:
        if stage.pose is None:
            return False, "missing move pose"
        if not self._execute_pose_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "motion execution service unavailable"
        request = ExecutePose.Request()
        request.target_pose = target_to_pose_stamped(stage.pose, self._target_frame)
        request.velocity_scaling = self._velocity_scaling_for_stage(stage.name)
        request.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        request.timeout_sec = self._motion_result_timeout_sec
        request.execute = self._execution_enabled()
        future = self._execute_pose_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec + self._motion_result_timeout_sec):
            return False, "motion execution service call timed out"
        result = future.result()
        if result is None:
            return False, "motion execution returned no result"
        return bool(result.success), f"{result.stage}: {result.message}"

    def _execution_enabled(self) -> bool:
        return self._execution_mode in ("execute", "real")

    def _gripper_execution_enabled(self) -> bool:
        return self._execution_enabled() and bool(self.get_parameter("execute_gripper").value)

    def _detected_jaw_width(self, plan: GraspPlan) -> float:
        plan_width = float(getattr(plan, "jaw_width", 0.0) or 0.0)
        candidate_width = float(getattr(plan.candidate, "jaw_width", 0.0) or 0.0)
        return plan_width if plan_width > 0.0 else candidate_width

    def _detected_object_length(self, plan: GraspPlan) -> float:
        return float(getattr(plan.candidate, "object_length", 0.0) or 0.0)

    def _velocity_scaling_for_stage(self, name: str) -> float:
        if name == "approach_grasp":
            return float(self.get_parameter("approach_velocity_scaling").value)
        if name == "lift":
            return float(self.get_parameter("lift_velocity_scaling").value)
        if name == "safe_retreat":
            return float(self.get_parameter("lift_velocity_scaling").value)
        return float(self.get_parameter("move_velocity_scaling").value)

    def _call_gripper(self, stage: VisualGraspStage) -> tuple[bool, str]:
        if stage.gripper_position_m is None or stage.gripper_max_effort is None:
            return False, "missing gripper target"
        if not self._gripper_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "gripper service unavailable"
        request = SetGripper.Request()
        request.position = float(stage.gripper_position_m)
        request.max_effort = float(stage.gripper_max_effort)
        future = self._gripper_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec):
            return False, "gripper service call timed out"
        result = future.result()
        if result is None:
            return False, "gripper service returned no result"
        return bool(result.success), f"reached_position={result.reached_position:.4f}"

    def _call_safe_home(self) -> tuple[bool, str]:
        if not self._execution_enabled():
            return True, "plan_only: safe_home skipped"
        if not self._safe_home_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "safe_home service unavailable"
        future = self._safe_home_client.call_async(Trigger.Request())
        if not self._wait_for_future(future, self._service_timeout_sec + self._motion_result_timeout_sec):
            return False, "safe_home service call timed out"
        result = future.result()
        if result is None:
            return False, "safe_home returned no result"
        return bool(result.success), str(result.message)

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if not self._running:
                return False
            time.sleep(0.02)
        return future.done()

    def _request_stop(self) -> None:
        self._request_stop_service(self._motion_stop_client, "motion execution stop")
        self._request_stop_service(self._trajectory_stop_client, "trajectory_stop")

    def _request_stop_service(self, client, label: str) -> None:
        try:
            if not client.wait_for_service(timeout_sec=0.2):
                return
            client.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().warn(f"failed to request {label}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualGraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
