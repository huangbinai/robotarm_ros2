from __future__ import annotations

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rebotarm_msgs.srv import SetMode

from .command_models import ExecutionState
from .execution_coordinator import InteractiveCoordinator
from .message_codec import decode_preview_command, encode_status
from .mode_manager import parse_control_mode
from .moveit_planner import MoveItMotionPlanner
from .preview_manager import PreviewManager


class ExecutionNode(Node):
    """Owns execute/estop/set_mode services using the latest preview output."""

    def __init__(self) -> None:
        super().__init__("execution_node")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("mode", "simulation")
        self.declare_parameter("default_move_duration", 2.0)
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        self.declare_parameter("moveit_planner_id", "")
        self.declare_parameter("moveit_planning_time", 2.0)
        self.declare_parameter("moveit_num_planning_attempts", 1)
        self.declare_parameter("marker_frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("goal_position_tolerance", 0.005)
        self.declare_parameter("goal_orientation_tolerance", 0.02)
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

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._default_duration = float(self.get_parameter("default_move_duration").value)
        self._moveit_group_name = str(self.get_parameter("moveit_group_name").value)
        self._moveit_planning_service = str(
            self.get_parameter("moveit_planning_service").value
        )
        self._moveit_planning_pipeline = str(
            self.get_parameter("moveit_planning_pipeline").value
        )
        self._moveit_planner_id = str(self.get_parameter("moveit_planner_id").value)
        self._marker_frame_id = str(self.get_parameter("marker_frame_id").value)
        self._ee_frame_id = str(self.get_parameter("ee_frame_id").value)
        joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        lower_limits = tuple(
            float(v) for v in self.get_parameter("joint_lower_limits").value
        )
        upper_limits = tuple(
            float(v) for v in self.get_parameter("joint_upper_limits").value
        )
        joint_limits = {
            name: (lower, upper)
            for name, lower, upper in zip(joint_names, lower_limits, upper_limits)
        }

        preview_manager = PreviewManager(
            joint_names=joint_names,
            joint_limits=joint_limits,
            initial_positions=tuple(0.0 for _ in joint_names),
            pose_solver=None,
        )
        self._coordinator = InteractiveCoordinator(
            preview_manager=preview_manager,
            default_mode=parse_control_mode(str(self.get_parameter("mode").value)),
        )

        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/interactive_control/status",
            10,
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
            callback_group=self._callback_group,
        )
        self._moveit_planner = MoveItMotionPlanner(
            self,
            group_name=self._moveit_group_name,
            ee_frame_id=self._ee_frame_id,
            frame_id=self._marker_frame_id,
            planning_service=self._moveit_planning_service,
            planning_pipeline=self._moveit_planning_pipeline,
            planner_id=self._moveit_planner_id,
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=float(
                self.get_parameter("goal_position_tolerance").value
            ),
            goal_orientation_tolerance=float(
                self.get_parameter("goal_orientation_tolerance").value
            ),
        )

        self.create_subscription(
            String,
            f"/{self._arm_namespace}/interactive_control/preview",
            self._on_preview,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/execute_preview",
            self._execute_preview,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/estop",
            self._trigger_estop,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/reset_estop",
            self._reset_estop,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetMode,
            f"/{self._arm_namespace}/interactive_control/set_mode",
            self._set_mode,
            callback_group=self._callback_group,
        )

        self.get_logger().info(f"execution node ready: namespace=/{self._arm_namespace}")
        self._publish_status("execution node idle")

    def _on_preview(self, msg: String) -> None:
        _state, parsed = decode_preview_command(msg.data)
        if parsed is None:
            return
        self._coordinator._last_preview = parsed  # noqa: SLF001
        self._coordinator._execution_state = (  # noqa: SLF001
            ExecutionState.PREVIEW_READY
            if parsed.reachable
            else ExecutionState.IDLE
        )

    def _execute_preview(self, _request, response):
        decision = self._coordinator.execute_preview(duration=self._default_duration)
        response.success = bool(decision.accepted)
        response.message = decision.message
        if not decision.accepted or decision.request is None:
            self._publish_status(decision.message)
            return response

        request = decision.request
        if request.preview_only:
            self._publish_status(
                "simulation preview accepted: "
                + ", ".join(f"{v:.3f}" for v in request.joint_positions)
            )
            self._coordinator.execution_finished()
            return response

        if not self._trajectory_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = "follow_joint_trajectory action unavailable"
            self._coordinator.execution_finished()
            self._publish_status(response.message)
            return response

        plan_result = self._moveit_planner.plan_preview(request.preview_command)
        if not plan_result.success or plan_result.trajectory is None:
            response.success = False
            response.message = plan_result.message
            self._coordinator.execution_finished()
            self._publish_status(response.message)
            return response

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = plan_result.trajectory

        send_future = self._trajectory_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        self._publish_status(
            f"real execution goal sent: {len(goal.trajectory.points)} trajectory points"
        )
        return response

    def _set_mode(self, request, response):
        try:
            mode = parse_control_mode(request.mode)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            self._publish_status(response.message)
            return response

        self._coordinator.set_mode(mode)
        response.success = True
        response.message = f"interactive mode set to {mode.value}"
        self._publish_status(response.message)
        return response

    def _trigger_estop(self, _request, response):
        self._coordinator.trigger_estop()
        response.success = True
        response.message = "interactive estop latched"
        self._publish_status(response.message)
        return response

    def _reset_estop(self, _request, response):
        self._coordinator.reset_estop()
        response.success = True
        response.message = "interactive estop reset"
        self._publish_status(response.message)
        return response

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._coordinator.execution_finished()
            self._publish_status(f"failed to send trajectory goal: {exc}")
            return
        if not goal_handle.accepted:
            self._coordinator.execution_finished()
            self._publish_status("trajectory goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._coordinator.execution_finished()
        try:
            wrapped_result = future.result()
            status = getattr(wrapped_result, "status", "unknown")
            result = getattr(wrapped_result, "result", None)
            message = getattr(result, "error_string", "trajectory finished")
            self._publish_status(f"trajectory result status={status}: {message}")
        except Exception as exc:  # pragma: no cover
            self._publish_status(f"trajectory result retrieval failed: {exc}")

    def _publish_status(self, message: str) -> None:
        msg = String()
        msg.data = encode_status(
            mode=self._coordinator.mode.value,
            state=self._coordinator.execution_state.value,
            message=message,
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
