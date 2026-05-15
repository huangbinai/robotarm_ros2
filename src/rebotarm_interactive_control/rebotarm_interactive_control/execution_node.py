from __future__ import annotations

import ast

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarm_msgs.srv import SetMode

from .command_models import ExecutionState, PreviewCommand
from .execution_coordinator import InteractiveCoordinator
from .mode_manager import parse_control_mode
from .preview_manager import PreviewManager


class ExecutionNode(Node):
    """Owns execute/estop/set_mode services using the latest preview output."""

    def __init__(self) -> None:
        super().__init__("execution_node")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("mode", "simulation")
        self.declare_parameter("default_move_duration", 2.0)
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
        )

        self.create_subscription(
            String,
            f"/{self._arm_namespace}/interactive_control/preview",
            self._on_preview,
            10,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/execute_preview",
            self._execute_preview,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/estop",
            self._trigger_estop,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/reset_estop",
            self._reset_estop,
        )
        self.create_service(
            SetMode,
            f"/{self._arm_namespace}/interactive_control/set_mode",
            self._set_mode,
        )

        self.get_logger().info(f"execution node ready: namespace=/{self._arm_namespace}")
        self._publish_status("execution node idle")

    def _on_preview(self, msg: String) -> None:
        parsed = self._parse_preview_payload(msg.data)
        if parsed is None:
            return
        self._coordinator._last_preview = parsed  # noqa: SLF001
        self._coordinator._execution_state = (  # noqa: SLF001
            ExecutionState.PREVIEW_READY
            if parsed.reachable
            else ExecutionState.IDLE
        )

    def _parse_preview_payload(self, payload: str) -> PreviewCommand | None:
        try:
            pieces = [segment.strip() for segment in payload.split(";")]
            values = {}
            for piece in pieces:
                if "=" not in piece:
                    continue
                key, value = piece.split("=", 1)
                values[key.strip()] = value.strip()
            joints_literal = values.get("joints")
            if joints_literal is None:
                return None
            joint_positions = tuple(float(v) for v in ast.literal_eval(joints_literal))
            message = values.get("message", "")
            reachable = "unreachable" not in message.lower() and "unavailable" not in message.lower()
            joint_names = self._coordinator._preview_manager.joint_names  # noqa: SLF001
            return PreviewCommand(
                command_type="pose",
                reachable=reachable,
                message=message,
                joint_names=joint_names,
                joint_positions=joint_positions,
            )
        except Exception:
            return None

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

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(request.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(request.joint_positions)
        secs = max(0.0, float(request.duration))
        point.time_from_start.sec = int(secs)
        point.time_from_start.nanosec = int((secs - int(secs)) * 1e9)
        goal.trajectory.points = [point]

        send_future = self._trajectory_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        self._publish_status("real execution goal sent")
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
        payload = (
            f"mode={self._coordinator.mode.value}; "
            f"state={self._coordinator.execution_state.value}; "
            f"message={message}"
        )
        msg = String()
        msg.data = payload
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
