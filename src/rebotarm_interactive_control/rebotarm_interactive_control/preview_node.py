from __future__ import annotations

from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .command_models import PoseTarget
from .message_codec import encode_preview_command, encode_status
from .parameter_helpers import build_joint_limits, sensor_qos_kwargs
from .pose_math import quaternion_to_rpy, rpy_to_quaternion
from .pose_preview_solver import PosePreviewSolver
from .preview_manager import PreviewManager


class PreviewNode(Node):
    """Consumes pose targets and publishes preview/status without handling marker UI."""

    def __init__(self) -> None:
        super().__init__("preview_node")

        self.declare_parameter("arm_namespace", "rebotarm")
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
        self.declare_parameter("preview_backend", "sdk")
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("moveit_ik_service", "/compute_ik")
        self.declare_parameter("marker_frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        lower_limits = tuple(
            float(v) for v in self.get_parameter("joint_lower_limits").value
        )
        upper_limits = tuple(
            float(v) for v in self.get_parameter("joint_upper_limits").value
        )
        preview_backend = str(self.get_parameter("preview_backend").value).strip().lower()
        self._preview_backend = preview_backend
        self._moveit_group_name = str(self.get_parameter("moveit_group_name").value)
        self._moveit_ik_service_name = str(
            self.get_parameter("moveit_ik_service").value
        )
        self._marker_frame_id = str(self.get_parameter("marker_frame_id").value)
        self._ee_frame_id = str(self.get_parameter("ee_frame_id").value)
        joint_limits = build_joint_limits(
            joint_names=joint_names,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )

        workspace_root = Path(__file__).resolve().parents[3]
        pose_solver = None
        if preview_backend == "sdk":
            try:
                pose_solver = PosePreviewSolver(workspace_root=workspace_root)
                self.get_logger().info("pose preview solver enabled: backend=sdk")
            except Exception as exc:  # pragma: no cover - depends on local ROS/Python env
                self.get_logger().warn(f"pose preview solver disabled: {exc}")
        elif preview_backend == "moveit":
            self.get_logger().info(
                "pose preview solver enabled: backend=moveit (service client mode)"
            )
        else:
            self.get_logger().warn(
                f"unknown preview_backend='{preview_backend}', pose solver disabled"
            )

        self._preview_manager = PreviewManager(
            joint_names=joint_names,
            joint_limits=joint_limits,
            initial_positions=tuple(0.0 for _ in joint_names),
            pose_solver=pose_solver,
        )

        self._preview_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/interactive_control/preview",
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/interactive_control/status",
            10,
        )

        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Pose,
            f"/{self._arm_namespace}/interactive_control/pose_target",
            self._on_pose_target,
            10,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self._pending_pose_target: PoseTarget | None = None
        self._moveit_ik_client = None
        if self._preview_backend == "moveit":
            self._moveit_ik_client = self.create_client(
                GetPositionIK,
                self._moveit_ik_service_name,
            )

        self.get_logger().info(
            f"preview node ready: namespace=/{self._arm_namespace}, backend={preview_backend}"
        )
        self._publish_status("preview node idle")

    def _on_pose_target(self, msg: Pose) -> None:
        roll, pitch, yaw = quaternion_to_rpy(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        pose_target = PoseTarget(
            x=float(msg.position.x),
            y=float(msg.position.y),
            z=float(msg.position.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        if self._preview_backend == "moveit":
            self._request_moveit_preview(pose_target)
            return

        preview = self._preview_manager.preview_pose_target(pose_target)
        self._publish_preview(preview)
        if preview.reachable:
            self._publish_status("preview ready")
        else:
            self._publish_status(preview.message)

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        self._preview_manager.sync_current_positions(
            {
                str(name): float(pos)
                for name, pos in zip(msg.name, msg.position)
            }
        )

    def _publish_preview(self, preview) -> None:
        msg = String()
        msg.data = encode_preview_command(preview, state="preview_ready")
        self._preview_pub.publish(msg)

    def _publish_status(self, message: str) -> None:
        msg = String()
        msg.data = encode_status(mode="simulation", state="idle", message=message)
        self._status_pub.publish(msg)

    def _request_moveit_preview(self, pose_target: PoseTarget) -> None:
        if self._moveit_ik_client is None:
            self._publish_status("moveit preview backend not initialized")
            return
        if not self._moveit_ik_client.wait_for_service(timeout_sec=0.2):
            self._publish_status("moveit compute_ik service unavailable")
            return

        self._pending_pose_target = pose_target
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._moveit_group_name
        request.ik_request.ik_link_name = self._ee_frame_id
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = self._marker_frame_id
        request.ik_request.pose_stamped.pose.position.x = pose_target.x
        request.ik_request.pose_stamped.pose.position.y = pose_target.y
        request.ik_request.pose_stamped.pose.position.z = pose_target.z
        qx, qy, qz, qw = rpy_to_quaternion(
            pose_target.roll,
            pose_target.pitch,
            pose_target.yaw,
        )
        request.ik_request.pose_stamped.pose.orientation.x = qx
        request.ik_request.pose_stamped.pose.orientation.y = qy
        request.ik_request.pose_stamped.pose.orientation.z = qz
        request.ik_request.pose_stamped.pose.orientation.w = qw
        request.ik_request.robot_state.joint_state.name = list(
            self._preview_manager.joint_names
        )
        request.ik_request.robot_state.joint_state.position = list(
            self._preview_manager.current_positions
        )
        request.ik_request.avoid_collisions = True
        request.ik_request.constraints = Constraints()
        for joint_name, current in zip(
            self._preview_manager.joint_names,
            self._preview_manager.current_positions,
        ):
            lower, upper = self._preview_manager._joint_limits[joint_name]  # noqa: SLF001
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = current
            constraint.tolerance_below = max(0.0, current - lower)
            constraint.tolerance_above = max(0.0, upper - current)
            constraint.weight = 1.0
            request.ik_request.constraints.joint_constraints.append(constraint)

        future = self._moveit_ik_client.call_async(request)
        future.add_done_callback(self._on_moveit_ik_response)
        self._publish_status("moveit preview solving")

    def _on_moveit_ik_response(self, future) -> None:
        pose_target = self._pending_pose_target
        self._pending_pose_target = None
        if pose_target is None:
            return
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            preview = self._preview_manager.preview_pose_target(pose_target)
            self._publish_preview(preview)
            self._publish_status(f"moveit preview request failed: {exc}")
            return

        error_code = int(response.error_code.val)
        if error_code != 1:
            preview = self._preview_manager.preview_pose_target(pose_target)
            self._publish_preview(preview)
            self._publish_status(f"moveit preview failed: error_code={error_code}")
            return

        name_to_position = {
            str(name): float(pos)
            for name, pos in zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        }
        joint_positions = tuple(
            name_to_position.get(name, current)
            for name, current in zip(
                self._preview_manager.joint_names,
                self._preview_manager.current_positions,
            )
        )
        preview = self._preview_manager.preview_joint_target(
            {
                name: position
                for name, position in zip(self._preview_manager.joint_names, joint_positions)
            }
        )
        preview = type(preview)(
            command_type="pose",
            reachable=True,
            message="moveit ik preview ready",
            joint_names=preview.joint_names,
            joint_positions=preview.joint_positions,
            pose_target=pose_target,
        )
        self._publish_preview(preview)
        self._publish_status("preview ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
