from __future__ import annotations

from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .command_models import PoseTarget
from .message_codec import encode_preview_command, encode_status
from .parameter_helpers import build_joint_limits, sensor_qos_kwargs
from .pose_math import quaternion_to_rpy
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

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        lower_limits = tuple(
            float(v) for v in self.get_parameter("joint_lower_limits").value
        )
        upper_limits = tuple(
            float(v) for v in self.get_parameter("joint_upper_limits").value
        )
        joint_limits = build_joint_limits(
            joint_names=joint_names,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )

        workspace_root = Path(__file__).resolve().parents[3]
        pose_solver = None
        try:
            pose_solver = PosePreviewSolver(workspace_root=workspace_root)
            self.get_logger().info("pose preview solver enabled")
        except Exception as exc:  # pragma: no cover - depends on local ROS/Python env
            self.get_logger().warn(f"pose preview solver disabled: {exc}")

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

        self.get_logger().info(f"preview node ready: namespace=/{self._arm_namespace}")
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
