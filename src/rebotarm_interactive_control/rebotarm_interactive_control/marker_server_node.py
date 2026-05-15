from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from .marker_builder import build_ee_target_marker


class MarkerServerNode(Node):
    """Owns the formal ee_target interactive marker and publishes pose targets."""

    def __init__(self) -> None:
        super().__init__("marker_server")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("marker_frame_id", "base_link")
        self.declare_parameter(
            "interactive_marker_namespace",
            "rebotarm/interactive_control/ee_target",
        )
        self.declare_parameter("interactive_marker_name", "ee_target")
        self.declare_parameter("interactive_marker_scale", 0.28)
        self.declare_parameter("default_marker_x", 0.5)
        self.declare_parameter("default_marker_y", 0.0)
        self.declare_parameter("default_marker_z", 0.5)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._marker_frame_id = str(self.get_parameter("marker_frame_id").value)
        self._marker_namespace = str(
            self.get_parameter("interactive_marker_namespace").value
        ).strip("/")
        self._marker_name = str(self.get_parameter("interactive_marker_name").value)
        self._marker_scale = float(self.get_parameter("interactive_marker_scale").value)
        self._default_marker_x = float(self.get_parameter("default_marker_x").value)
        self._default_marker_y = float(self.get_parameter("default_marker_y").value)
        self._default_marker_z = float(self.get_parameter("default_marker_z").value)

        self._pose_target_pub = self.create_publisher(
            Pose,
            f"/{self._arm_namespace}/interactive_control/pose_target",
            10,
        )
        self._interactive_server = None
        self._interactive_classes = None

        self._setup_interactive_marker()
        self.get_logger().info(
            f"marker server ready: namespace=/{self._arm_namespace}, "
            f"marker=/{self._marker_namespace}/update"
        )

    def _setup_interactive_marker(self) -> None:
        try:
            from interactive_markers.interactive_marker_server import (  # pylint: disable=import-outside-toplevel
                InteractiveMarkerServer,
            )
            from visualization_msgs.msg import (  # pylint: disable=import-outside-toplevel
                InteractiveMarker,
                InteractiveMarkerControl,
                InteractiveMarkerFeedback,
                Marker,
            )
        except Exception as exc:  # pragma: no cover - depends on ROS env
            self.get_logger().warn(f"interactive marker disabled: {exc}")
            return

        self._interactive_classes = (
            InteractiveMarker,
            InteractiveMarkerControl,
            InteractiveMarkerFeedback,
            Marker,
        )
        self._interactive_server = InteractiveMarkerServer(self, self._marker_namespace)
        marker = build_ee_target_marker(
            interactive_marker_cls=InteractiveMarker,
            control_cls=InteractiveMarkerControl,
            marker_cls=Marker,
            frame_id=self._marker_frame_id,
            marker_name=self._marker_name,
            marker_scale=self._marker_scale,
            pose=self._default_pose_message(),
        )
        self._interactive_server.insert(marker, feedback_callback=self._on_marker_feedback)
        self._interactive_server.applyChanges()
        self.get_logger().info(
            f"interactive marker ready: /{self._marker_namespace}/update"
        )

    def _default_pose_message(self) -> Pose:
        pose = Pose()
        pose.position.x = self._default_marker_x
        pose.position.y = self._default_marker_y
        pose.position.z = self._default_marker_z
        pose.orientation.w = 1.0
        return pose

    def _on_marker_feedback(self, feedback) -> None:
        if self._interactive_classes is None:
            return
        _InteractiveMarker, _InteractiveMarkerControl, InteractiveMarkerFeedback, _Marker = (
            self._interactive_classes
        )
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        pose = Pose()
        pose.position = feedback.pose.position
        pose.orientation = feedback.pose.orientation
        self._pose_target_pub.publish(pose)

    def destroy_node(self):
        if self._interactive_server is not None:
            try:
                self._interactive_server.shutdown()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerServerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
