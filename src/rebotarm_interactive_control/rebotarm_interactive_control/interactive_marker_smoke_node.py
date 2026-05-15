from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from .marker_builder import build_smoke_marker


class InteractiveMarkerSmokeNode(Node):
    """Minimal interactive-marker smoke test node for RViz verification."""

    def __init__(self) -> None:
        super().__init__("interactive_marker_smoke")
        self.declare_parameter(
            "interactive_marker_namespace",
            "rebotarm/interactive_control/smoke_marker",
        )
        self.declare_parameter("marker_frame_id", "base_link")
        self.declare_parameter("marker_scale", 0.35)

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
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(f"interactive markers unavailable: {exc}") from exc

        self._feedback_type = InteractiveMarkerFeedback
        namespace = str(
            self.get_parameter("interactive_marker_namespace").value
        ).strip("/")
        frame_id = str(self.get_parameter("marker_frame_id").value)
        scale = float(self.get_parameter("marker_scale").value)

        self._server = InteractiveMarkerServer(self, namespace)

        marker = build_smoke_marker(
            interactive_marker_cls=InteractiveMarker,
            control_cls=InteractiveMarkerControl,
            marker_cls=Marker,
            frame_id=frame_id,
            marker_scale=scale,
            pose=self._default_pose(),
        )

        self._server.insert(marker, feedback_callback=self._on_feedback)
        self._server.applyChanges()
        self.get_logger().info(f"smoke marker ready: /{namespace}/update")

    def _default_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = 0.55
        pose.position.z = 0.45
        pose.orientation.w = 1.0
        return pose

    def _on_feedback(self, feedback) -> None:
        event_type = int(feedback.event_type)
        pose = feedback.pose.position
        self.get_logger().info(
            f"feedback event={event_type} x={pose.x:.3f} y={pose.y:.3f} z={pose.z:.3f}"
        )

    def destroy_node(self):
        try:
            self._server.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractiveMarkerSmokeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
