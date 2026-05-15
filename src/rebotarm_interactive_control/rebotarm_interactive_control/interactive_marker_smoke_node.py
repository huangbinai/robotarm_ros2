from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node


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

        marker = InteractiveMarker()
        marker.header.frame_id = frame_id
        marker.name = "smoke_marker"
        marker.description = "Smoke Marker"
        marker.scale = scale
        marker.pose = self._default_pose()

        move_plane = InteractiveMarkerControl()
        move_plane.name = "move_plane"
        move_plane.orientation.w = 1.0
        move_plane.orientation.x = 0.0
        move_plane.orientation.y = 1.0
        move_plane.orientation.z = 0.0
        move_plane.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker.controls.append(move_plane)

        move_axis = InteractiveMarkerControl()
        move_axis.name = "move_x"
        move_axis.orientation.w = 1.0
        move_axis.orientation.x = 1.0
        move_axis.orientation.y = 0.0
        move_axis.orientation.z = 0.0
        move_axis.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move_axis)

        rotate_axis = InteractiveMarkerControl()
        rotate_axis.name = "rotate_z"
        rotate_axis.orientation.w = 1.0
        rotate_axis.orientation.x = 0.0
        rotate_axis.orientation.y = 1.0
        rotate_axis.orientation.z = 0.0
        rotate_axis.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(rotate_axis)

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = scale * 0.45
        sphere.scale.y = scale * 0.45
        sphere.scale.z = scale * 0.45
        sphere.color.r = 1.0
        sphere.color.g = 0.2
        sphere.color.b = 0.2
        sphere.color.a = 1.0
        visual.markers.append(sphere)
        marker.controls.append(visual)

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
