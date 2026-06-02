from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rebotarm_msgs.msg import GraspPlan

from .grasp_preview_sender_node import _transform_from_msg, transform_pose_message


def copy_pose(pose: Pose) -> Pose:
    copied = Pose()
    copied.position.x = float(pose.position.x)
    copied.position.y = float(pose.position.y)
    copied.position.z = float(pose.position.z)
    copied.orientation.x = float(pose.orientation.x)
    copied.orientation.y = float(pose.orientation.y)
    copied.orientation.z = float(pose.orientation.z)
    copied.orientation.w = float(pose.orientation.w)
    return copied


def identity_orientation(pose: Pose) -> Pose:
    copied = copy_pose(pose)
    copied.orientation.x = 0.0
    copied.orientation.y = 0.0
    copied.orientation.z = 0.0
    copied.orientation.w = 1.0
    return copied


def object_diameter(jaw_width: float, minimum: float) -> float:
    return max(float(minimum), min(max(float(jaw_width), 0.0) * 1.8, 0.16))


def object_height(object_length: float, minimum: float) -> float:
    return max(float(minimum), min(max(float(object_length), 0.0), 0.35))


class VisualGraspMarkerBuilder:
    """Builds RViz markers for a real GraspPlan without owning ROS subscriptions."""

    def __init__(
        self,
        *,
        object_min_diameter_m: float = 0.06,
        object_min_height_m: float = 0.12,
        upright_object_marker: bool = True,
    ) -> None:
        self._object_min_diameter_m = float(object_min_diameter_m)
        self._object_min_height_m = float(object_min_height_m)
        self._upright_object_marker = bool(upright_object_marker)

    def build(self, plan: GraspPlan, *, frame_id: str, stamp) -> MarkerArray:
        markers = MarkerArray()
        if not plan.valid:
            markers.markers.append(self._delete_all(frame_id, stamp))
            return markers

        candidate_pose = copy_pose(plan.candidate.pose)
        if self._upright_object_marker:
            candidate_pose = identity_orientation(candidate_pose)

        markers.markers.append(
            self._object_marker(
                frame_id,
                stamp,
                candidate_pose,
                jaw_width=float(plan.jaw_width or plan.candidate.jaw_width),
                object_length=float(plan.candidate.object_length),
            )
        )
        markers.markers.append(
            self._sphere_marker(
                frame_id,
                stamp,
                1,
                plan.pregrasp_pose,
                "visual_pregrasp",
                0.035,
                (0.1, 0.65, 1.0, 1.0),
            )
        )
        markers.markers.append(
            self._sphere_marker(
                frame_id,
                stamp,
                2,
                plan.grasp_pose,
                "visual_grasp",
                0.04,
                (1.0, 0.18, 0.12, 1.0),
            )
        )
        markers.markers.append(self._text_marker(frame_id, stamp, plan))
        return markers

    def _base_marker(self, frame_id: str, stamp, marker_id: int, marker_type: int, ns: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime.sec = 0
        marker.frame_locked = True
        return marker

    def _delete_all(self, frame_id: str, stamp) -> Marker:
        marker = self._base_marker(frame_id, stamp, 0, Marker.CUBE, "visual_grasp")
        marker.action = Marker.DELETEALL
        return marker

    def _object_marker(self, frame_id: str, stamp, pose: Pose, *, jaw_width: float, object_length: float) -> Marker:
        marker = self._base_marker(frame_id, stamp, 0, Marker.CYLINDER, "visual_object")
        marker.pose = copy_pose(pose)
        diameter = object_diameter(jaw_width, self._object_min_diameter_m)
        height = object_height(object_length, self._object_min_height_m)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = height
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.18
        marker.color.a = 0.82
        return marker

    def _sphere_marker(
        self,
        frame_id: str,
        stamp,
        marker_id: int,
        pose: Pose,
        ns: str,
        size: float,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = self._base_marker(frame_id, stamp, marker_id, Marker.SPHERE, ns)
        marker.pose = copy_pose(pose)
        marker.scale.x = float(size)
        marker.scale.y = float(size)
        marker.scale.z = float(size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _text_marker(self, frame_id: str, stamp, plan: GraspPlan) -> Marker:
        marker = self._base_marker(frame_id, stamp, 3, Marker.TEXT_VIEW_FACING, "visual_object_label")
        marker.pose = identity_orientation(plan.candidate.pose)
        height = object_height(float(plan.candidate.object_length), self._object_min_height_m)
        marker.pose.position.z += height * 0.5 + 0.055
        marker.scale.z = 0.055
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{plan.candidate.class_name} {float(plan.candidate.confidence):.2f}".strip()
        return marker


class VisualGraspMarkerNode(Node):
    """Publishes RViz markers for the real visual grasp plan."""

    def __init__(self) -> None:
        super().__init__("rebotarm_visual_grasp_markers")
        self.declare_parameter("input_topic", "/grasp/plan")
        self.declare_parameter("output_topic", "/grasp/visual_markers")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("object_min_diameter_m", 0.06)
        self.declare_parameter("object_min_height_m", 0.12)
        self.declare_parameter("upright_object_marker", True)
        self.declare_parameter("publish_invalid_delete", True)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._publish_invalid_delete = bool(self.get_parameter("publish_invalid_delete").value)
        self._builder = VisualGraspMarkerBuilder(
            object_min_diameter_m=float(self.get_parameter("object_min_diameter_m").value),
            object_min_height_m=float(self.get_parameter("object_min_height_m").value),
            upright_object_marker=bool(self.get_parameter("upright_object_marker").value),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(MarkerArray, self._output_topic, 10)
        self.create_subscription(GraspPlan, self._input_topic, self._on_plan, 10)
        self.get_logger().info(
            f"visual grasp markers ready: input={self._input_topic}, output={self._output_topic}, "
            f"target_frame={self._target_frame}"
        )

    def _on_plan(self, plan: GraspPlan) -> None:
        marker_stamp = rclpy.time.Time().to_msg()
        if not plan.valid and not self._publish_invalid_delete:
            return
        converted = self._convert_plan(plan)
        markers = self._builder.build(converted, frame_id=str(converted.header.frame_id), stamp=marker_stamp)
        self._publisher.publish(markers)

    def _convert_plan(self, plan: GraspPlan) -> GraspPlan:
        source_frame = str(plan.header.frame_id)
        if not self._target_frame or not source_frame or source_frame == self._target_frame:
            converted = deepcopy(plan)
            converted.header.frame_id = source_frame or self._target_frame
            return converted
        converted = deepcopy(plan)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"skip visual markers: cannot transform {source_frame} -> {self._target_frame}: {exc}"
            )
            return converted
        transform = _transform_from_msg(tf_msg)
        converted.header.frame_id = self._target_frame
        converted.candidate.header.frame_id = self._target_frame
        converted.candidate.pose = transform_pose_message(converted.candidate.pose, transform)
        converted.pregrasp_pose = transform_pose_message(converted.pregrasp_pose, transform)
        converted.grasp_pose = transform_pose_message(converted.grasp_pose, transform)
        return converted


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualGraspMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
