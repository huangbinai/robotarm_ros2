from __future__ import annotations

from geometry_msgs.msg import Pose


def build_ee_target_marker(
    *,
    interactive_marker_cls,
    control_cls,
    marker_cls,
    frame_id: str,
    marker_name: str,
    marker_scale: float,
    pose: Pose,
):
    marker = interactive_marker_cls()
    marker.header.frame_id = frame_id
    marker.name = marker_name
    marker.description = "reBotArm EE Target"
    marker.scale = marker_scale
    marker.pose = pose

    visual_control = control_cls()
    visual_control.always_visible = True
    visual_control.interaction_mode = control_cls.NONE
    visual_control.markers.extend(
        _make_ee_visible_markers(marker_cls=marker_cls, marker_scale=marker_scale)
    )
    marker.controls.append(visual_control)

    for name, orientation, mode in (
        ("move_plane", (1.0, 0.0, 1.0, 0.0), control_cls.MOVE_PLANE),
        ("move_x", (1.0, 1.0, 0.0, 0.0), control_cls.MOVE_AXIS),
        ("move_z", (1.0, 0.0, 1.0, 0.0), control_cls.MOVE_AXIS),
        ("rotate_z", (1.0, 0.0, 1.0, 0.0), control_cls.ROTATE_AXIS),
    ):
        control = control_cls()
        control.name = name
        control.orientation.w = orientation[0]
        control.orientation.x = orientation[1]
        control.orientation.y = orientation[2]
        control.orientation.z = orientation[3]
        control.interaction_mode = mode
        marker.controls.append(control)

    return marker


def build_smoke_marker(
    *,
    interactive_marker_cls,
    control_cls,
    marker_cls,
    frame_id: str,
    marker_scale: float,
    pose: Pose,
):
    marker = interactive_marker_cls()
    marker.header.frame_id = frame_id
    marker.name = "smoke_marker"
    marker.description = "Smoke Marker"
    marker.scale = marker_scale
    marker.pose = pose

    move_plane = control_cls()
    move_plane.name = "move_plane"
    move_plane.orientation.w = 1.0
    move_plane.orientation.x = 0.0
    move_plane.orientation.y = 1.0
    move_plane.orientation.z = 0.0
    move_plane.interaction_mode = control_cls.MOVE_PLANE
    marker.controls.append(move_plane)

    move_axis = control_cls()
    move_axis.name = "move_x"
    move_axis.orientation.w = 1.0
    move_axis.orientation.x = 1.0
    move_axis.orientation.y = 0.0
    move_axis.orientation.z = 0.0
    move_axis.interaction_mode = control_cls.MOVE_AXIS
    marker.controls.append(move_axis)

    rotate_axis = control_cls()
    rotate_axis.name = "rotate_z"
    rotate_axis.orientation.w = 1.0
    rotate_axis.orientation.x = 0.0
    rotate_axis.orientation.y = 1.0
    rotate_axis.orientation.z = 0.0
    rotate_axis.interaction_mode = control_cls.ROTATE_AXIS
    marker.controls.append(rotate_axis)

    visual_control = control_cls()
    visual_control.always_visible = True
    visual_control.interaction_mode = control_cls.NONE
    visual_control.markers.append(
        _make_smoke_visible_marker(marker_cls=marker_cls, marker_scale=marker_scale)
    )
    marker.controls.append(visual_control)

    return marker


def _make_ee_visible_markers(*, marker_cls, marker_scale: float):
    markers = []

    center = marker_cls()
    center.type = marker_cls.SPHERE
    center.scale.x = marker_scale * 0.7
    center.scale.y = marker_scale * 0.7
    center.scale.z = marker_scale * 0.7
    center.color.r = 1.0
    center.color.g = 0.85
    center.color.b = 0.10
    center.color.a = 1.0
    markers.append(center)

    label = marker_cls()
    label.type = marker_cls.TEXT_VIEW_FACING
    label.text = "EE Target"
    label.scale.z = marker_scale * 0.35
    label.pose.position.z = marker_scale * 0.7
    label.color.r = 1.0
    label.color.g = 1.0
    label.color.b = 1.0
    label.color.a = 1.0
    markers.append(label)

    axis_length = marker_scale * 1.1
    axis_shaft = marker_scale * 0.15
    axis_head = marker_scale * 0.24
    axis_offset = axis_length * 0.5

    for axis_name, rgba, position, orientation in (
        (
            "x",
            (0.95, 0.25, 0.25, 0.95),
            (axis_offset, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        (
            "y",
            (0.20, 0.85, 0.25, 0.95),
            (0.0, axis_offset, 0.0),
            (0.0, 0.0, 0.70710678, 0.70710678),
        ),
        (
            "z",
            (0.20, 0.45, 0.95, 0.95),
            (0.0, 0.0, axis_offset),
            (0.0, 0.70710678, 0.0, 0.70710678),
        ),
    ):
        axis = marker_cls()
        axis.ns = "ee_target_axes"
        axis.id = ord(axis_name)
        axis.type = marker_cls.ARROW
        axis.scale.x = axis_length
        axis.scale.y = axis_shaft
        axis.scale.z = axis_head
        axis.pose.position.x = position[0]
        axis.pose.position.y = position[1]
        axis.pose.position.z = position[2]
        axis.pose.orientation.x = orientation[0]
        axis.pose.orientation.y = orientation[1]
        axis.pose.orientation.z = orientation[2]
        axis.pose.orientation.w = orientation[3]
        axis.color.r = rgba[0]
        axis.color.g = rgba[1]
        axis.color.b = rgba[2]
        axis.color.a = rgba[3]
        markers.append(axis)

    return markers


def _make_smoke_visible_marker(*, marker_cls, marker_scale: float):
    sphere = marker_cls()
    sphere.type = marker_cls.SPHERE
    sphere.scale.x = marker_scale * 0.45
    sphere.scale.y = marker_scale * 0.45
    sphere.scale.z = marker_scale * 0.45
    sphere.color.r = 1.0
    sphere.color.g = 0.2
    sphere.color.b = 0.2
    sphere.color.a = 1.0
    return sphere
