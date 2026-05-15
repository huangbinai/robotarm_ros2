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
