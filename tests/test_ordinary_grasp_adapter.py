from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import types

import numpy as np
import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
for path in (
    ROS2_ROOT / "src" / "rebotarm_vision",
    ROS2_ROOT / "src" / "rebotarm_msgs",
):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


def _install_ros_message_stubs_if_needed():
    if "geometry_msgs" in sys.modules:
        return
    if importlib.util.find_spec("geometry_msgs") is not None:
        return

    class Header:
        def __init__(self):
            self.frame_id = ""
            self.stamp = None

    class Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class Quaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()

    class Detection2D:
        def __init__(self):
            self.header = Header()
            self.class_name = ""
            self.confidence = 0.0
            self.center_u = 0
            self.center_v = 0
            self.x_min = 0
            self.y_min = 0
            self.x_max = 0
            self.y_max = 0
            self.has_obb = False
            self.obb_cx = 0.0
            self.obb_cy = 0.0
            self.obb_w = 0.0
            self.obb_h = 0.0
            self.obb_theta = 0.0
            self.obb_points_xy = []
            self.has_mask = False
            self.mask_polygon_xy = []

    class Detection2DArray:
        def __init__(self):
            self.header = Header()
            self.detections = []

    class GraspCandidate:
        def __init__(self):
            self.header = Header()
            self.class_name = ""
            self.confidence = 0.0
            self.pose = Pose()
            self.jaw_width = 0.0
            self.object_length = 0.0
            self.valid = False
            self.source = ""

    class GraspPlan:
        def __init__(self):
            self.header = Header()
            self.candidate = GraspCandidate()
            self.pregrasp_pose = Pose()
            self.grasp_pose = Pose()
            self.jaw_width = 0.0
            self.valid = False
            self.source = ""
            self.reason = ""

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Pose = Pose
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    rebotarm_msgs = types.ModuleType("rebotarm_msgs")
    rebotarm_msgs_msg = types.ModuleType("rebotarm_msgs.msg")
    rebotarm_msgs_msg.Detection2D = Detection2D
    rebotarm_msgs_msg.Detection2DArray = Detection2DArray
    rebotarm_msgs_msg.GraspCandidate = GraspCandidate
    rebotarm_msgs_msg.GraspPlan = GraspPlan
    sys.modules["rebotarm_msgs"] = rebotarm_msgs
    sys.modules["rebotarm_msgs.msg"] = rebotarm_msgs_msg


def _make_detection_array():
    from rebotarm_msgs.msg import Detection2D, Detection2DArray

    msg = Detection2DArray()
    msg.header.frame_id = "camera_depth_frame"

    det = Detection2D()
    det.header.frame_id = "camera_depth_frame"
    det.class_name = "bottle"
    det.confidence = 0.92
    det.x_min = 280
    det.y_min = 180
    det.x_max = 360
    det.y_max = 300
    det.center_u = 320
    det.center_v = 240
    det.has_obb = True
    det.obb_cx = 320.0
    det.obb_cy = 240.0
    det.obb_w = 40.0
    det.obb_h = 120.0
    det.obb_theta = 0.0
    det.obb_points_xy = [
        300.0,
        180.0,
        340.0,
        180.0,
        340.0,
        300.0,
        300.0,
        300.0,
    ]
    msg.detections.append(det)
    return msg


def test_ordinary_grasp_adapter_reuses_old_python_algorithm():
    _install_ros_message_stubs_if_needed()

    from rebotarm_vision.converters.ordinary_grasp_adapter import (
        CameraIntrinsics,
        plan_from_detections_and_depth,
    )

    local_repo_root = Path(__file__).resolve().parents[3]
    ordinary_grasp_root = Path("/home/u24/rebot_grasp")
    if not ordinary_grasp_root.exists():
        ordinary_grasp_root = local_repo_root / "softare" / "rebot_grasp"
    depth_mm = np.full((480, 640), 1000, dtype=np.uint16)

    plan = plan_from_detections_and_depth(
        _make_detection_array(),
        depth_mm,
        CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0),
        ordinary_grasp_root=ordinary_grasp_root,
        output_frame_id="camera_depth_frame",
        pregrasp_offset_m=0.08,
    )

    assert plan.valid is True
    assert plan.header.frame_id == "camera_depth_frame"
    assert plan.source == "ordinary_grasp"
    assert plan.candidate.class_name == "bottle"
    assert plan.candidate.source == "ordinary_grasp_obb_depth"
    assert plan.candidate.pose.position.x == 0.0
    assert plan.candidate.pose.position.y == 0.0
    assert plan.candidate.pose.position.z == 1.0
    assert plan.jaw_width > 0.07
    assert plan.grasp_pose.position.z == 1.0
    dx = plan.pregrasp_pose.position.x - plan.grasp_pose.position.x
    dy = plan.pregrasp_pose.position.y - plan.grasp_pose.position.y
    dz = plan.pregrasp_pose.position.z - plan.grasp_pose.position.z
    assert (dx * dx + dy * dy + dz * dz) ** 0.5 == pytest.approx(0.08, abs=1e-3)


def test_ordinary_grasp_adapter_marks_bbox_fallback_source():
    _install_ros_message_stubs_if_needed()

    from rebotarm_vision.converters.ordinary_grasp_adapter import (
        CameraIntrinsics,
        plan_from_detections_and_depth,
    )

    local_repo_root = Path(__file__).resolve().parents[3]
    ordinary_grasp_root = Path("/home/u24/rebot_grasp")
    if not ordinary_grasp_root.exists():
        ordinary_grasp_root = local_repo_root / "softare" / "rebot_grasp"
    detections = _make_detection_array()
    detections.detections[0].has_obb = False
    detections.detections[0].obb_points_xy = []

    plan = plan_from_detections_and_depth(
        detections,
        np.full((480, 640), 1000, dtype=np.uint16),
        CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0),
        ordinary_grasp_root=ordinary_grasp_root,
        output_frame_id="camera_depth_frame",
    )

    assert plan.valid is True
    assert plan.candidate.source == "ordinary_grasp_bbox_depth"


def test_ordinary_grasp_adapter_uses_mask_when_available_without_obb():
    _install_ros_message_stubs_if_needed()

    from rebotarm_vision.converters.ordinary_grasp_adapter import (
        CameraIntrinsics,
        plan_from_detections_and_depth,
    )

    local_repo_root = Path(__file__).resolve().parents[3]
    ordinary_grasp_root = Path("/home/u24/rebot_grasp")
    if not ordinary_grasp_root.exists():
        ordinary_grasp_root = local_repo_root / "softare" / "rebot_grasp"
    detections = _make_detection_array()
    det = detections.detections[0]
    det.has_obb = False
    det.obb_points_xy = []
    det.has_mask = True
    det.mask_polygon_xy = [
        300.0,
        180.0,
        340.0,
        180.0,
        340.0,
        300.0,
        300.0,
        300.0,
    ]

    plan = plan_from_detections_and_depth(
        detections,
        np.full((480, 640), 1000, dtype=np.uint16),
        CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0),
        ordinary_grasp_root=ordinary_grasp_root,
        output_frame_id="camera_depth_frame",
    )

    assert plan.valid is True
    assert plan.candidate.source == "ordinary_grasp_mask_depth"
