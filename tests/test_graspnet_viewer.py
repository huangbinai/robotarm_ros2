"""Candidate geometry and ROS viewer synchronization; no display or hardware."""
from copy import deepcopy
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from rebotarm_msgs.msg import GraspCandidateArray
from rebotarm_vision.graspnet_baseline_adapter import predictions_to_candidate_array
from rebotarm_vision.graspnet_visualization import build_grasp_scene, candidate_transform, scene_geometries
from rebotarm_vision.graspnet_viewer_node import GraspNetViewerNode
from test_local_ros_perception import bundle, prediction
from test_architecture_migration import composed_nodes


def frame():
    color, depth, info, _ = bundle()
    candidates = predictions_to_candidate_array([prediction()],frame_id=color.header.frame_id,class_name="cup",max_candidates=1)
    candidates.header = deepcopy(color.header)
    candidates.candidates[0].header = deepcopy(color.header)
    return color, depth, info, candidates


def test_scene_point_cloud_uses_intrinsics_color_and_bounded_point_count():
    candidates = frame()[-1]
    camera = dict(fx=100.,fy=100.,cx=2.,cy=2.)
    color = np.zeros((4,4,3),dtype=np.uint8)
    color[:,:,2] = 255
    depth = np.full((4,4),500,dtype=np.uint16)
    scene = build_grasp_scene(color,depth,camera,candidates,max_points=4,top_n=1)
    assert scene.points.shape == (4,3)
    np.testing.assert_allclose(scene.points[:,2],0.5)
    np.testing.assert_allclose(scene.colors,[[1.,0.,0.]]*4)
    assert len(scene.grasps) == 1
    np.testing.assert_allclose(scene.grasps[0][0][:3,3],[0.,0.,0.5])


@pytest.mark.parametrize("fault", ["invalid", "frame", "quaternion", "nan"])
def test_bad_candidate_pose_is_rejected(fault):
    candidate = frame()[-1].candidates[0]
    if fault == "invalid": candidate.valid = False
    if fault == "frame": candidate.header.frame_id = "base_link"
    if fault == "quaternion": candidate.pose.orientation.w = 0.
    if fault == "nan": candidate.pose.position.z = float("nan")
    with pytest.raises(ValueError): candidate_transform(candidate,"camera_depth_frame")


def test_empty_candidates_show_cloud_without_fake_grippers():
    candidates = frame()[-1]
    candidates.candidates = []
    scene = build_grasp_scene(np.zeros((4,4,3),dtype=np.uint8),np.full((4,4),500,dtype=np.uint16),dict(fx=100.,fy=100.,cx=2.,cy=2.),candidates)
    assert not scene.grasps
    assert len(scene_geometries(scene)) == 1


class Window:
    def __init__(self): self.scenes=[]; self.cleared=0
    def update(self,scene): self.scenes.append(scene)
    def clear(self): self.cleared += 1
    def poll(self): return True
    def close(self): pass


@pytest.fixture
def viewer():
    rclpy.init(args=[])
    window = Window()
    node = GraspNetViewerNode(window=window)
    node.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_100_000_000))
    yield node,window
    node.destroy_node()
    rclpy.shutdown()


def test_viewer_pairs_candidates_with_their_source_rgbd(viewer):
    node,window = viewer
    messages = frame()
    wrong_depth = deepcopy(messages[1]); wrong_depth.header.stamp.sec = 11
    for index,message in enumerate(messages):
        node._subscribers[index].signalMessage(wrong_depth if index==1 else message)
    node.render_pending()
    assert not window.scenes
    node._subscribers[1].signalMessage(messages[1])
    node.render_pending()
    assert len(window.scenes) == 1
    assert len(window.scenes[0].grasps) == 1


def test_stale_view_is_cleared(viewer):
    node,window = viewer
    node._on_frame(*frame()); node.render_pending()
    node.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=12_000_000_000))
    node.render_pending()
    assert window.cleared == 1


def test_new_empty_result_cannot_redisplay_pending_old_candidates(viewer):
    node,window = viewer
    node._on_frame(*frame())
    node._observe_candidates(GraspCandidateArray())
    node.render_pending()
    assert window.cleared == 1
    assert not window.scenes


def test_viewer_is_read_only_and_windows_entrypoints_are_removed():
    import ast
    root=Path(__file__).resolve().parents[1]
    path=root/'src/rebotarm_vision/rebotarm_vision/graspnet_viewer_node.py'
    calls=[getattr(n.func,'attr','') for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.Call)]
    assert not {'create_client','create_service','create_publisher','send_goal_async'} & set(calls)
    assert not list((root/'tools').glob('windows_*'))
    assert not (root/'tools/graspnet_baseline_inference.py').exists()


def test_open3d_is_opt_in_and_does_not_start_hardware():
    viewer = ("rebotarm_vision", "rebotarm_graspnet_viewer")
    assert viewer not in composed_nodes("visual_grasp_perception_preview", use_local_rviz="false")
    nodes = composed_nodes("visual_grasp_perception_preview", show_open3d="true", use_local_rviz="false")
    assert nodes.count(viewer) == 1
    assert ("rebotarmcontroller", "reBotArmController") not in nodes
