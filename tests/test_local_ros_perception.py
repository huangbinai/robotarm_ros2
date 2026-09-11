"""Ubuntu perception contracts; synthetic frames/model outputs, no camera/GPU."""
from copy import deepcopy
import importlib
from pathlib import Path
from types import SimpleNamespace
import sys
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image
from builtin_interfaces.msg import Time
from rebotarm_msgs.msg import Detection2D, Detection2DArray, GraspCandidateArray

from rebotarm_vision.camera.gemini2_driver import Gemini2Config, Gemini2Driver, scaled_depth_mm
from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg
from rebotarm_vision.converters.image_msgs import camera_info_to_msg, color_to_msg, depth_to_msg
from rebotarm_vision.depth_utils import depth_image_to_array
from rebotarm_vision.graspnet_baseline_adapter import GraspNetBaselineBackend, GraspNetPrediction
from rebotarm_vision.graspnet_baseline_node import GraspNetBaselineNode
from rebotarm_vision.graspnet_inference import build_scene_cloud, graspnet_array_to_candidates
from rebotarm_vision.perception_frames import (
    color_image_to_array, detection_payload, require_fresh, stamp_ns, validate_frame_bundle,
)
from rebotarm_vision.vision_node import RebotArmVisionNode
from rebotarm_vision.yolo_node import YoloDetectionNode
from rebotarm_vision.candidate_ik_filter_node import CandidateIkFilterNode
from rebotarm_vision.tcp_calibration_node import AutoArucoReferenceProvider
from test_architecture_migration import composed_nodes


ROOT = Path(__file__).resolve().parents[1]
FRAME = "camera_depth_frame"


def bundle(seconds=10):
    stamp = Time(sec=seconds)
    color = color_to_msg(np.full((4, 4, 3), 100, dtype=np.uint8), stamp, FRAME)
    depth = depth_to_msg(np.full((4, 4), 500, dtype=np.uint16), stamp, FRAME)
    info = camera_info_to_msg(dict(width=4, height=4, fx=100., fy=100., cx=2., cy=2.), stamp, FRAME)
    detection = Detection2D()
    detection.header = deepcopy(color.header)
    detection.class_name = "cup"
    detection.confidence = 0.9
    detection.x_max = detection.y_max = 4
    detections = Detection2DArray()
    detections.header = deepcopy(color.header)
    detections.detections = [detection]
    return color, depth, info, detections


def prediction():
    return GraspNetPrediction(
        score=0.9, translation_xyz=(0., 0., 0.5),
        rotation_matrix=((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)), width_m=0.05,
    )


def test_matching_frame_uses_live_camera_info():
    messages = bundle()
    payload = validate_frame_bundle(*messages, now_ns=10_100_000_000, max_age_sec=1.5)
    assert payload["fx"] == 100.
    assert payload["depth_scale_m"] == 0.001


@pytest.mark.parametrize("bad", ["stamp", "frame", "size", "uncalibrated", "distorted", "stale", "future"])
def test_mismatched_or_unusable_bundle_is_rejected(bad):
    color, depth, info, detections = bundle()
    now = 10_100_000_000
    if bad == "stamp": depth.header.stamp.sec = 9
    if bad == "frame": detections.header.frame_id = "camera_color_frame"
    if bad == "size": info.width = 1280
    if bad == "uncalibrated": info.k[0] = 0.
    if bad == "distorted": info.d = [0.1]
    if bad == "stale": now = 12_000_000_000
    if bad == "future": now = 9_000_000_000
    with pytest.raises(ValueError):
        validate_frame_bundle(color, depth, info, detections, now_ns=now, max_age_sec=1.5)


def test_ros_image_stride_endianness_and_color_order():
    color = Image(height=1, width=1, encoding="rgb8", step=4, data=bytes([1, 2, 3, 99]))
    assert color_image_to_array(color).tolist() == [[[3, 2, 1]]]
    depth = Image(height=1, width=1, encoding="16UC1", step=4, is_bigendian=1, data=bytes([1, 244, 0, 0]))
    assert depth_image_to_array(depth).tolist() == [[500]]
    depth.data = bytes([0])
    with pytest.raises(ValueError): depth_image_to_array(depth)


def test_device_depth_scale_is_applied_in_mm():
    frame = SimpleNamespace(
        get_depth_scale=lambda: 0.1, get_height=lambda: 1, get_width=lambda: 2,
        get_data=lambda: np.array([5000, 0], dtype=np.uint16).tobytes(),
    )
    assert scaled_depth_mm(frame).tolist() == [[500, 0]]


def test_partial_camera_frames_never_reuse_previous_images(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyorbbecsdk", SimpleNamespace(OBFormat=SimpleNamespace(RGB=1, MJPG=2, BGR=3)))
    driver = Gemini2Driver(Gemini2Config(4,4,30,True,4,4,30,100,False))
    color = SimpleNamespace(get_timestamp=lambda: 10., get_width=lambda: 4, get_height=lambda: 4,
                            get_format=lambda: 1, get_data=lambda: np.zeros((4,4,3),dtype=np.uint8).tobytes())
    depth = SimpleNamespace(get_timestamp=lambda: 10., get_width=lambda: 4, get_height=lambda: 4,
                            get_depth_scale=lambda: 1., get_data=lambda: np.full((4,4),500,dtype=np.uint16).tobytes())
    frames = [SimpleNamespace(get_color_frame=lambda: color, get_depth_frame=lambda: depth),
              SimpleNamespace(get_color_frame=lambda: color, get_depth_frame=lambda: None)]
    driver._pipeline = SimpleNamespace(wait_for_frames=lambda timeout: frames.pop(0))
    assert driver.get_frame()[1] is not None
    assert driver.get_frame() == (None, None)


def test_rectification_uses_depth_profile_intrinsics():
    driver = Gemini2Driver(Gemini2Config(4,4,30,True,4,4,30,100,True))
    intrinsic = SimpleNamespace(width=4,height=4,fx=100.,fy=100.,cx=2.,cy=2.)
    distortion = SimpleNamespace(**{name: 0. for name in ("k1","k2","k3","k4","k5","k6","p1","p2")})
    profile = SimpleNamespace(get_intrinsic=lambda:intrinsic,get_distortion=lambda:distortion)
    frame = SimpleNamespace(get_stream_profile=lambda: SimpleNamespace(as_video_stream_profile=lambda:profile))
    color = np.zeros((4,4,3),dtype=np.uint8)
    depth = np.full((4,4),500,dtype=np.uint16)
    registered_color, registered_depth = driver._rectify(color,depth,frame)
    assert np.array_equal(registered_depth, depth)
    assert driver.get_camera_info()["fx"] == 100.
    with pytest.raises(ValueError): driver._rectify(color[:2],depth,frame)


def detector_results():
    box = SimpleNamespace(xyxy=np.array([[0., 0., 4., 4.]]), cls=np.array([0]), conf=np.array([0.9]))
    return [SimpleNamespace(names={0: "cup"}, boxes=[box], masks=SimpleNamespace(xy=[np.array([[0.,0.],[4.,0.],[4.,4.],[0.,4.]])]))]


def test_yolo_segmentation_and_source_header_survive_conversion():
    detections = result_to_detection_array_msg(detector_results(), Time(sec=10), FRAME)
    assert detections.header.stamp.sec == 10
    assert detections.detections[0].has_mask
    assert detection_payload(detections.detections[0])["mask_polygon_xy"] == [[0.,0.],[4.,0.],[4.,4.],[0.,4.]]


def test_scene_inference_keeps_background_and_filters_predictions_afterwards():
    color, depth, info, _ = bundle()
    camera = dict(fx=100., fy=100., cx=2., cy=2.)
    points, colors = build_scene_cloud(color_bgr=color_image_to_array(color), depth_mm=depth_image_to_array(depth), camera_info=camera)
    assert len(points) == 16  # Full scene, not the detection ROI.
    rows = np.zeros((2, 17), dtype=np.float32)
    rows[:, 0] = [0.9, 0.8]
    rows[:, 1] = 0.05
    rows[:, 4:13] = np.eye(3).reshape(1, 9)
    rows[:, 13:16] = [[0., 0., 0.5], [1., 1., 0.5]]
    predictions = graspnet_array_to_candidates(
        rows, class_name="cup", max_grasps=10,
        target_detection={"bbox_xyxy": [1, 1, 3, 3]}, camera_info=camera,
    )
    assert len(predictions) == 1


def test_packaged_backend_uses_the_same_rgbd_api_as_the_runner(monkeypatch, tmp_path):
    calls = []
    class Runner:
        def __init__(self, **kwargs): calls.append(kwargs)
        def infer(self, **kwargs):
            calls.append(kwargs)
            return [prediction()]
    module = SimpleNamespace(GraspNetBaselineInference=Runner)
    monkeypatch.setitem(sys.modules, "test_graspnet_runner", module)
    original_path = list(sys.path)
    try:
        backend = GraspNetBaselineBackend(model_root=str(tmp_path), module_name="test_graspnet_runner")
        backend.infer(color_bgr="color", depth_mm="depth", detections=["detection"], camera_info={"fx": 100.}, max_grasps=10)
    finally:
        sys.path[:] = original_path
    assert calls[-1]["color_bgr"] == "color"
    assert calls[-1]["detections"] == ["detection"]
    assert "points" not in calls[-1]


class Camera:
    last_debug_message = "synthetic"
    enabled = True
    def open(self): pass
    def close(self): pass
    def warmup(self, frames): return True
    def get_frame(self):
        if not self.enabled: return None, None
        return np.full((4,4,3), 100, dtype=np.uint8), np.full((4,4), 500, dtype=np.uint16)
    def get_camera_info(self):
        return dict(width=4,height=4,fx=100.,fy=100.,cx=2.,cy=2.,d=[0.] * 5)


class Detector:
    def infer(self, image): return detector_results()


class Backend:
    def __init__(self): self.calls = []
    def infer(self, **kwargs):
        self.calls.append(kwargs)
        return [prediction()]


@pytest.fixture
def ros_context():
    rclpy.init(args=[])
    yield
    if rclpy.ok(): rclpy.shutdown()


def test_real_ros_camera_yolo_graspnet_messages_and_disconnect(ros_context):
    camera, backend = Camera(), Backend()
    nodes = [RebotArmVisionNode(camera=camera), YoloDetectionNode(detector=Detector()), GraspNetBaselineNode(backend=backend)]
    probe = rclpy.create_node("local_perception_test_probe")
    nodes.append(probe)
    received = []
    probe.create_subscription(GraspCandidateArray, "/grasp/graspnet_candidates", received.append, 10)
    executor = SingleThreadedExecutor()
    for node in nodes: executor.add_node(node)
    try:
        deadline = time.monotonic() + 5.
        while not any(message.candidates for message in received) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        nonempty = [message for message in received if message.candidates]
        assert nonempty, "ROS-only synthetic pipeline did not produce candidates"
        message = nonempty[-1]
        assert stamp_ns(message.header.stamp) > 0
        assert message.header.frame_id == FRAME
        assert message.candidates[0].header == message.header
        assert backend.calls[-1]["camera_info"]["fx"] == 100.
        assert backend.calls[-1]["detections"][0]["mask_polygon_xy"]
        camera.enabled = False
        received.clear()
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received and not received[-1].candidates
        assert received[-1].best_index == -1
    finally:
        executor.shutdown()
        for node in reversed(nodes): node.destroy_node()


def test_camera_does_not_publish_fabricated_intrinsics(ros_context):
    camera = Camera()
    camera.get_camera_info = lambda: None
    node = RebotArmVisionNode(camera=camera)
    published = []
    try:
        node.color_pub = node.depth_pub = node.depth_camera_info_pub = node.color_camera_info_pub = SimpleNamespace(publish=published.append)
        node._on_timer()
        assert not published
    finally:
        node.destroy_node()


def test_synchronizer_does_not_pair_different_frame_times(ros_context):
    backend = Backend()
    node = GraspNetBaselineNode(backend=backend)
    try:
        first, second = bundle(10), bundle(11)
        for index, message in enumerate(first):
            node._subscribers[index].signalMessage(second[index] if index == 1 else message)
        assert node._pending is None
        node._subscribers[1].signalMessage(first[1])
        assert node._pending is not None
        assert {stamp_ns(msg.header.stamp) for msg in node._pending} == {10_000_000_000}
    finally:
        node.destroy_node()


def test_graspnet_discards_result_that_expires_during_inference(ros_context):
    backend = Backend()
    node = GraspNetBaselineNode(backend=backend)
    published = []
    now = [10_100_000_000]
    try:
        node.candidates_pub = SimpleNamespace(publish=published.append)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0]))
        def slow_infer(**kwargs):
            now[0] = 12_000_000_000
            return [prediction()]
        backend.infer = slow_infer
        node._on_frame(*bundle())
        node._on_timer()
        assert published and not published[-1].candidates
    finally:
        node.destroy_node()


def test_invalid_or_empty_input_clears_candidate_output(ros_context):
    backend = Backend()
    node = GraspNetBaselineNode(backend=backend)
    published = []
    try:
        node.candidates_pub = SimpleNamespace(publish=published.append)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
        messages = bundle()
        messages[3].detections = []
        node._on_frame(*messages)
        assert not published[-1].candidates and not backend.calls
        messages = bundle()
        messages[2].k[0] = 0.
        node._on_frame(*messages)
        node._on_timer()
        assert not published[-1].candidates and not backend.calls
    finally:
        node.destroy_node()


def test_candidate_transform_uses_capture_time_and_plan_header_is_independent():
    looked_up = []
    probe = SimpleNamespace(
        _capture_stamp=Time(sec=10),
        _tf_buffer=SimpleNamespace(lookup_transform=lambda *args, **kwargs: looked_up.append(args)),
    )
    CandidateIkFilterNode._lookup_transform(probe,"base_link",FRAME)
    assert looked_up[0][2].nanoseconds == 10_000_000_000
    from rebotarm_vision.graspnet_baseline_adapter import predictions_to_candidate_array
    from rebotarm_vision.visual_grasp_sequence import PoseTarget
    candidates = predictions_to_candidate_array([prediction()],frame_id=FRAME,class_name="cup",max_candidates=1)
    candidates.header.stamp.sec = 10
    target = PoseTarget(position=(0.3,0.,0.1),orientation=(0.,0.,0.,1.))
    plan = CandidateIkFilterNode._plan_from_filtered(SimpleNamespace(_target_frame="base_link"),candidates,[(target,target)])
    assert plan.header.frame_id == "base_link"
    assert candidates.header.frame_id == FRAME
    assert plan.header.stamp.sec == 10


def test_aruco_uses_ros_image_and_matching_camera_tf():
    color, depth, info, _ = bundle()
    lookups = []
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.,y=0.,z=0.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.),
    ))
    provider = AutoArucoReferenceProvider(
        frame_supplier=lambda:(color,info),
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args,**kwargs:lookups.append(args) or transform),
        base_frame="base_link",camera_frame=FRAME,lookup_timeout_sec=0.1,
        detector=lambda image,camera_info:(0.,0.,0.5),
    )
    assert provider.reference_position() == (0.,0.,0.5)
    assert lookups[0][2].nanoseconds == 10_000_000_000


@pytest.mark.parametrize("scene", ["visual_grasp_perception_preview", "real_perception_sim_execution"])
def test_native_launch_composes_three_perception_nodes_without_hardware(scene):
    nodes = composed_nodes(scene, use_local_rviz="false")
    for executable in ("rebotarm_vision_node", "rebotarm_yolo_node", "rebotarm_graspnet_baseline_node"):
        assert nodes.count(("rebotarm_vision", executable)) == 1
    assert ("rebotarmcontroller", "reBotArmController") not in nodes


def test_no_active_perception_entry_uses_http():
    paths = [ROOT / "src/rebotarm_vision/config/camera.yaml", ROOT / "src/rebotarm_vision/config/graspnet_policy.yaml"]
    paths += list((ROOT / "src/rebotarm_bringup/launch").glob("*visual*.launch.py"))
    paths += [ROOT / "src/rebotarm_bringup/launch/real_perception_sim_execution.launch.py"]
    for path in paths:
        text = path.read_text()
        assert "http://" not in text, path
        assert "graspnet_source_mode\": \"network" not in text, path
        assert "network_candidates_url" not in text, path
    for module_name in ("vision_node", "yolo_node", "graspnet_baseline_node"):
        module = importlib.import_module("rebotarm_vision." + module_name)
        assert "network_graspnet_client" not in Path(module.__file__).read_text()
        assert "network_mjpeg_driver" not in Path(module.__file__).read_text()
