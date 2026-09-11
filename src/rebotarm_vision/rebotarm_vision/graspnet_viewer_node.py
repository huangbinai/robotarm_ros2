from __future__ import annotations

from pathlib import Path
import os
import threading
import message_filters
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rebotarm_msgs.msg import GraspCandidateArray

from .depth_utils import depth_image_to_array
from .graspnet_visualization import build_grasp_scene, Open3DGraspWindow
from .perception_frames import color_image_to_array, require_fresh, validate_frame_bundle


class GraspNetViewerNode(Node):
    def __init__(self, *, window=None):
        super().__init__("rebotarm_graspnet_viewer")
        defaults = {
            "input_color_topic": "/grasp/preview/color", "input_depth_topic": "/grasp/preview/depth",
            "input_camera_info_topic": "/grasp/preview/camera_info", "input_candidates_topic": "/grasp/graspnet_candidates",
            "top_n": 5, "max_points": 30000, "sync_queue_size": 30, "max_frame_age_sec": 1.5,
            "min_depth_m": 0.15, "max_depth_m": 1.2, "point_size": 2.0,
            "save_directory": str(Path.cwd() / ".codex_tmp/graspnet-previews"),
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.window = window or Open3DGraspWindow(save_directory=Path(str(value("save_directory"))), point_size=float(value("point_size")))
        self._max_age = float(value("max_frame_age_sec"))
        self._pending = None
        self._last_header = None
        self._empty_candidates = False
        self._frame_lock = threading.Lock()
        self._subscribers = [
            message_filters.Subscriber(self, Image, str(value("input_color_topic")), qos_profile=3),
            message_filters.Subscriber(self, Image, str(value("input_depth_topic")), qos_profile=3),
            message_filters.Subscriber(self, CameraInfo, str(value("input_camera_info_topic")), qos_profile=3),
            message_filters.Subscriber(self, GraspCandidateArray, str(value("input_candidates_topic")), qos_profile=10),
        ]
        self._subscribers[-1].registerCallback(self._observe_candidates)
        self._sync = message_filters.TimeSynchronizer(self._subscribers, max(2, int(value("sync_queue_size"))))
        self._sync.registerCallback(self._on_frame)
        self.get_logger().info("Open3D candidate preview: mouse rotate/zoom, R reset view, S save PNG+PLY. No motion interfaces.")

    def _observe_candidates(self, candidates):
        with self._frame_lock:
            self._empty_candidates = not bool(candidates.candidates)
            if self._empty_candidates:
                self._pending = None

    def _on_frame(self, color, depth, info, candidates):
        with self._frame_lock:
            self._pending = (color, depth, info, candidates)

    def render_pending(self):
        with self._frame_lock:
            pending, self._pending = self._pending, None
            empty, self._empty_candidates = self._empty_candidates, False
        if empty:
            self.window.clear()
            self._last_header = None
        if pending is not None:
            color, depth, info, candidates = pending
            try:
                camera = validate_frame_bundle(*pending, now_ns=self.get_clock().now().nanoseconds, max_age_sec=self._max_age)
                camera["workspace_min_depth_m"] = float(self.get_parameter("min_depth_m").value)
                camera["workspace_max_depth_m"] = float(self.get_parameter("max_depth_m").value)
                self.window.update(build_grasp_scene(
                    color_image_to_array(color), depth_image_to_array(depth), camera, candidates,
                    top_n=int(self.get_parameter("top_n").value), max_points=int(self.get_parameter("max_points").value),
                ))
                self._last_header = color.header
            except ValueError as exc:
                self.window.clear()
                self._last_header = None
                self.get_logger().warn(f"Open3D frame rejected: {exc}", throttle_duration_sec=5.0)
        elif self._last_header is not None:
            try:
                require_fresh(self._last_header, self.get_clock().now().nanoseconds, self._max_age)
            except ValueError:
                self.window.clear()
                self._last_header = None
        return self.window.poll()

    def destroy_node(self):
        self.window.close()
        return super().destroy_node()


def main(args=None):
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("Open3D viewer requires an Ubuntu desktop DISPLAY")
    rclpy.init(args=args)
    node = None
    executor = None
    thread = None
    try:
        node = GraspNetViewerNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        while rclpy.ok():
            if not node.render_pending(): break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None: executor.shutdown()
        if thread is not None: thread.join(timeout=2.0)
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
