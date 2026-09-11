from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import message_filters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from rebotarm_msgs.msg import Detection2DArray, GraspCandidateArray

from .depth_utils import depth_image_to_array
from .graspnet_baseline_adapter import GraspNetBaselineBackend, predictions_to_candidate_array
from .model_paths import default_workspace_path
from .perception_frames import (
    color_image_to_array, detection_payload, require_fresh, stamp_ns, validate_frame_bundle,
)


class GraspNetBaselineNode(Node):
    """Infer from exactly matched ROS RGB-D/CameraInfo/detection messages."""

    def __init__(self, *, backend=None) -> None:
        super().__init__("rebotarm_graspnet_baseline_node")
        for name, default in {
            "input_color_topic": "/camera/color/image_raw", "input_depth_topic": "/camera/depth/image_raw",
            "input_camera_info_topic": "/camera/depth/camera_info", "input_detections_topic": "/grasp/detections",
            "output_candidates_topic": "/grasp/graspnet_candidates", "output_frame_id": "camera_depth_frame",
            "source_mode": "ros", "model_root": "", "checkpoint_path": "", "device": "cuda:0",
            "backend_module": "rebotarm_vision.graspnet_inference", "max_grasps": 5, "max_points": 20000,
            "sync_queue_size": 30, "inference_rate_hz": 2.0, "max_frame_age_sec": 1.5,
            "min_depth_m": 0.15, "max_depth_m": 1.20,
            "preview_color_topic": "/grasp/preview/color",
            "preview_depth_topic": "/grasp/preview/depth",
            "preview_camera_info_topic": "/grasp/preview/camera_info",
        }.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        if value("source_mode") not in ("ros", "local_backend"):
            raise ValueError("HTTP GraspNet input is retired; source_mode must be ros")
        self.output_frame_id = str(value("output_frame_id"))
        self._max_age = float(value("max_frame_age_sec"))
        self._pending = None
        self._last_header = None
        self._last_processed_stamp = -1
        self.backend = backend or self._create_backend()
        self.candidates_pub = self.create_publisher(GraspCandidateArray, str(value("output_candidates_topic")), 10)
        # Republish only the actual inference input, at candidate rate. Reliable
        # delivery avoids trying to recover a large RGB-D frame from lossy raw
        # streams after inference has completed.
        self._preview_publishers = [
            self.create_publisher(Image, str(value("preview_color_topic")), 3),
            self.create_publisher(Image, str(value("preview_depth_topic")), 3),
            self.create_publisher(CameraInfo, str(value("preview_camera_info_topic")), 3),
        ]
        self._subscribers = [
            message_filters.Subscriber(self, Image, str(value("input_color_topic")), qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(self, Image, str(value("input_depth_topic")), qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(self, CameraInfo, str(value("input_camera_info_topic")), qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(self, Detection2DArray, str(value("input_detections_topic")), qos_profile=10),
        ]
        self._sync = message_filters.TimeSynchronizer(self._subscribers, max(2, int(value("sync_queue_size"))))
        self._sync.registerCallback(self._on_frame)
        self.create_timer(1.0 / max(float(value("inference_rate_hz")), 0.1), self._on_timer)
        self.get_logger().info("Local GraspNet: full-scene inference, YOLO projection filtering, ROS messages only")

    def _create_backend(self):
        model_root = str(self.get_parameter("model_root").value).strip() or default_workspace_path("third_party/graspnet-baseline")
        checkpoint = str(self.get_parameter("checkpoint_path").value).strip() or default_workspace_path("models/graspnet/checkpoint-rs.tar")
        if not model_root or not Path(model_root).expanduser().is_dir():
            raise FileNotFoundError("Local GraspNet repository missing; configure graspnet_model_root")
        if not checkpoint or not Path(checkpoint).expanduser().is_file():
            raise FileNotFoundError("Local GraspNet weights missing; configure graspnet_checkpoint_path")
        return GraspNetBaselineBackend(
            model_root=str(Path(model_root).expanduser()), checkpoint_path=str(Path(checkpoint).expanduser()),
            device=str(self.get_parameter("device").value),
            module_name=str(self.get_parameter("backend_module").value),
            num_point=int(self.get_parameter("max_points").value),
        )

    def _on_frame(self, color, depth, info, detections):
        self._pending = (color, depth, info, detections)
        self._last_header = deepcopy(color.header)
        if not detections.detections:
            self._pending = None
            self._publish_empty(color.header)

    def _on_timer(self):
        pending, self._pending = self._pending, None
        if pending is None:
            if self._last_header is not None:
                try:
                    require_fresh(self._last_header, self.get_clock().now().nanoseconds, self._max_age)
                except ValueError:
                    self._publish_empty(self._last_header)
            return
        color, depth, info, detections = pending
        try:
            camera_info = validate_frame_bundle(
                *pending, now_ns=self.get_clock().now().nanoseconds, max_age_sec=self._max_age,
            )
            if color.header.frame_id != self.output_frame_id:
                raise ValueError("output_frame_id must match the calibrated RGB-D optical frame")
            source_stamp = stamp_ns(color.header.stamp)
            if source_stamp <= self._last_processed_stamp:
                return
            self._last_processed_stamp = source_stamp
            payloads = [detection_payload(detection) for detection in detections.detections]
            if not payloads:
                self._publish_empty(color.header)
                return
            target = max(payloads, key=lambda item: item["confidence"])
            camera_info["workspace_min_depth_m"] = float(self.get_parameter("min_depth_m").value)
            camera_info["workspace_max_depth_m"] = float(self.get_parameter("max_depth_m").value)
            predictions = self.backend.infer(
                color_bgr=color_image_to_array(color), depth_mm=depth_image_to_array(depth),
                detections=payloads, camera_info=camera_info,
                max_grasps=int(self.get_parameter("max_grasps").value),
            )
            require_fresh(color.header, self.get_clock().now().nanoseconds, self._max_age)
            candidates = predictions_to_candidate_array(
                predictions, frame_id=self.output_frame_id, class_name=target["class_name"],
                max_candidates=int(self.get_parameter("max_grasps").value),
            )
            candidates.header = deepcopy(color.header)
            for candidate in candidates.candidates:
                candidate.header = deepcopy(color.header)
            for publisher, message in zip(self._preview_publishers, (color, depth, info)):
                publisher.publish(message)
            self.candidates_pub.publish(candidates)
        except Exception as exc:
            self.get_logger().warn(f"GraspNet frame rejected: {exc}", throttle_duration_sec=5.0)
            self._publish_empty(color.header)

    def _publish_empty(self, header=None):
        msg = GraspCandidateArray()
        msg.header.frame_id = self.output_frame_id
        if header is not None:
            msg.header = deepcopy(header)
        msg.best_index = -1
        self.candidates_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GraspNetBaselineNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
