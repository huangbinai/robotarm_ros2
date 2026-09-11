from __future__ import annotations

from copy import deepcopy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rebotarm_msgs.msg import Detection2DArray

from .converters.detection_msgs import result_to_detection_array_msg
from .converters.image_msgs import color_to_msg
from .detector.yolo_detector import YoloDetector
from .model_paths import yolo_model_path
from .perception_frames import color_image_to_array, require_fresh, stamp_ns
from .utils.visualization import draw_detections


class YoloDetectionNode(Node):
    def __init__(self, *, detector=None) -> None:
        super().__init__("rebotarm_yolo_node")
        for name, default in {
            "input_color_topic": "/camera/color/image_raw", "output_detections_topic": "/grasp/detections",
            "yolo.model_path": "", "yolo.device": "cuda:0", "yolo.conf_threshold": 0.25,
            "yolo.iou_threshold": 0.45, "publish_annotated": True, "inference_rate_hz": 5.0,
            "max_frame_age_sec": 1.5,
        }.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.detector = detector or YoloDetector(
            model_path=str(yolo_model_path(str(value("yolo.model_path")))), device=str(value("yolo.device")),
            conf_threshold=float(value("yolo.conf_threshold")), iou_threshold=float(value("yolo.iou_threshold")),
            use_world=False, custom_classes=[],
        )
        self._max_age = float(value("max_frame_age_sec"))
        self._latest = None
        self._last_processed_stamp = -1
        self.detection_pub = self.create_publisher(Detection2DArray, str(value("output_detections_topic")), 10)
        self.annotated_pub = self.create_publisher(Image, "/camera/color/annotated", qos_profile_sensor_data) if value("publish_annotated") else None
        self.create_subscription(Image, str(value("input_color_topic")), self._on_color, qos_profile_sensor_data)
        self.create_timer(1.0 / max(float(value("inference_rate_hz")), 0.1), self._on_timer)

    def _on_color(self, msg):
        self._latest = msg

    def _empty(self, header):
        empty = Detection2DArray()
        empty.header = deepcopy(header)
        self.detection_pub.publish(empty)

    def _on_timer(self):
        msg = self._latest
        if msg is None:
            return
        try:
            require_fresh(msg.header, self.get_clock().now().nanoseconds, self._max_age)
            if stamp_ns(msg.header.stamp) <= self._last_processed_stamp:
                return
            self._last_processed_stamp = stamp_ns(msg.header.stamp)
            image = color_image_to_array(msg)
            results = self.detector.infer(image)
            detections = result_to_detection_array_msg(results, msg.header.stamp, msg.header.frame_id)
            require_fresh(msg.header, self.get_clock().now().nanoseconds, self._max_age)
            self.detection_pub.publish(detections)
            if self.annotated_pub is not None:
                self.annotated_pub.publish(color_to_msg(draw_detections(image, detections), msg.header.stamp, msg.header.frame_id))
        except Exception as exc:
            self.get_logger().warn(f"YOLO frame rejected: {exc}", throttle_duration_sec=5.0)
            self._empty(msg.header)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
