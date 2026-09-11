from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .camera.gemini2_driver import Gemini2Config, Gemini2Driver
from .converters.image_msgs import camera_info_to_msg, color_to_msg, depth_to_msg
from .perception_frames import camera_info_payload


class RebotArmVisionNode(Node):
    """Local camera producer. YOLO runs in a separate ROS subscriber."""

    def __init__(self, *, camera=None) -> None:
        super().__init__("rebotarm_vision_node")
        defaults = {
            "camera.type": "gemini2", "camera.color_width": 640,
            "camera.color_height": 480, "camera.color_fps": 30,
            "camera.enable_depth": True, "camera.depth_width": 0,
            "camera.depth_height": 0, "camera.depth_fps": 30,
            "camera.enable_align": True, "camera.warmup_frames": 15,
            "camera.frame_timeout_ms": 1000, "camera.max_frame_skew_ms": 50.0,
            "ros.frame_id_depth": "camera_depth_frame", "ros.loop_rate_hz": 15.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        if value("camera.type") != "gemini2":
            raise ValueError("The active visual pipeline requires local gemini2; HTTP camera input is retired")
        if not value("camera.enable_depth") or not value("camera.enable_align"):
            raise ValueError("The grasp pipeline requires complete, aligned and calibrated RGB-D")
        self.frame_id = str(value("ros.frame_id_depth"))
        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.depth_camera_info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data)
        self.color_camera_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data)
        self.camera = camera or Gemini2Driver(Gemini2Config(
            color_width=int(value("camera.color_width")), color_height=int(value("camera.color_height")),
            color_fps=int(value("camera.color_fps")), enable_depth=True,
            depth_width=int(value("camera.depth_width")), depth_height=int(value("camera.depth_height")),
            depth_fps=int(value("camera.depth_fps")), frame_timeout_ms=int(value("camera.frame_timeout_ms")),
            enable_align=True, max_frame_skew_ms=float(value("camera.max_frame_skew_ms")),
        ))
        try:
            self.camera.open()
            self.camera.warmup(int(value("camera.warmup_frames")))
        except Exception:
            self.camera.close()
            raise
        self.create_timer(1.0 / max(float(value("ros.loop_rate_hz")), 1.0), self._on_timer)
        self.get_logger().info("Local registered RGB-D producer ready; color and depth use the depth optical frame")

    def _on_timer(self) -> None:
        # Stamp before acquisition/registration so their latency counts toward age.
        stamp = self.get_clock().now().to_msg()
        color, depth = self.camera.get_frame()
        if color is None or depth is None:
            self.get_logger().warn(
                f"RGB-D unavailable: {self.camera.last_debug_message}", throttle_duration_sec=5.0,
            )
            return
        try:
            payload = self.camera.get_camera_info()
            if payload is None:
                raise ValueError("device CameraInfo unavailable; refusing guessed intrinsics")
            info = camera_info_to_msg(payload, stamp, self.frame_id)
            camera_info_payload(info)
            if color.shape[:2] != depth.shape or depth.shape != (info.height, info.width):
                raise ValueError("registered image dimensions do not match device CameraInfo")
            self.color_pub.publish(color_to_msg(color, stamp, self.frame_id))
            self.depth_pub.publish(depth_to_msg(depth, stamp, self.frame_id))
            self.depth_camera_info_pub.publish(info)
            self.color_camera_info_pub.publish(info)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"RGB-D frame rejected: {exc}", throttle_duration_sec=5.0)

    def destroy_node(self):
        try:
            self.camera.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RebotArmVisionNode()
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
