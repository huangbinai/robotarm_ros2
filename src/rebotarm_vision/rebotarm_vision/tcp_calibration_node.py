from __future__ import annotations

import sys
import threading

import message_filters
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from .aruco_reference import (
    build_camera_matrix,
    detect_aruco_center_in_camera,
    transform_camera_point_to_base,
)
from .perception_frames import camera_info_payload, color_image_to_array, require_fresh
from .tcp_calibration import average_offsets, estimate_sample_offset, format_tcp_offset_yaml


def _tuple3(values, name: str) -> tuple[float, float, float]:
    items = list(values)
    if len(items) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return (float(items[0]), float(items[1]), float(items[2]))


def estimate_offset_from_transform(
    transform_stamped,
    *,
    tcp_reference_position: tuple[float, float, float],
) -> tuple[float, float, float]:
    transform = transform_stamped.transform
    translation = transform.translation
    rotation = transform.rotation
    return estimate_sample_offset(
        end_link_position=(translation.x, translation.y, translation.z),
        end_link_orientation_xyzw=(rotation.x, rotation.y, rotation.z, rotation.w),
        tcp_reference_position=tcp_reference_position,
    )


class AutoArucoReferenceProvider:
    def __init__(
        self,
        *,
        frame_supplier,
        tf_buffer,
        base_frame: str,
        camera_frame: str,
        lookup_timeout_sec: float,
        detector,
    ) -> None:
        self.frame_supplier = frame_supplier
        self.tf_buffer = tf_buffer
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.lookup_timeout_sec = lookup_timeout_sec
        self.detector = detector
        self.capture_stamp = None

    def reference_position(self) -> tuple[float, float, float]:
        color, info = self.frame_supplier()
        color_bgr = color_image_to_array(color)
        camera_point = self.detector(color_bgr, info)
        self.capture_stamp = color.header.stamp
        tf_msg = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            rclpy.time.Time.from_msg(self.capture_stamp),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )
        return transform_camera_point_to_base(tf_msg, camera_point_xyz=camera_point)


class TcpCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_tcp_calibration")
        self.declare_parameter("reference_mode", "manual")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_link_frame", "end_link")
        self.declare_parameter("tcp_reference_position", [0.0, 0.0, 0.0])
        self.declare_parameter("sample_count", 5)
        self.declare_parameter("lookup_timeout_sec", 0.5)
        self.declare_parameter("aruco.image_topic", "/camera/color/image_raw")
        self.declare_parameter("aruco.camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("aruco.max_frame_age_sec", 1.5)
        self.declare_parameter("aruco.camera_frame", "camera_depth_frame")
        self.declare_parameter("aruco.dictionary", "DICT_4X4_50")
        self.declare_parameter("aruco.marker_id", 0)
        self.declare_parameter("aruco.marker_length_m", 0.10)

        self.reference_mode = str(self.get_parameter("reference_mode").value).strip().lower()
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.end_link_frame = str(self.get_parameter("end_link_frame").value)
        self.tcp_reference_position = _tuple3(
            self.get_parameter("tcp_reference_position").value,
            "tcp_reference_position",
        )
        self.sample_count = max(1, int(self.get_parameter("sample_count").value))
        self.lookup_timeout_sec = max(0.05, float(self.get_parameter("lookup_timeout_sec").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.offset_samples: list[tuple[float, float, float]] = []
        self._aruco_frame = None
        self.reference_provider = self._build_reference_provider()

    def _build_reference_provider(self):
        if self.reference_mode == "manual":
            return None
        if self.reference_mode != "aruco":
            raise ValueError("reference_mode must be 'manual' or 'aruco'")

        camera_frame = str(self.get_parameter("aruco.camera_frame").value)
        dictionary_name = str(self.get_parameter("aruco.dictionary").value)
        marker_id = int(self.get_parameter("aruco.marker_id").value)
        marker_length_m = float(self.get_parameter("aruco.marker_length_m").value)
        self._aruco_subscribers = [
            message_filters.Subscriber(self, Image, str(self.get_parameter("aruco.image_topic").value), qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(self, CameraInfo, str(self.get_parameter("aruco.camera_info_topic").value), qos_profile=qos_profile_sensor_data),
        ]
        self._aruco_sync = message_filters.TimeSynchronizer(self._aruco_subscribers, 10)
        self._aruco_sync.registerCallback(lambda image, info: setattr(self, "_aruco_frame", (image, info)))

        def frame_supplier():
            frame = self._aruco_frame
            if frame is None:
                raise RuntimeError("waiting for local ROS image and CameraInfo")
            image, info = frame
            require_fresh(image.header, self.get_clock().now().nanoseconds,
                          float(self.get_parameter("aruco.max_frame_age_sec").value))
            if image.header.frame_id != camera_frame or info.header.frame_id != camera_frame:
                raise ValueError("ArUco optical frame does not match ROS CameraInfo")
            if (image.width, image.height) != (info.width, info.height):
                raise ValueError("ArUco image and CameraInfo dimensions differ")
            camera_info_payload(info)
            return frame

        def detector(color_bgr, info):
            intrinsics = camera_info_payload(info)
            camera_matrix = build_camera_matrix(**{key: intrinsics[key] for key in ("fx", "fy", "cx", "cy")})
            return detect_aruco_center_in_camera(
                color_bgr,
                camera_matrix=camera_matrix,
                marker_length_m=marker_length_m,
                dictionary_name=dictionary_name,
                marker_id=marker_id,
                dist_coeffs=list(info.d),
            )

        return AutoArucoReferenceProvider(
            frame_supplier=frame_supplier,
            tf_buffer=self.tf_buffer,
            base_frame=self.base_frame,
            camera_frame=camera_frame,
            lookup_timeout_sec=self.lookup_timeout_sec,
            detector=detector,
        )

    def capture_sample(self) -> tuple[float, float, float]:
        reference_position = self.tcp_reference_position
        if self.reference_provider is not None:
            reference_position = self.reference_provider.reference_position()
            self.get_logger().info(
                "aruco reference="
                f"({reference_position[0]:+.6f}, {reference_position[1]:+.6f}, "
                f"{reference_position[2]:+.6f}) in {self.base_frame}"
            )
        tf_msg = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.end_link_frame,
            rclpy.time.Time.from_msg(self.reference_provider.capture_stamp) if self.reference_provider is not None else rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )
        offset = estimate_offset_from_transform(
            tf_msg,
            tcp_reference_position=reference_position,
        )
        self.offset_samples.append(offset)
        return offset

    def average_result(self) -> tuple[float, float, float]:
        return average_offsets(self.offset_samples)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpCalibrationNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.get_logger().info(
            "TCP calibration ready. Align the real gripper center to "
            f"{node.reference_mode} reference in {node.base_frame}, then press Enter "
            f"for each of {node.sample_count} samples."
        )
        for index in range(node.sample_count):
            input(f"Sample {index + 1}/{node.sample_count}: press Enter after alignment...")
            try:
                offset = node.capture_sample()
            except Exception as exc:
                node.get_logger().warn(f"sample skipped: {exc}")
                continue
            node.get_logger().info(
                "sample "
                f"{len(node.offset_samples)} offset="
                f"({offset[0]:+.6f}, {offset[1]:+.6f}, {offset[2]:+.6f})"
            )

        result = node.average_result()
        print("")
        print("Suggested camera.yaml value:")
        print(format_tcp_offset_yaml(result))
        print("")
        print("Apply the same value to both rebotarm_grasp_tcp_frame and rebotarm_grasp_preview_sender.")
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
