from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class Gemini2Config:
    color_width: int
    color_height: int
    color_fps: int
    enable_depth: bool
    depth_width: int
    depth_height: int
    depth_fps: int
    frame_timeout_ms: int
    enable_align: bool
    max_frame_skew_ms: float = 50.0


def scaled_depth_mm(depth_frame) -> np.ndarray:
    scale = float(depth_frame.get_depth_scale())
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("invalid device depth scale")
    raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
        depth_frame.get_height(), depth_frame.get_width()
    )
    values = raw.astype(np.float64) * scale
    return np.where((values > 0) & (values <= 65535), np.rint(values), 0).astype(np.uint16)


class Gemini2Driver:
    """Capture a complete frameset; C2D registration keeps the depth optical frame."""

    def __init__(self, config: Gemini2Config) -> None:
        self._config = config
        self._pipeline = None
        self._align_filter = None
        self._camera_info = None
        self._rectify_maps = None
        self._last_device_stamp = None
        self._last_debug_message = "driver_not_opened"

    def open(self) -> None:
        from pyorbbecsdk import (
            AlignFilter, Config, OBFormat, OBFrameAggregateOutputMode,
            OBSensorType, OBStreamType, Pipeline,
        )

        pipeline = Pipeline()
        config = Config()
        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = None
        for fmt in (OBFormat.RGB, OBFormat.MJPG):
            try:
                color_profile = color_profiles.get_video_stream_profile(
                    self._config.color_width, self._config.color_height,
                    fmt, self._config.color_fps,
                )
                break
            except Exception:
                continue
        if color_profile is None:
            color_profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(color_profile)
        if self._config.enable_depth:
            profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = profiles.get_video_stream_profile(
                self._config.depth_width, self._config.depth_height,
                OBFormat.Y16, self._config.depth_fps,
            )
            config.enable_stream(depth_profile)
            config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
            pipeline.enable_frame_sync()
        if self._config.enable_align:
            if not self._config.enable_depth:
                raise ValueError("registered RGB-D requires the depth stream")
            self._align_filter = AlignFilter(align_to_stream=OBStreamType.DEPTH_STREAM)
        self._pipeline = pipeline
        try:
            pipeline.start(config)
        except Exception:
            self.close()
            raise

    def warmup(self, frames: int) -> bool:
        for _ in range(max(frames, 0)):
            color, depth = self.get_frame()
            if color is not None and (depth is not None or not self._config.enable_depth):
                return True
        return False

    def _rectify(self, color, depth, depth_frame):
        profile = depth_frame.get_stream_profile().as_video_stream_profile()
        intrinsics = profile.get_intrinsic()
        distortion = profile.get_distortion()
        width, height = int(intrinsics.width), int(intrinsics.height)
        fx, fy, cx, cy = (float(getattr(intrinsics, key)) for key in ("fx", "fy", "cx", "cy"))
        if color.shape[:2] != depth.shape or depth.shape != (height, width):
            raise ValueError("C2D output and calibrated depth dimensions do not match")
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            raise ValueError("device intrinsics are invalid")
        # Orbbec Brown-Conrady coefficients in OpenCV rational-polynomial order.
        coefficients = [float(getattr(distortion, key)) for key in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")]
        if not np.isfinite(coefficients).all():
            raise ValueError("device distortion coefficients are invalid")
        signature = (width, height, fx, fy, cx, cy, *coefficients)
        if self._rectify_maps is None or self._rectify_maps[0] != signature:
            matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
            maps = cv2.initUndistortRectifyMap(
                matrix, np.asarray(coefficients), np.eye(3), matrix,
                (width, height), cv2.CV_32FC1,
            )
            self._rectify_maps = (signature, maps)
        map_x, map_y = self._rectify_maps[1]
        color = cv2.remap(color, map_x, map_y, cv2.INTER_LINEAR)
        depth = cv2.remap(depth, map_x, map_y, cv2.INTER_NEAREST)
        self._camera_info = dict(
            width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy,
            distortion_model="plumb_bob", d=[0.0] * 5,
        )
        return np.ascontiguousarray(color), np.ascontiguousarray(depth)

    def get_frame(self, allow_partial: bool = False):
        # Kept for the standalone camera preview API; never reuse cached images.
        del allow_partial
        if self._pipeline is None:
            return None, None
        try:
            from pyorbbecsdk import OBFormat

            frames = self._pipeline.wait_for_frames(self._config.frame_timeout_ms)
            if frames is None:
                self._last_debug_message = "no_frames"
                return None, None
            color_frame, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
            if color_frame is None or (self._config.enable_depth and depth_frame is None):
                self._last_debug_message = "incomplete_frameset"
                return None, None
            color_time = float(color_frame.get_timestamp())
            depth_time = float(depth_frame.get_timestamp()) if depth_frame is not None else color_time
            if not all(math.isfinite(value) for value in (color_time, depth_time, self._config.max_frame_skew_ms)):
                raise ValueError("non-finite device timestamps or allowed skew")
            if abs(color_time - depth_time) > self._config.max_frame_skew_ms:
                raise ValueError("device color/depth timestamps exceed allowed skew")
            device_stamp = (color_time, depth_time)
            if self._last_device_stamp is not None and any(
                current <= previous for current, previous in zip(device_stamp, self._last_device_stamp)
            ):
                self._last_debug_message = "duplicate_or_out_of_order_frameset"
                return None, None
            self._last_device_stamp = device_stamp
            if self._align_filter is not None:
                aligned = self._align_filter.process(frames)
                if aligned is None:
                    raise ValueError("C2D alignment failed")
                frames = aligned.as_frame_set()
                color_frame, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
                if color_frame is None or depth_frame is None:
                    raise ValueError("C2D returned an incomplete frameset")
            width, height = color_frame.get_width(), color_frame.get_height()
            raw = np.frombuffer(color_frame.get_data(), dtype=np.uint8)
            fmt = color_frame.get_format()
            if fmt == OBFormat.MJPG:
                color = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            elif fmt == OBFormat.RGB:
                color = cv2.cvtColor(raw.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
            elif fmt == OBFormat.BGR:
                color = raw.reshape(height, width, 3)
            else:
                raise ValueError(f"unsupported camera color format: {fmt}")
            if color is None:
                raise ValueError("color decoding failed")
            depth = scaled_depth_mm(depth_frame) if depth_frame is not None else None
            if self._align_filter is not None:
                color, depth = self._rectify(color, depth, depth_frame)
            self._last_debug_message = "complete_registered_frameset" if self._align_filter else "complete_frameset"
            return color, depth
        except Exception as exc:
            self._last_debug_message = f"{type(exc).__name__}: {exc}"
            return None, None

    def get_camera_info(self):
        return dict(self._camera_info) if self._camera_info is not None else None

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None
        self._align_filter = None
        self._camera_info = None
        self._rectify_maps = None
        self._last_device_stamp = None

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message
