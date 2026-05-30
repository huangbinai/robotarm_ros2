from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image


def color_to_msg(image_bgr, stamp, frame_id: str) -> Image:
    image_bgr = np.ascontiguousarray(image_bgr)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(image_bgr.shape[0])
    msg.width = int(image_bgr.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(image_bgr.shape[1] * image_bgr.shape[2])
    msg.data = image_bgr.tobytes()
    return msg


def depth_to_msg(depth_mm, stamp, frame_id: str) -> Image:
    depth_mm = np.ascontiguousarray(depth_mm)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(depth_mm.shape[0])
    msg.width = int(depth_mm.shape[1])
    msg.encoding = "mono16"
    msg.is_bigendian = 0
    msg.step = int(depth_mm.shape[1] * 2)
    msg.data = depth_mm.tobytes()
    return msg
