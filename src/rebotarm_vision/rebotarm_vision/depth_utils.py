from __future__ import annotations

import numpy as np


def depth_image_to_array(msg) -> np.ndarray:
    if msg.encoding not in ("mono16", "16UC1"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0 or step < width * 2 or step % 2 or len(msg.data) != step * height:
        raise ValueError("invalid depth image dimensions/stride/buffer")
    dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    rows = np.frombuffer(msg.data, dtype=dtype).reshape(height, step // 2)
    return np.ascontiguousarray(rows[:, :width], dtype=np.uint16)
