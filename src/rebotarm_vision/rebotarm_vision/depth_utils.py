from __future__ import annotations

import numpy as np


def depth_image_to_array(msg) -> np.ndarray:
    if msg.encoding not in ("mono16", "16UC1"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    depth = np.frombuffer(msg.data, dtype=np.uint16)
    return depth.reshape((msg.height, msg.width))
