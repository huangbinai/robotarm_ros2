from types import SimpleNamespace

import numpy as np
import pytest


def _depth_message(array: np.ndarray, encoding: str):
    return SimpleNamespace(
        encoding=encoding,
        data=array.tobytes(),
        height=array.shape[0],
        width=array.shape[1],
    )


@pytest.mark.parametrize("encoding", ["mono16", "16UC1"])
def test_depth_image_to_array_preserves_uint16_depth(encoding):
    from rebotarm_vision.depth_utils import depth_image_to_array

    expected = np.array([[100, 200], [300, 400]], dtype=np.uint16)

    result = depth_image_to_array(_depth_message(expected, encoding))

    np.testing.assert_array_equal(result, expected)
    assert result.dtype == np.uint16


def test_depth_image_to_array_rejects_unsupported_encoding():
    from rebotarm_vision.depth_utils import depth_image_to_array

    message = _depth_message(np.ones((2, 2), dtype=np.uint16), "32FC1")

    with pytest.raises(ValueError, match="unsupported depth encoding: 32FC1"):
        depth_image_to_array(message)
