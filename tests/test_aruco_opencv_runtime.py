"""Exercise actual installed OpenCV, including Ubuntu's pre-4.7 ArUco API."""
import cv2
import numpy as np
import pytest

from rebotarm_vision.aruco_reference import build_camera_matrix, detect_aruco_center_in_camera


def marker_image():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 200)
    else:
        marker = cv2.aruco.drawMarker(dictionary, 0, 200)
    image = np.full((400, 400), 255, dtype=np.uint8)
    image[100:300, 100:300] = marker
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def test_marker_geometry_with_installed_opencv():
    position = detect_aruco_center_in_camera(
        marker_image(), camera_matrix=build_camera_matrix(fx=400., fy=400., cx=199.5, cy=199.5),
        marker_length_m=0.1, marker_id=0,
    )
    assert position[0] == pytest.approx(0., abs=0.002)
    assert position[1] == pytest.approx(0., abs=0.002)
    assert position[2] == pytest.approx(0.2, abs=0.004)


def test_another_marker_does_not_produce_a_reference():
    with pytest.raises(ValueError, match="id 1 not detected"):
        detect_aruco_center_in_camera(
            marker_image(), camera_matrix=build_camera_matrix(fx=400., fy=400., cx=199.5, cy=199.5),
            marker_length_m=0.1, marker_id=1,
        )
