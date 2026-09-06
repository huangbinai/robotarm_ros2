"""Stable names shared by model generation and simulation runtime.

This module deliberately has no MuJoCo or ROS dependency.  The URDF converter,
runtime, front ends, and tests can therefore agree on model names without
creating a build-time/runtime dependency cycle.
"""

from __future__ import annotations


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
FINGER_JOINT_NAMES = ("left_finger_joint", "right_finger_joint")
JOINT_NAMES = ARM_JOINT_NAMES + FINGER_JOINT_NAMES

HOME_JOINT_POSITIONS = (0.0, -0.8, -1.0, 0.3, 0.0, 0.0)
HOME_KEYFRAME_NAME = "home"
END_EFFECTOR_SITE_NAME = "ee_site"
TEST_CUBE_BODY_NAME = "test_cube"
DEFAULT_SCENE_RESOURCE = "models/rebotarm/scene.xml"


def actuator_name_for_joint(joint_name: str) -> str:
    """Return the canonical MJCF actuator name for a model joint."""
    name = str(joint_name)
    if name in ARM_JOINT_NAMES:
        return f"{name}_torque"
    if name in FINGER_JOINT_NAMES:
        return f"{name.removesuffix('_joint')}_force"
    raise ValueError(f"unknown reBotArm joint: {name!r}")
