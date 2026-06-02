from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_vision_launch_does_not_wrap_static_tf_in_ros2_run():
    text = (ROOT / "src" / "rebotarm_vision" / "launch" / "vision.launch.py").read_text(encoding="utf-8")

    assert "exec ros2 run tf2_ros static_transform_publisher" not in text
    assert "package=\"tf2_ros\"" in text
    assert "executable=\"static_transform_publisher\"" in text


def test_interactive_launch_honors_start_interaction_nodes_flag():
    text = (ROOT / "src" / "rebotarm_bringup" / "launch" / "interactive_system.launch.py").read_text(encoding="utf-8")

    assert "DeclareLaunchArgument(\"start_interaction_nodes\"" in text
    assert "start_interaction_nodes = LaunchConfiguration(\"start_interaction_nodes\")" in text
    assert "IfCondition(start_interaction_nodes)" in text


def test_visual_grasp_launch_exposes_adaptive_gripper_and_retreat_params():
    text = (ROOT / "src" / "rebotarm_bringup" / "launch" / "visual_grasp_system.launch.py").read_text(encoding="utf-8")

    for name in (
        "auto_gripper_effort",
        "min_gripper_effort",
        "max_gripper_effort",
        "max_allowed_grasp_width_m",
        "gripper_grasp_enabled",
        "gripper_grasp_close_force",
        "gripper_grasp_timeout_sec",
        "gripper_grasp_min_close_time_sec",
        "gripper_grasp_velocity_threshold",
        "gripper_grasp_min_closure_distance_m",
        "safe_retreat_enabled",
        "safe_retreat_min_lift_z_m",
        "safe_retreat_distance_m",
        "safe_retreat_axis_xyz",
        "safe_home_after_grasp",
    ):
        assert f'DeclareLaunchArgument("{name}"' in text
        assert f'"{name}": {name}' in text
