"""Start only the explicitly selected, headless MuJoCo ROS adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("rebotarm_simulation")) / "config" / "mujoco_sim.yaml"
    show_viewer = LaunchConfiguration("show_viewer")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_viewer",
                default_value="false",
                description="Open a MuJoCo Viewer synchronized with this ROS simulation instance",
            ),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_node",
                name="rebotarm_mujoco_node",
                output="screen",
                parameters=[
                    str(config),
                    {"backend": "mujoco", "headless": True, "show_viewer": show_viewer},
                ],
            )
        ]
    )
