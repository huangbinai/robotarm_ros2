from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vision_python_executable", default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3")),
        Node(package="rebotarm_vision", executable="rebotarm_graspnet_viewer",
             name="rebotarm_graspnet_viewer", output="screen",
             prefix=LaunchConfiguration("vision_python_executable"),
             parameters=[str(Path(get_package_share_directory("rebotarm_vision")) / "config/graspnet_viewer.yaml")]),
    ])
