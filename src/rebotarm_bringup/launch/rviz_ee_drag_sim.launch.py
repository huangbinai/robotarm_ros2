from __future__ import annotations

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 单一 RViz 仿真执行后端；不是 MuJoCo 物理仿真，不连接真机。
    bringup_share = FindPackageShare("rebotarm_bringup")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "core.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_hardware": "false",
                    "hardware_mode": "sim",
                    "use_local_rviz": "true",
                    "use_moveit_preview": "true",
                    "start_passive_joint_state_publisher": "false",
                    "use_moveit_fake_joint_states": "false",
                }.items(),
            ),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_rviz_fake_controller",
                name="rebotarm_rviz_fake_controller",
                output="screen",
                parameters=[{"arm_namespace": "rebotarm"}],
            ),
        ]
    )
