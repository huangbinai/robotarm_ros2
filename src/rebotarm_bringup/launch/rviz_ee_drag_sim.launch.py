from __future__ import annotations

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 仿真 RViz 末端拖动入口：启用 MoveIt 预览和 fake joint states，不连接真机。
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
                    "start_passive_joint_state_publisher": "true",
                    "use_moveit_fake_joint_states": "true",
                }.items(),
            )
        ]
    )
