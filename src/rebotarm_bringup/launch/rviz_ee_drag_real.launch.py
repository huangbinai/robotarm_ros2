from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_channel(context: LaunchContext) -> str:
    # auto 时探测常用 ACM 设备；显式传入的 channel 始终优先。
    channel = LaunchConfiguration("channel").perform(context)
    if channel and channel != "auto":
        return channel
    for candidate in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if Path(candidate).exists():
            return candidate
    return "/dev/ttyACM0"


def _include_system(context: LaunchContext):
    # 固定启用真实硬件、MoveIt 和真实 joint states，不启动 fake 状态。
    bringup_share = FindPackageShare("rebotarm_bringup")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "core.launch.py"])
            ),
            launch_arguments={
                "channel": _resolve_channel(context),
                "use_hardware": "true",
                "hardware_mode": "real",
                "use_local_rviz": "true",
                "use_moveit_preview": "true",
                "start_passive_joint_state_publisher": "false",
                "use_moveit_fake_joint_states": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    # 真机 RViz 末端拖动的薄封装，具体交互逻辑集中在 interactive_system 中。
    return LaunchDescription(
        [
            DeclareLaunchArgument("channel", default_value="auto"),
            OpaqueFunction(function=_include_system),
        ]
    )
