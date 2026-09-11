from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_channel(context: LaunchContext) -> str:
    # SDK 仅识别 /dev/tty 前缀，固定设备标识必须先解析为实际设备路径。
    channel = LaunchConfiguration("channel").perform(context).strip()
    if channel and channel != "auto":
        if channel.startswith("/dev/"):
            return str(Path(channel).resolve(strict=True))
        return channel
    candidates = {
        str(path.resolve())
        for path in Path("/dev/serial/by-id").glob("usb-HDSC_CDC_Device_*")
        if path.exists()
    }
    if not candidates:
        candidates = {str(path) for path in Path("/dev").glob("ttyACM*")}
    if len(candidates) != 1:
        raise RuntimeError(
            "无法唯一确定机械臂串口。请检查 USB 连接，或在 rviz_real.yaml 中设置 channel。"
            f" 候选设备：{sorted(candidates)}"
        )
    return candidates.pop()


def _include_system(context: LaunchContext):
    # 固定启用真实硬件、MoveIt 和真实 joint states，不启动 fake 状态。
    bringup_share = FindPackageShare("rebotarm_bringup")
    channel = _resolve_channel(context)
    return [
        LogInfo(msg=f"真机 RViz：串口 {channel}；Plan 只规划，Execute 校验轨迹后按需使能并执行。"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "core.launch.py"])
            ),
            launch_arguments={
                "channel": channel,
                "use_hardware": "true",
                "hardware_mode": "real",
                "use_local_rviz": "true",
                "use_moveit_preview": "true",
                "start_passive_joint_state_publisher": "false",
                "use_moveit_fake_joint_states": "false",
                "shutdown_safe_home": LaunchConfiguration("shutdown_safe_home"),
                "enable_on_trajectory": "true",
            }.items(),
        ),
    ]


def generate_launch_description():
    # 日常默认值集中维护；命令行只在临时覆盖配置时使用。
    config_path = Path(get_package_share_directory("rebotarm_bringup")) / "config" / "rviz_real.yaml"
    with config_path.open(encoding="utf-8") as stream:
        defaults = yaml.safe_load(stream)
    return LaunchDescription(
        [
            DeclareLaunchArgument("channel", default_value=str(defaults["channel"])),
            DeclareLaunchArgument(
                "shutdown_safe_home", default_value=str(defaults["shutdown_safe_home"]).lower()
            ),
            OpaqueFunction(function=_include_system),
        ]
    )
