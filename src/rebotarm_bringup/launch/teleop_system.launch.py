from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 键盘、示教录制和网页状态面板的组合入口；完整真机网页工作台优先使用 rebotarm_app。
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_hardware = LaunchConfiguration("use_hardware")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    channel = LaunchConfiguration("channel")
    panel = LaunchConfiguration("panel")
    web_execute_enabled = LaunchConfiguration("web_execute_enabled")
    record = LaunchConfiguration("record")
    record_path = LaunchConfiguration("record_path")
    teleop_config = LaunchConfiguration("teleop_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    bringup_share = FindPackageShare("rebotarm_bringup")
    config_share = FindPackageShare("rebotarm_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_hardware", default_value="false"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("panel", default_value="true"),
            DeclareLaunchArgument("web_execute_enabled", default_value="false"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument("record_path", default_value="teleop_records/teach_record.jsonl"),
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'",
            ),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "teleop_control.yaml"]
                ),
            ),
            # 复用键盘入口，避免重复维护控制器、模型和键盘节点。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "teleop_keyboard.launch.py"])
                ),
                launch_arguments={
                    "arm_namespace": arm_namespace,
                    "use_hardware": use_hardware,
                    "use_local_rviz": use_local_rviz,
                    "channel": channel,
                    "teach_record_path": record_path,
                    "start_teach_recorder": "false",
                    "teleop_config": teleop_config,
                    "keyboard_prefix": keyboard_prefix,
                }.items(),
            ),
            # 两种后端均由 teach 包提供唯一录制服务；core 默认不启动录制器。
            Node(
                package="rebotarm_teach",
                executable="TeachRecorderNode",
                name="teach_recorder_node",
                output="screen",
                parameters=[
                    teleop_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": record_path,
                        "start_on_launch": False,
                        "keyboard_quit_enabled": False,
                        "require_gravity_comp": use_hardware,
                    },
                ],
            ),
            # 网页面板读取同一份 teleop_control.yaml，并提供录制/回放操作。
            Node(
                package="rebotarm_dashboard",
                executable="TeleopStatusPanelNode",
                name="teleop_status_panel_node",
                output="screen",
                condition=IfCondition(panel),
                parameters=[
                    teleop_config,
                    {
                        "arm_namespace": arm_namespace,
                        "web_execute_enabled": web_execute_enabled,
                        "record_path": record_path,
                        "use_hardware": use_hardware,
                    },
                ],
            ),
        ]
    )
