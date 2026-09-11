from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 独立示教录制入口；使用 teleop_control.yaml 的采样率、关节列表和重力补偿默认值。
    # 它不负责启动控制器，真机控制器应由其他整机入口先启动。
    arm_namespace = LaunchConfiguration("arm_namespace")
    record_path = LaunchConfiguration("record_path")
    auto_start_gravity_comp = LaunchConfiguration("auto_start_gravity_comp")
    teleop_config = LaunchConfiguration("teleop_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    config_share = FindPackageShare("rebotarm_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("record_path", default_value="teleop_records/teach_record.jsonl"),
            DeclareLaunchArgument("auto_start_gravity_comp", default_value="false"),
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="",
            ),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "teleop_control.yaml"]
                ),
            ),
            # 只启动 TeachRecorderNode，record_path 和自动重力补偿由 launch 参数覆盖。
            Node(
                package="rebotarm_teach",
                executable="TeachRecorderNode",
                name="teach_recorder_node",
                output="screen",
                prefix=keyboard_prefix,
                parameters=[
                    teleop_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": record_path,
                        "auto_start_gravity_comp": auto_start_gravity_comp,
                    },
                ],
            ),
        ]
    )
