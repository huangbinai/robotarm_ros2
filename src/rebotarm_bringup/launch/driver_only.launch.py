# 该启动文件仅启动机械臂控制器节点，适用于需要单独控制机械臂的场景。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 最小真机入口：只启动 reBotArmController，不启动 RViz、MoveIt、网页或视觉节点。
    bringup_share = FindPackageShare("rebotarm_bringup")
    description_share = FindPackageShare("rebotarm_description")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    # 即使是最小驱动入口，也必须同时加载运行参数和控制器安全边界。
    controller_safety_params = PathJoinSubstitution(
        [bringup_share, "config", "controller_safety.yaml"]
    )
    controller_runtime_params = PathJoinSubstitution(
        [bringup_share, "config", "controller_runtime.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([description_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([description_share, "config", "gripper.yaml"]),
            ),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[controller_runtime_params,
                    controller_safety_params,
                    {
                        "arm_config": arm_config,
                        "gripper_config": gripper_config,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                    }
                ],
            ),
        ]
    )
