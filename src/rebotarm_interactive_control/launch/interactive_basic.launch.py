from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("interactive_config")
    arm_namespace = LaunchConfiguration("arm_namespace")

    package_share = FindPackageShare("rebotarm_interactive_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interactive_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "interactive_control.yaml"]
                ),
            ),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            Node(
                package="rebotarm_interactive_control",
                executable="InteractiveTargetNode",
                name="interactive_control",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "arm_namespace": arm_namespace,
                    },
                ],
            ),
        ]
    )
