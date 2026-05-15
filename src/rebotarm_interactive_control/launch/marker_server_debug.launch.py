from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rebotarm_interactive_control")
    interactive_config = PathJoinSubstitution(
        [package_share, "config", "interactive_control.yaml"]
    )
    rviz_config = PathJoinSubstitution([package_share, "rviz", "marker_server_debug.rviz"])
    use_rviz = LaunchConfiguration("use_rviz")
    arm_namespace = LaunchConfiguration("arm_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            Node(
                package="rebotarm_interactive_control",
                executable="MarkerServerNode",
                name="marker_server",
                output="screen",
                parameters=[
                    interactive_config,
                    {
                        "arm_namespace": arm_namespace,
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="marker_server_world_to_base",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
