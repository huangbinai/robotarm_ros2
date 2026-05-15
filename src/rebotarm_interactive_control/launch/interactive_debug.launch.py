from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rebotarm_interactive_control")
    marker_frame_id = LaunchConfiguration("marker_frame_id")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution([package_share, "rviz", "interactive_debug.rviz"])
    debug_config = PathJoinSubstitution([package_share, "config", "interactive_debug.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("marker_frame_id", default_value="base_link"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="rebotarm_interactive_control",
                executable="InteractiveMarkerSmokeNode",
                name="interactive_marker_smoke",
                output="screen",
                parameters=[
                    debug_config,
                    {
                        "marker_frame_id": marker_frame_id,
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="interactive_debug_world_to_base",
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
