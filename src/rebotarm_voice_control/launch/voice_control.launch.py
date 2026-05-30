from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_voice_control_node",
                name="rebotarm_voice_control_node",
                output="screen",
            )
        ]
    )
