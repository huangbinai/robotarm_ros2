from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Keyboard teleop wrapper. The core entry owns the robot runtime chain.
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_hardware = LaunchConfiguration("use_hardware")
    hardware_mode = LaunchConfiguration("hardware_mode")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    teach_record_path = LaunchConfiguration("teach_record_path")
    teach_record_rate_hz = LaunchConfiguration("teach_record_rate_hz")
    teleop_config = LaunchConfiguration("teleop_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    bringup_share = FindPackageShare("rebotarm_bringup")
    core_launch = PathJoinSubstitution(
        [bringup_share, "launch", "core.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_hardware", default_value="false"),
            DeclareLaunchArgument(
                "hardware_mode",
                default_value="auto",
                description="none, sim, real, or auto (legacy use_hardware)",
            ),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument(
                "teach_record_path",
                default_value="teleop_records/teach_record.jsonl",
            ),
            DeclareLaunchArgument("teach_record_rate_hz", default_value="150.0"),
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'",
            ),
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "gripper.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rebotarm_interactive_control"),
                        "config",
                        "teleop_control.yaml",
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(core_launch),
                launch_arguments={
                    "arm_namespace": arm_namespace,
                    "arm_config": arm_config,
                    "gripper_config": gripper_config,
                    "channel": channel,
                    "joint_state_rate": joint_state_rate,
                    "teach_record_path": teach_record_path,
                    "teach_record_rate_hz": teach_record_rate_hz,
                    "use_hardware": use_hardware,
                    "hardware_mode": hardware_mode,
                    "use_moveit_preview": "false",
                    "use_local_rviz": use_local_rviz,
                    "rviz_config": PathJoinSubstitution(
                        [bringup_share, "rviz", "rebotarm.rviz"]
                    ),
                    "interactive_config": teleop_config,
                }.items(),
            ),
            Node(
                package="rebotarm_teleop",
                executable="TeleopKeyboardNode",
                name="teleop_keyboard_node",
                output="screen",
                prefix=keyboard_prefix,
                parameters=[teleop_config, {"arm_namespace": arm_namespace}],
            ),
        ]
    )
