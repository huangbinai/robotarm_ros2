from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Deprecated compatibility entrypoint.
    # Prefer: ros2 launch rebotarm_bringup core.launch.py
    bringup_share = FindPackageShare("rebotarm_bringup")
    argument_defaults = {
        "arm_config": PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
        "gripper_config": PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
        "arm_namespace": "rebotarm",
        "channel": "",
        "shutdown_safe_home": "true",
        "joint_state_rate": "100.0",
        "cmd_arbitration": "reject",
        "use_local_rviz": "true",
        "use_moveit_preview": "false",
        "hardware_mode": "auto",
        "use_hardware": "false",
        "use_sim_time": "false",
        "teach_record_path": "teleop_records/teach_record.jsonl",
        "teach_record_rate_hz": "150.0",
        "frame_id": "base_link",
        "ee_frame_id": "end_link",
        "start_passive_joint_state_publisher": "true",
        "use_moveit_fake_joint_states": "true",
        "rviz_config": PathJoinSubstitution(
            [bringup_share, "rviz", "interactive_system.rviz"]
        ),
        "interactive_config": PathJoinSubstitution(
            [
                FindPackageShare("rebotarm_interactive_control"),
                "config",
                "interactive_control.yaml",
            ]
        ),
    }
    argument_names = list(argument_defaults)
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=argument_defaults[name])
                for name in argument_names
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "core.launch.py"])
                ),
                launch_arguments={
                    name: LaunchConfiguration(name) for name in argument_names
                }.items(),
            )
        ]
    )
