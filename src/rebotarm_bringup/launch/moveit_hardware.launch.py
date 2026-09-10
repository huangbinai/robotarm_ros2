from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 兼容入口：真实 MoveIt 链路统一由 interactive_system 维护。
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    arm_namespace = LaunchConfiguration("arm_namespace")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    teach_record_path = LaunchConfiguration("teach_record_path")
    teach_record_rate_hz = LaunchConfiguration("teach_record_rate_hz")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    use_rviz = LaunchConfiguration("use_rviz")
    interactive_launch = PathJoinSubstitution(
        [bringup_share, "launch", "core.launch.py"]
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("teach_record_path", default_value="teleop_records/teach_record.jsonl"),
            DeclareLaunchArgument("teach_record_rate_hz", default_value="150.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(interactive_launch),
                launch_arguments={
                    "use_hardware": "true",
                    "hardware_mode": "real",
                    "use_moveit_preview": "true",
                    "use_moveit_fake_joint_states": "false",
                    "use_local_rviz": use_rviz,
                    "arm_config": arm_config,
                    "gripper_config": gripper_config,
                    "arm_namespace": arm_namespace,
                    "channel": channel,
                    "joint_state_rate": joint_state_rate,
                    "teach_record_path": teach_record_path,
                    "teach_record_rate_hz": teach_record_rate_hz,
                    "cmd_arbitration": cmd_arbitration,
                    "frame_id": frame_id,
                    "ee_frame_id": ee_frame_id,
                }.items(),
            ),
        ]
    )
