from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    bringup_share = FindPackageShare("rebotarm_bringup")
    interactive_share = FindPackageShare("rebotarm_interactive_control")
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    use_rviz = LaunchConfiguration("use_rviz")
    use_moveit_preview = LaunchConfiguration("use_moveit_preview")
    use_hardware = LaunchConfiguration("use_hardware")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    interactive_config = LaunchConfiguration("interactive_config")

    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "reBot-DevArm_fixend.urdf"]
    )
    rviz_config = PathJoinSubstitution(
        [bringup_share, "rviz", "interactive_system.rviz"]
    )
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_moveit_preview", default_value="false"),
            DeclareLaunchArgument("use_hardware", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument(
                "interactive_config",
                default_value=PathJoinSubstitution(
                    [interactive_share, "config", "interactive_control.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([moveit_share, "launch", "demo.launch.py"])
                ),
                condition=IfCondition(use_moveit_preview),
                launch_arguments={"use_rviz": "false"}.items(),
            ),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                condition=IfCondition(use_hardware),
                parameters=[
                    {
                        "arm_config": arm_config,
                        "gripper_config": gripper_config,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "frame_id": frame_id,
                        "ee_frame_id": ee_frame_id,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
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
                condition=IfCondition(use_moveit_preview),
            ),
            Node(
                package="rebotarm_interactive_control",
                executable="PreviewNode",
                name="preview_node",
                output="screen",
                parameters=[
                    interactive_config,
                    {
                        "arm_namespace": arm_namespace,
                        "preview_backend": "moveit",
                    },
                ],
                condition=IfCondition(use_moveit_preview),
            ),
            Node(
                package="rebotarm_interactive_control",
                executable="PreviewNode",
                name="preview_node",
                output="screen",
                parameters=[
                    interactive_config,
                    {
                        "arm_namespace": arm_namespace,
                        "preview_backend": "sdk",
                    },
                ],
                condition=UnlessCondition(use_moveit_preview),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="interactive_joint_state_publisher",
                output="screen",
                condition=IfCondition(use_moveit_preview),
                parameters=[
                    {"robot_description": robot_description},
                    {"rate": 30.0},
                ],
            ),
            Node(
                package="rebotarm_interactive_control",
                executable="ExecutionNode",
                name="execution_node",
                output="screen",
                parameters=[
                    interactive_config,
                    {
                        "arm_namespace": arm_namespace,
                    },
                ],
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
