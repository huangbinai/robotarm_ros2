from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    rviz_config = PathJoinSubstitution([moveit_share, "rviz", "moveit.rviz"])

    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm.urdf")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    sensors_3d = {
        "sensors": ["no_depth_sensor"],
        "no_depth_sensor": {"sensor_plugin": "~"},
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="screen",
                arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    {"rate": 30.0},
                ],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    trajectory_execution,
                    sensors_3d,
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
