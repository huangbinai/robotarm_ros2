import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return file.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    rviz_config = PathJoinSubstitution([moveit_share, "rviz", "moveit.rviz"])

    robot_description = {
        "robot_description": load_file(
            "rebotarm_bringup", "description/urdf/reBot-DevArm_fixend.urdf"
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "rebotarm_moveit_config", "config/reBot-DevArm_fixend.srdf"
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "rebotarm_moveit_config", "config/kinematics.yaml"
        )
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "rebotarm_moveit_config", "config/joint_limits.yaml"
        )
    }
    ompl_planning_yaml = load_yaml(
        "rebotarm_moveit_config", "config/ompl_planning.yaml"
    )
    planning_pipelines = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    moveit_controllers = load_yaml(
        "rebotarm_moveit_config", "config/moveit_controllers.yaml"
    )
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
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
                parameters=[robot_description],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    planning_pipelines,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters,
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    planning_pipelines,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
