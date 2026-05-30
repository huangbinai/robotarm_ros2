from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")

    arm_namespace = LaunchConfiguration("arm_namespace")
    use_hardware = LaunchConfiguration("use_hardware")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    execution_mode = LaunchConfiguration("execution_mode")
    start_vision = LaunchConfiguration("start_vision")
    start_grasp_preview = LaunchConfiguration("start_grasp_preview")
    start_visual_grasp_executor = LaunchConfiguration("start_visual_grasp_executor")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")
    pose_mode = LaunchConfiguration("pose_mode")
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    base_z_offset_m = LaunchConfiguration("base_z_offset_m")
    min_target_z_m = LaunchConfiguration("min_target_z_m")
    grasp_base_z_offset_m = LaunchConfiguration("grasp_base_z_offset_m")
    pose_policy = LaunchConfiguration("pose_policy")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    base_approach_axis_xyz = LaunchConfiguration("base_approach_axis_xyz")
    base_pregrasp_distance_m = LaunchConfiguration("base_pregrasp_distance_m")
    lift_z_m = LaunchConfiguration("lift_z_m")
    close_position_m = LaunchConfiguration("close_position_m")
    close_max_effort = LaunchConfiguration("close_max_effort")
    open_before_approach = LaunchConfiguration("open_before_approach")
    auto_gripper_width = LaunchConfiguration("auto_gripper_width")
    auto_gripper_effort = LaunchConfiguration("auto_gripper_effort")
    open_clearance_m = LaunchConfiguration("open_clearance_m")
    close_margin_m = LaunchConfiguration("close_margin_m")
    min_gripper_effort = LaunchConfiguration("min_gripper_effort")
    max_gripper_effort = LaunchConfiguration("max_gripper_effort")
    max_allowed_grasp_width_m = LaunchConfiguration("max_allowed_grasp_width_m")
    safe_retreat_enabled = LaunchConfiguration("safe_retreat_enabled")
    safe_retreat_min_lift_z_m = LaunchConfiguration("safe_retreat_min_lift_z_m")
    safe_retreat_distance_m = LaunchConfiguration("safe_retreat_distance_m")
    safe_retreat_axis_xyz = LaunchConfiguration("safe_retreat_axis_xyz")
    safe_home_after_grasp = LaunchConfiguration("safe_home_after_grasp")
    plan_only_stage_pause_sec = LaunchConfiguration("plan_only_stage_pause_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_hardware", default_value="true"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="plan_only"),
            DeclareLaunchArgument("start_vision", default_value="true"),
            DeclareLaunchArgument("start_grasp_preview", default_value="false"),
            DeclareLaunchArgument("start_visual_grasp_executor", default_value="true"),
            DeclareLaunchArgument("start_visual_grasp_markers", default_value="true"),
            DeclareLaunchArgument("pose_mode", default_value="pregrasp"),
            DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]"),
            DeclareLaunchArgument("target_base_offset_xyz", default_value="[0.0, 0.01, 0.0]"),
            DeclareLaunchArgument("base_z_offset_m", default_value="0.05"),
            DeclareLaunchArgument("min_target_z_m", default_value="0.12"),
            DeclareLaunchArgument("grasp_base_z_offset_m", default_value="0.0"),
            DeclareLaunchArgument("pose_policy", default_value="base_axis"),
            DeclareLaunchArgument("fixed_grasp_orientation_xyzw", default_value="[0.0, 0.0, 0.0, 1.0]"),
            DeclareLaunchArgument("base_approach_axis_xyz", default_value="[1.0, 0.0, 0.0]"),
            DeclareLaunchArgument("base_pregrasp_distance_m", default_value="0.08"),
            DeclareLaunchArgument("lift_z_m", default_value="0.08"),
            DeclareLaunchArgument("close_position_m", default_value="0.025"),
            DeclareLaunchArgument("close_max_effort", default_value="0.3"),
            DeclareLaunchArgument("open_before_approach", default_value="true"),
            DeclareLaunchArgument("auto_gripper_width", default_value="true"),
            DeclareLaunchArgument("auto_gripper_effort", default_value="true"),
            DeclareLaunchArgument("open_clearance_m", default_value="0.02"),
            DeclareLaunchArgument("close_margin_m", default_value="0.012"),
            DeclareLaunchArgument("min_gripper_effort", default_value="0.22"),
            DeclareLaunchArgument("max_gripper_effort", default_value="0.60"),
            DeclareLaunchArgument("max_allowed_grasp_width_m", default_value="0.085"),
            DeclareLaunchArgument("safe_retreat_enabled", default_value="true"),
            DeclareLaunchArgument("safe_retreat_min_lift_z_m", default_value="0.24"),
            DeclareLaunchArgument("safe_retreat_distance_m", default_value="0.06"),
            DeclareLaunchArgument("safe_retreat_axis_xyz", default_value="[-1.0, 0.0, 0.0]"),
            DeclareLaunchArgument("safe_home_after_grasp", default_value="false"),
            DeclareLaunchArgument("plan_only_stage_pause_sec", default_value="3.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "interactive_system.launch.py"])
                ),
                launch_arguments={
                    "arm_namespace": arm_namespace,
                    "use_moveit_preview": "true",
                    "use_hardware": use_hardware,
                    "use_local_rviz": use_local_rviz,
                    "execution_mode": execution_mode,
                    "start_interaction_nodes": "false",
                    "start_passive_joint_state_publisher": "false",
                    "rviz_config": PathJoinSubstitution([bringup_share, "rviz", "visual_grasp_system.rviz"]),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])
                ),
                condition=IfCondition(start_vision),
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_send_grasp_preview",
                name="rebotarm_grasp_preview_sender",
                output="screen",
                condition=IfCondition(start_grasp_preview),
                parameters=[
                    {
                        "input_topic": "/grasp/plan",
                        "output_topic": ["/", arm_namespace, "/interactive_control/pose_target"],
                        "pose_mode": pose_mode,
                        "target_frame": "base_link",
                        "tcp_offset_xyz": tcp_offset_xyz,
                        "target_base_offset_xyz": target_base_offset_xyz,
                        "base_z_offset_m": base_z_offset_m,
                        "min_target_z_m": min_target_z_m,
                        "publish_count": 5,
                        "exit_after_publish": False,
                    }
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_visual_grasp_markers",
                name="rebotarm_visual_grasp_markers",
                output="screen",
                condition=IfCondition(start_visual_grasp_markers),
                parameters=[
                    {
                        "input_topic": "/grasp/plan",
                        "output_topic": "/grasp/visual_markers",
                        "target_frame": "base_link",
                        "object_min_diameter_m": 0.06,
                        "object_min_height_m": 0.12,
                        "upright_object_marker": True,
                    }
                ],
            ),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_sim_trajectory_controller",
                name="rebotarm_sim_trajectory_controller",
                output="screen",
                condition=UnlessCondition(use_hardware),
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                    }
                ],
            ),
            Node(
                package="rebotarm_motion_execution",
                executable="rebotarm_motion_execution_node",
                name="rebotarm_motion_execution",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "frame_id": "base_link",
                        "ee_frame_id": "end_link",
                        "default_velocity_scaling": 0.10,
                        "default_acceleration_scaling": 0.08,
                    }
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_visual_grasp_executor",
                name="rebotarm_visual_grasp_executor",
                output="screen",
                condition=IfCondition(start_visual_grasp_executor),
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "input_topic": "/grasp/plan",
                        "target_frame": "base_link",
                        "tcp_offset_xyz": tcp_offset_xyz,
                        "target_base_offset_xyz": target_base_offset_xyz,
                        "pregrasp_base_z_offset_m": base_z_offset_m,
                        "grasp_base_z_offset_m": grasp_base_z_offset_m,
                        "pose_policy": pose_policy,
                        "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                        "base_approach_axis_xyz": base_approach_axis_xyz,
                        "base_pregrasp_distance_m": base_pregrasp_distance_m,
                        "min_grasp_z_m": min_target_z_m,
                        "lift_z_m": lift_z_m,
                        "close_position_m": close_position_m,
                        "close_max_effort": close_max_effort,
                        "open_before_approach": open_before_approach,
                        "auto_gripper_width": auto_gripper_width,
                        "auto_gripper_effort": auto_gripper_effort,
                        "open_clearance_m": open_clearance_m,
                        "close_margin_m": close_margin_m,
                        "min_gripper_effort": min_gripper_effort,
                        "max_gripper_effort": max_gripper_effort,
                        "max_allowed_grasp_width_m": max_allowed_grasp_width_m,
                        "safe_retreat_enabled": safe_retreat_enabled,
                        "safe_retreat_min_lift_z_m": safe_retreat_min_lift_z_m,
                        "safe_retreat_distance_m": safe_retreat_distance_m,
                        "safe_retreat_axis_xyz": safe_retreat_axis_xyz,
                        "safe_home_after_grasp": safe_home_after_grasp,
                        "execute_gripper": True,
                        "execution_mode": execution_mode,
                        "plan_only_stage_pause_sec": plan_only_stage_pause_sec,
                    }
                ],
            ),
        ]
    )
