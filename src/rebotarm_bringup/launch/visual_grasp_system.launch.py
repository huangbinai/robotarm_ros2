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
    ordinary_depth_quality_enabled = LaunchConfiguration("ordinary_depth_quality_enabled")
    start_grasp_preview = LaunchConfiguration("start_grasp_preview")
    start_candidate_ik_filter = LaunchConfiguration("start_candidate_ik_filter")
    move_to_visual_ready_on_start = LaunchConfiguration("move_to_visual_ready_on_start")
    visual_ready_joint_positions = LaunchConfiguration("visual_ready_joint_positions")
    visual_ready_duration_sec = LaunchConfiguration("visual_ready_duration_sec")
    visual_ready_wait_timeout_sec = LaunchConfiguration("visual_ready_wait_timeout_sec")
    visual_ready_max_start_delta_rad = LaunchConfiguration("visual_ready_max_start_delta_rad")
    start_visual_grasp_executor = LaunchConfiguration("start_visual_grasp_executor")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")
    filtered_candidates_topic = LaunchConfiguration("filtered_candidates_topic")
    filtered_plan_topic = LaunchConfiguration("filtered_plan_topic")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    candidate_joint_state_topic = LaunchConfiguration("candidate_joint_state_topic")
    candidate_filter_service_timeout_sec = LaunchConfiguration("candidate_filter_service_timeout_sec")
    candidate_collision_check_enabled = LaunchConfiguration("candidate_collision_check_enabled")
    candidate_collision_check_service = LaunchConfiguration("candidate_collision_check_service")
    candidate_collision_group_name = LaunchConfiguration("candidate_collision_group_name")
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
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    candidate_orientation_yaw_offsets_rad = LaunchConfiguration("candidate_orientation_yaw_offsets_rad")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
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
    gripper_grasp_enabled = LaunchConfiguration("gripper_grasp_enabled")
    gripper_grasp_close_force = LaunchConfiguration("gripper_grasp_close_force")
    gripper_grasp_timeout_sec = LaunchConfiguration("gripper_grasp_timeout_sec")
    gripper_grasp_min_close_time_sec = LaunchConfiguration("gripper_grasp_min_close_time_sec")
    gripper_grasp_velocity_threshold = LaunchConfiguration("gripper_grasp_velocity_threshold")
    gripper_grasp_min_closure_distance_m = LaunchConfiguration("gripper_grasp_min_closure_distance_m")
    safe_retreat_enabled = LaunchConfiguration("safe_retreat_enabled")
    safe_retreat_min_lift_z_m = LaunchConfiguration("safe_retreat_min_lift_z_m")
    safe_retreat_distance_m = LaunchConfiguration("safe_retreat_distance_m")
    safe_retreat_axis_xyz = LaunchConfiguration("safe_retreat_axis_xyz")
    safe_home_after_grasp = LaunchConfiguration("safe_home_after_grasp")
    plan_only_stage_pause_sec = LaunchConfiguration("plan_only_stage_pause_sec")
    approach_visual_servo_enabled = LaunchConfiguration("approach_visual_servo_enabled")
    approach_visual_servo_max_iterations = LaunchConfiguration("approach_visual_servo_max_iterations")
    approach_visual_servo_max_step_m = LaunchConfiguration("approach_visual_servo_max_step_m")
    approach_visual_servo_position_tolerance_m = LaunchConfiguration("approach_visual_servo_position_tolerance_m")
    approach_visual_servo_require_fresh_plan = LaunchConfiguration("approach_visual_servo_require_fresh_plan")
    auto_retry_enabled = LaunchConfiguration("auto_retry_enabled")
    auto_retry_max_attempts = LaunchConfiguration("auto_retry_max_attempts")
    safe_retreat_before_retry = LaunchConfiguration("safe_retreat_before_retry")
    grasp_verification_enabled = LaunchConfiguration("grasp_verification_enabled")
    grasp_verification_min_closure_distance_m = LaunchConfiguration("grasp_verification_min_closure_distance_m")
    grasp_verification_require_contact = LaunchConfiguration("grasp_verification_require_contact")
    visual_lift_check_enabled = LaunchConfiguration("visual_lift_check_enabled")
    visual_lift_min_delta_m = LaunchConfiguration("visual_lift_min_delta_m")
    place_after_grasp_enabled = LaunchConfiguration("place_after_grasp_enabled")
    place_position_xyz = LaunchConfiguration("place_position_xyz")
    place_orientation_xyzw = LaunchConfiguration("place_orientation_xyzw")
    place_open_position_m = LaunchConfiguration("place_open_position_m")
    place_open_max_effort = LaunchConfiguration("place_open_max_effort")
    place_retreat_z_m = LaunchConfiguration("place_retreat_z_m")
    trajectory_precheck_enabled = LaunchConfiguration("trajectory_precheck_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_hardware", default_value="true"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="plan_only"),
            DeclareLaunchArgument("start_vision", default_value="true"),
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            DeclareLaunchArgument("start_grasp_preview", default_value="false"),
            DeclareLaunchArgument("start_candidate_ik_filter", default_value="false"),
            DeclareLaunchArgument("move_to_visual_ready_on_start", default_value="true"),
            DeclareLaunchArgument("visual_ready_joint_positions", default_value="[0.0, 0.0, -0.20, 0.20, 0.0, 0.0]"),
            DeclareLaunchArgument("visual_ready_duration_sec", default_value="4.0"),
            DeclareLaunchArgument("visual_ready_wait_timeout_sec", default_value="12.0"),
            DeclareLaunchArgument("visual_ready_max_start_delta_rad", default_value="1.0"),
            DeclareLaunchArgument("start_visual_grasp_executor", default_value="true"),
            DeclareLaunchArgument("start_visual_grasp_markers", default_value="true"),
            DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates"),
            DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan"),
            DeclareLaunchArgument("executor_input_topic", default_value="/grasp/plan"),
            DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states"),
            DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("candidate_collision_check_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity"),
            DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper"),
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
            DeclareLaunchArgument("candidate_pose_policy", default_value="hybrid_geometry_with_base_axis_fallback"),
            DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad", default_value="[0.0, 3.141592653589793]"),
            DeclareLaunchArgument("candidate_grasp_z_offsets_m", default_value="[0.0, 0.03]"),
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
            DeclareLaunchArgument("gripper_grasp_enabled", default_value="true"),
            DeclareLaunchArgument("gripper_grasp_close_force", default_value="0.6"),
            DeclareLaunchArgument("gripper_grasp_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("gripper_grasp_min_close_time_sec", default_value="0.08"),
            DeclareLaunchArgument("gripper_grasp_velocity_threshold", default_value="0.04"),
            DeclareLaunchArgument("gripper_grasp_min_closure_distance_m", default_value="0.006"),
            DeclareLaunchArgument("safe_retreat_enabled", default_value="true"),
            DeclareLaunchArgument("safe_retreat_min_lift_z_m", default_value="0.24"),
            DeclareLaunchArgument("safe_retreat_distance_m", default_value="0.06"),
            DeclareLaunchArgument("safe_retreat_axis_xyz", default_value="[-1.0, 0.0, 0.0]"),
            DeclareLaunchArgument("safe_home_after_grasp", default_value="false"),
            DeclareLaunchArgument("plan_only_stage_pause_sec", default_value="3.0"),
            DeclareLaunchArgument("approach_visual_servo_enabled", default_value="false"),
            DeclareLaunchArgument("approach_visual_servo_max_iterations", default_value="5"),
            DeclareLaunchArgument("approach_visual_servo_max_step_m", default_value="0.02"),
            DeclareLaunchArgument("approach_visual_servo_position_tolerance_m", default_value="0.008"),
            DeclareLaunchArgument("approach_visual_servo_require_fresh_plan", default_value="true"),
            DeclareLaunchArgument("auto_retry_enabled", default_value="false"),
            DeclareLaunchArgument("auto_retry_max_attempts", default_value="3"),
            DeclareLaunchArgument("safe_retreat_before_retry", default_value="true"),
            DeclareLaunchArgument("grasp_verification_enabled", default_value="true"),
            DeclareLaunchArgument("grasp_verification_min_closure_distance_m", default_value="0.006"),
            DeclareLaunchArgument("grasp_verification_require_contact", default_value="true"),
            DeclareLaunchArgument("visual_lift_check_enabled", default_value="false"),
            DeclareLaunchArgument("visual_lift_min_delta_m", default_value="0.03"),
            DeclareLaunchArgument("place_after_grasp_enabled", default_value="false"),
            DeclareLaunchArgument("place_position_xyz", default_value="[0.20, -0.20, 0.25]"),
            DeclareLaunchArgument("place_orientation_xyzw", default_value="[0.0, 0.0, 0.0, 1.0]"),
            DeclareLaunchArgument("place_open_position_m", default_value="0.08"),
            DeclareLaunchArgument("place_open_max_effort", default_value="0.25"),
            DeclareLaunchArgument("place_retreat_z_m", default_value="0.06"),
            DeclareLaunchArgument("trajectory_precheck_enabled", default_value="false"),
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
                    "use_moveit_fake_joint_states": "false",
                    "rviz_config": PathJoinSubstitution([bringup_share, "rviz", "interactive_system.rviz"]),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])
                ),
                condition=IfCondition(start_vision),
                launch_arguments={
                    "ordinary_depth_quality_enabled": ordinary_depth_quality_enabled,
                }.items(),
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_visual_ready",
                name="rebotarm_visual_ready",
                output="screen",
                condition=IfCondition(move_to_visual_ready_on_start),
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "joint_positions": visual_ready_joint_positions,
                        "duration_sec": visual_ready_duration_sec,
                        "wait_timeout_sec": visual_ready_wait_timeout_sec,
                        "max_start_delta_rad": visual_ready_max_start_delta_rad,
                    }
                ],
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
                package="rebotarm_vision",
                executable="rebotarm_grasp_candidate_ik_filter",
                name="rebotarm_grasp_candidate_ik_filter",
                output="screen",
                condition=IfCondition(start_candidate_ik_filter),
                parameters=[
                    {
                        "input_topic": "/grasp/candidates",
                        "output_topic": filtered_candidates_topic,
                        "output_plan_topic": filtered_plan_topic,
                        "joint_state_topic": candidate_joint_state_topic,
                        "service_timeout_sec": candidate_filter_service_timeout_sec,
                        "target_frame": "base_link",
                        "moveit_ik_service": "/compute_ik",
                        "collision_check_enabled": candidate_collision_check_enabled,
                        "collision_check_service": candidate_collision_check_service,
                        "collision_group_name": candidate_collision_group_name,
                        "moveit_group_name": "arm",
                        "ee_frame_id": "end_link",
                        "pose_policy": candidate_pose_policy,
                        "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                        "base_approach_axis_xyz": base_approach_axis_xyz,
                        "base_pregrasp_distance_m": base_pregrasp_distance_m,
                        "orientation_yaw_offsets_rad": candidate_orientation_yaw_offsets_rad,
                        "candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m,
                        "tcp_offset_xyz": tcp_offset_xyz,
                        "target_base_offset_xyz": target_base_offset_xyz,
                        "pregrasp_base_z_offset_m": base_z_offset_m,
                        "grasp_base_z_offset_m": grasp_base_z_offset_m,
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
                        "input_topic": executor_input_topic,
                        "candidates_topic": filtered_candidates_topic,
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
                        "gripper_grasp_enabled": gripper_grasp_enabled,
                        "gripper_grasp_close_force": gripper_grasp_close_force,
                        "gripper_grasp_timeout_sec": gripper_grasp_timeout_sec,
                        "gripper_grasp_min_close_time_sec": gripper_grasp_min_close_time_sec,
                        "gripper_grasp_velocity_threshold": gripper_grasp_velocity_threshold,
                        "gripper_grasp_min_closure_distance_m": gripper_grasp_min_closure_distance_m,
                        "safe_retreat_enabled": safe_retreat_enabled,
                        "safe_retreat_min_lift_z_m": safe_retreat_min_lift_z_m,
                        "safe_retreat_distance_m": safe_retreat_distance_m,
                        "safe_retreat_axis_xyz": safe_retreat_axis_xyz,
                        "safe_home_after_grasp": safe_home_after_grasp,
                        "execute_gripper": True,
                        "execution_mode": execution_mode,
                        "plan_only_stage_pause_sec": plan_only_stage_pause_sec,
                        "refresh_plan_at_pregrasp_enabled": False,
                        "refresh_plan_at_pregrasp_required": False,
                        "refresh_plan_timeout_sec": 1.0,
                        "approach_visual_servo_enabled": approach_visual_servo_enabled,
                        "approach_visual_servo_max_iterations": approach_visual_servo_max_iterations,
                        "approach_visual_servo_max_step_m": approach_visual_servo_max_step_m,
                        "approach_visual_servo_position_tolerance_m": approach_visual_servo_position_tolerance_m,
                        "approach_visual_servo_require_fresh_plan": approach_visual_servo_require_fresh_plan,
                        "auto_retry_enabled": auto_retry_enabled,
                        "auto_retry_max_attempts": auto_retry_max_attempts,
                        "safe_retreat_before_retry": safe_retreat_before_retry,
                        "grasp_verification_enabled": grasp_verification_enabled,
                        "grasp_verification_min_closure_distance_m": grasp_verification_min_closure_distance_m,
                        "grasp_verification_require_contact": grasp_verification_require_contact,
                        "visual_lift_check_enabled": visual_lift_check_enabled,
                        "visual_lift_min_delta_m": visual_lift_min_delta_m,
                        "place_after_grasp_enabled": place_after_grasp_enabled,
                        "place_position_xyz": place_position_xyz,
                        "place_orientation_xyzw": place_orientation_xyzw,
                        "place_open_position_m": place_open_position_m,
                        "place_open_max_effort": place_open_max_effort,
                        "place_retreat_z_m": place_retreat_z_m,
                        "trajectory_precheck_enabled": trajectory_precheck_enabled,
                    }
                ],
            ),
        ]
    )
