from __future__ import annotations

import ast
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    if relative == "src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py":
        return "\n".join(
            [
                _read_file(relative),
                _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_page.py"),
                _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_http.py"),
                _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_assets/index.html"),
            ]
        )
    return _read_file(relative)


def _read_file(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def _console_scripts(setup_relative: str) -> set[str]:
    tree = ast.parse(_read(setup_relative))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "setup":
            continue
        for keyword in node.keywords:
            if keyword.arg != "entry_points":
                continue
            entry_points = ast.literal_eval(keyword.value)
            scripts = entry_points.get("console_scripts", [])
            return {script.split("=", 1)[0].strip() for script in scripts}
    raise AssertionError(f"setup() entry_points not found in {setup_relative}")


def test_status_panel_page_is_split_from_ros_node():
    panel_text = _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    page_text = _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_page.py")
    html_text = _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_assets/index.html")
    setup_text = _read_file("src/rebotarm_interactive_control/setup.py")

    assert "from .status_panel_page import HTML_PAGE" in panel_text
    assert "HTML_PAGE = r\"\"\"" not in panel_text
    assert "importlib.resources" in page_text
    assert "status_panel_assets" in page_text
    assert "HTML_PAGE = " in page_text
    assert "HTML_PAGE = r\"\"\"" not in page_text
    assert 'id="robot-view"' in html_text
    assert 'id="arm-safe-home"' in html_text
    assert '"rebotarm_interactive_control.status_panel_assets": ["index.html"]' in setup_text


def test_status_panel_http_server_is_split_from_ros_node():
    panel_text = _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    http_text = _read_file("src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_http.py")

    assert "from .status_panel_http import create_status_panel_server" in panel_text
    assert "BaseHTTPRequestHandler" not in panel_text
    assert "ThreadingHTTPServer" not in panel_text
    assert "def do_POST" not in panel_text
    assert "create_status_panel_server(" in panel_text
    assert "class StatusPanelRequestHandler" in http_text
    assert "dispatch_post_request" in http_text
    assert "encode_sse_event" in http_text


def test_rebotarm_vision_exposes_grasp_console_entrypoints():
    scripts = _console_scripts("src/rebotarm_vision/setup.py")

    assert {
        "rebotarm_ordinary_grasp_node",
        "rebotarm_send_grasp_preview",
        "rebotarm_visual_grasp_executor",
        "rebotarm_visual_grasp_markers",
        "rebotarm_grasp_tcp_frame",
        "rebotarm_grasp_depth_probe",
    }.issubset(scripts)


def test_visual_grasp_system_launch_defaults_to_safe_preview_before_executor():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert (
        'DeclareLaunchArgument("execution_mode", default_value="simulation")' in launch_text
        or 'DeclareLaunchArgument("execution_mode", default_value="plan_only")' in launch_text
    )
    assert 'DeclareLaunchArgument("start_grasp_preview",' in launch_text
    assert (
        'DeclareLaunchArgument("start_visual_grasp_executor", default_value="false")'
        in launch_text
        or 'DeclareLaunchArgument("start_visual_grasp_executor", default_value="true")'
        in launch_text
    )
    assert 'executable="rebotarm_send_grasp_preview"' in launch_text
    assert 'executable="rebotarm_visual_grasp_executor"' in launch_text
    assert '"input_topic": "/grasp/plan"' in launch_text
    assert (
        '"/interactive_control/pose_target"' in launch_text
        or '"motion_execution"'
        in launch_text
    )
    assert '"min_grasp_z_m": min_target_z_m' in launch_text


def test_visual_grasp_system_uses_measured_grasp_tcp_offset_by_default():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]")' in launch_text


def test_visual_grasp_executor_keeps_stop_paths_wired():
    executor_text = _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py")

    assert 'f"/{self._arm_namespace}/visual_grasp/execute"' in executor_text
    assert 'f"/{self._arm_namespace}/visual_grasp/stop"' in executor_text
    assert (
        'f"/{self._arm_namespace}/interactive_control/execute_preview"' in executor_text
        or 'f"/{self._arm_namespace}/motion_execution/execute_pose"' in executor_text
    )
    assert (
        'f"/{self._arm_namespace}/interactive_control/stop"' in executor_text
        or 'f"/{self._arm_namespace}/motion_execution/stop"' in executor_text
    )
    assert 'f"/{self._arm_namespace}/trajectory_stop"' in executor_text
    assert 'self._request_stop_service(self._trajectory_stop_client, "trajectory_stop")' in executor_text


def test_low_level_controller_exports_trajectory_stop_service():
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_services.py")

    assert 'self._service("trajectory_stop")' in controller_text
    assert "def trajectory_stop(self, _request, response):" in controller_text


def test_rebotarm_msgs_exports_grasp_gripper_service():
    cmake_text = _read("src/rebotarm_msgs/CMakeLists.txt")
    srv_text = _read("src/rebotarm_msgs/srv/GraspGripper.srv")

    assert '"srv/GraspGripper.srv"' in cmake_text
    assert "float64 close_force" in srv_text
    assert "float64 hold_force" in srv_text
    assert "bool contact_detected" in srv_text
    assert "float64 contact_position" in srv_text


def test_ordinary_grasp_node_publishes_candidate_array_topic():
    node_text = _read("src/rebotarm_vision/rebotarm_vision/ordinary_grasp_node.py")
    launch_text = _read("src/rebotarm_vision/launch/vision.launch.py")
    camera_config = _read("src/rebotarm_vision/config/camera.yaml")

    assert "GraspCandidateArray" in node_text
    assert 'ordinary_grasp.candidates_topic", "/grasp/candidates"' in node_text
    assert "self.candidates_pub.publish(candidates)" in node_text
    assert "plan_and_candidates_from_detections_and_depth" in node_text
    assert "-p ordinary_grasp.candidates_topic:=/grasp/candidates " in launch_text
    assert "depth_quality.max_depth_m: 1.2" in camera_config
    assert "DepthQualityConfig" in node_text


def test_rebotarm_vision_exposes_candidate_ik_filter_entrypoint():
    scripts = _console_scripts("src/rebotarm_vision/setup.py")

    assert "rebotarm_grasp_candidate_ik_filter" in scripts


def test_visual_grasp_system_launch_wires_candidate_ik_filter():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'DeclareLaunchArgument("start_candidate_ik_filter", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates")' in launch_text
    assert 'DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan")' in launch_text
    assert 'DeclareLaunchArgument("candidate_collision_check_enabled", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity")' in launch_text
    assert 'DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper")' in launch_text
    assert 'executable="rebotarm_grasp_candidate_ik_filter"' in launch_text
    assert '"input_topic": "/grasp/candidates"' in launch_text
    assert '"output_topic": filtered_candidates_topic' in launch_text
    assert '"output_plan_topic": filtered_plan_topic' in launch_text
    assert '"collision_check_enabled": candidate_collision_check_enabled' in launch_text
    assert '"collision_check_service": candidate_collision_check_service' in launch_text
    assert '"collision_group_name": candidate_collision_group_name' in launch_text


def test_visual_grasp_system_disables_extra_fake_joint_state_sources():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    interactive_text = _read("src/rebotarm_bringup/launch/interactive_system.launch.py")

    assert 'DeclareLaunchArgument("start_passive_joint_state_publisher", default_value="true")' in interactive_text
    assert 'DeclareLaunchArgument("use_moveit_fake_joint_states", default_value="true")' in interactive_text
    assert 'start_passive_joint_state_publisher = LaunchConfiguration("start_passive_joint_state_publisher")' in interactive_text
    assert 'use_moveit_fake_joint_states = LaunchConfiguration("use_moveit_fake_joint_states")' in interactive_text
    assert '"use_moveit_fake_joint_states": "false"' in launch_text
    assert '"start_passive_joint_state_publisher": "false"' in launch_text
    assert "condition=IfCondition(" in interactive_text
    assert "PythonExpression(" in interactive_text
    assert 'start_passive_joint_state_publisher' in interactive_text


def test_visual_grasp_executor_consumes_grasp_plan_by_default():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    executor_text = _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py")

    assert 'DeclareLaunchArgument("executor_input_topic", default_value="/grasp/plan")' in launch_text
    assert 'executor_input_topic = LaunchConfiguration("executor_input_topic")' in launch_text
    assert '"input_topic": executor_input_topic' in launch_text
    assert 'if str(getattr(plan, "source", "")).strip() == "candidate_ik_filter":' in executor_text
    assert "return self._build_motion_targets_from_filtered_plan(plan)" in executor_text
    assert "def _build_motion_targets_from_filtered_plan" in executor_text


def test_visual_grasp_executor_refreshes_plan_after_pregrasp():
    executor_text = _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py")
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'self.declare_parameter("input_topic", "/grasp/filtered_plan")' in executor_text
    assert 'self.declare_parameter("refresh_plan_at_pregrasp_enabled", True)' in executor_text
    assert 'self.declare_parameter("refresh_plan_at_pregrasp_required", True)' in executor_text
    assert 'self.declare_parameter("refresh_plan_timeout_sec", 1.0)' in executor_text
    assert "self._plan_revision" in executor_text
    assert "def _wait_for_refreshed_plan" in executor_text
    assert 'stage.name == "move_to_pregrasp"' in executor_text
    assert "fresh grasp plan unavailable after pregrasp" in executor_text
    assert "stages = self._replace_remaining_after_pregrasp" in executor_text
    assert 'DeclareLaunchArgument("refresh_plan_at_pregrasp_enabled"' not in launch_text
    assert 'DeclareLaunchArgument("refresh_plan_at_pregrasp_required"' not in launch_text
    assert '"refresh_plan_at_pregrasp_enabled": False' in launch_text
    assert '"refresh_plan_at_pregrasp_required": False' in launch_text


def test_visual_grasp_executor_has_bounded_approach_visual_servo():
    executor_text = _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py")
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert "from .visual_servo_policy import VisualServoApproachConfig, build_visual_servo_step" in executor_text
    assert 'self.declare_parameter("approach_visual_servo_enabled", False)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_max_iterations", 5)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_max_step_m", 0.02)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_position_tolerance_m", 0.008)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_require_fresh_plan", True)' in executor_text
    assert "def _run_visual_servo_approach" in executor_text
    assert "fresh visual servo plan unavailable" in executor_text
    assert "build_visual_servo_step" in executor_text
    assert 'VisualGraspStage(name="visual_servo_approach"' in executor_text
    assert 'DeclareLaunchArgument("approach_visual_servo_enabled", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("approach_visual_servo_require_fresh_plan", default_value="true")' in launch_text
    assert '"approach_visual_servo_enabled": approach_visual_servo_enabled' in launch_text
    assert '"approach_visual_servo_require_fresh_plan": approach_visual_servo_require_fresh_plan' in launch_text


def test_visual_grasp_executor_wires_retry_verification_place_and_recovery():
    executor_text = _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py")
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert "GraspCandidateArray, GraspPlan" in executor_text
    assert "from .grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices" in executor_text
    assert "from .grasp_verification_policy import" in executor_text
    assert "from .place_task_policy import PlaceTaskConfig, build_place_stages" in executor_text
    assert "from .trajectory_recovery_policy import RecoveryConfig, recovery_decision_for_stage" in executor_text
    assert 'self.declare_parameter("candidates_topic", "/grasp/filtered_candidates")' in executor_text
    assert 'self.declare_parameter("auto_retry_enabled", False)' in executor_text
    assert 'self.declare_parameter("grasp_verification_enabled", True)' in executor_text
    assert 'self.declare_parameter("place_after_grasp_enabled", False)' in executor_text
    assert 'self.declare_parameter("trajectory_precheck_enabled", True)' in executor_text
    assert "def _candidate_plans_for_attempts" in executor_text
    assert "attempts: list[tuple[int, GraspPlan]] = [(-1, deepcopy(self._latest_plan))]" in executor_text
    assert "if index == int(candidates.best_index):" in executor_text
    assert "continue" in executor_text
    assert "def _verify_after_lift" in executor_text
    assert "def _append_place_stages" in executor_text
    assert "def _precheck_execute_pose" in executor_text
    assert 'name="retry_safe_retreat"' in executor_text
    assert "self._retry_retreat_stage = stage" in executor_text
    assert "self._run_stage(retreat)" in executor_text
    assert "recovery_decision_for_stage" in executor_text
    assert 'DeclareLaunchArgument("auto_retry_enabled", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("place_after_grasp_enabled", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("trajectory_precheck_enabled", default_value="false")' in launch_text
    assert '"candidates_topic": filtered_candidates_topic' in launch_text
    assert '"auto_retry_enabled": auto_retry_enabled' in launch_text
    assert '"grasp_verification_enabled": grasp_verification_enabled' in launch_text
    assert '"place_after_grasp_enabled": place_after_grasp_enabled' in launch_text
    assert '"trajectory_precheck_enabled": trajectory_precheck_enabled' in launch_text


def test_candidate_ik_filter_node_uses_moveit_ik_and_state_validity_without_execution():
    node_text = _read("src/rebotarm_vision/rebotarm_vision/candidate_ik_filter_node.py")
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert "GetPositionIK" in node_text
    assert "GetStateValidity" in node_text
    assert "from sensor_msgs.msg import JointState" in node_text
    assert 'self.declare_parameter("moveit_ik_service", "/compute_ik")' in node_text
    assert 'self.declare_parameter("joint_state_topic", "/rebotarm/visual_joint_states")' in node_text
    assert 'self.declare_parameter("collision_check_service", "/check_state_validity")' in node_text
    assert 'self.declare_parameter("collision_group_name", "arm_with_gripper")' in node_text
    assert 'self.declare_parameter("service_timeout_sec", 5.0)' in node_text
    assert 'self.declare_parameter("pose_policy", "hybrid_geometry_with_base_axis_fallback")' in node_text
    assert 'self.declare_parameter("orientation_yaw_offsets_rad",' in node_text
    assert 'self.declare_parameter("candidate_grasp_z_offsets_m",' in node_text
    assert "self._filter_busy = False" in node_text
    assert "candidate IK filter is still processing previous candidates; dropping this frame" in node_text
    assert "self._filter_busy = True" in node_text
    assert "self._filter_busy = False" in node_text
    assert "self._latest_joint_state" in node_text
    assert "def _on_joint_state" in node_text
    assert "def _valid_joint_state" in node_text
    assert "def _candidate_target_variants" in node_text
    assert "def _official_geometry_target_variants" in node_text
    assert "def _hybrid_geometry_target_variants" in node_text
    assert "build_hybrid_geometry_grasp_targets" in node_text
    assert "build_official_geometry_grasp_targets" in node_text
    assert "for pregrasp, grasp, variant_label in self._candidate_target_variants" in node_text
    assert "request.ik_request.robot_state.joint_state = deepcopy(self._latest_joint_state)" in node_text
    assert "request.ik_request.avoid_collisions = False" in node_text
    assert "candidate IK filter IK failed" in node_text
    assert "def _check_state_validity" in node_text
    assert "candidate IK filter state validity failed" in node_text
    assert "filter_candidate_array_by_reachability" in node_text
    assert "plan.pregrasp_pose = _pose_from_target(pregrasp)" in node_text
    assert "plan.grasp_pose = _pose_from_target(grasp)" in node_text
    assert "self._plan_pub.publish" in node_text
    assert "follow_joint_trajectory" not in node_text
    assert "execute_pose" not in node_text
    assert 'DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states")' in launch_text
    assert 'DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0")' in launch_text
    assert 'DeclareLaunchArgument("candidate_pose_policy", default_value="hybrid_geometry_with_base_axis_fallback")' in launch_text
    assert 'DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad",' in launch_text
    assert 'DeclareLaunchArgument("candidate_grasp_z_offsets_m",' in launch_text
    assert '"joint_state_topic": candidate_joint_state_topic' in launch_text
    assert '"service_timeout_sec": candidate_filter_service_timeout_sec' in launch_text
    assert '"pose_policy": candidate_pose_policy' in launch_text
    assert '"orientation_yaw_offsets_rad": candidate_orientation_yaw_offsets_rad' in launch_text
    assert '"candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m' in launch_text


def test_gripper_visual_joint_state_node_rejects_empty_or_incomplete_arm_state():
    node_text = _read(
        "src/rebotarm_interactive_control/rebotarm_interactive_control/gripper_visual_joint_state_node.py"
    )

    assert '"required_arm_joint_names"' in node_text
    assert "self._warned_invalid_arm_state = False" in node_text
    assert "def _valid_arm_joint_state" in node_text
    assert "if not self._valid_arm_joint_state(msg):" in node_text
    assert "ignored empty or incomplete arm joint state" in node_text
    assert "return all(math.isfinite" in node_text


def test_candidate_ik_filter_builds_filtered_plan_from_reachable_targets_without_ros_init():
    import importlib.util

    try:
        moveit_msgs_spec = importlib.util.find_spec("moveit_msgs")
    except ValueError:
        moveit_msgs_spec = None
    if moveit_msgs_spec is None:
        pytest.skip("moveit_msgs is not installed in this Python environment")

    from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray
    from rebotarm_vision.candidate_ik_filter_node import CandidateIkFilterNode
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    filtered = GraspCandidateArray()
    filtered.header.frame_id = "camera_depth_frame"
    filtered.best_index = 0
    candidate = GraspCandidate()
    candidate.class_name = "bottle"
    candidate.jaw_width = 0.04
    filtered.candidates.append(candidate)

    node = object.__new__(CandidateIkFilterNode)
    node._target_frame = "base_link"
    pregrasp = PoseTarget(position=(0.30, 0.00, 0.24), orientation=(0.0, 0.0, 0.0, 1.0))
    grasp = PoseTarget(position=(0.38, 0.00, 0.18), orientation=(0.0, 0.0, 0.0, 1.0))

    plan = CandidateIkFilterNode._plan_from_filtered(node, filtered, [(pregrasp, grasp)])

    assert plan.valid is True
    assert plan.source == "candidate_ik_filter"
    assert plan.header.frame_id == "base_link"
    assert plan.pregrasp_pose.position.z == pytest.approx(0.24)
    assert plan.grasp_pose.position.x == pytest.approx(0.38)


def test_low_level_controller_exports_grasp_gripper_service():
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_services.py")
    hardware_text = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")

    assert "GraspGripper" in controller_text
    assert 'self._service("gripper/grasp")' in controller_text
    assert "def grasp_gripper(self, request, response):" in controller_text
    assert "def grasp_gripper(" in hardware_text
    assert "_gripper_mode = \"grasp_closing\"" in hardware_text
    assert "_gripper_mode = \"grasp_holding\"" in hardware_text
    assert "_G_GRASP_CLOSE_KP = 0.0" in hardware_text
    assert "_G_GRASP_CLOSE_KD = 0.5" in hardware_text
    assert "_G_GRASP_HOLD_KP = 5.0" in hardware_text
    assert "_G_GRASP_HOLD_KD = 1.0" in hardware_text


def test_visual_grasp_executor_uses_grasp_service_for_close_stage():
    executor_text = _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py")

    assert "GraspGripper" in executor_text
    assert 'f"/{self._arm_namespace}/gripper/grasp"' in executor_text
    assert "gripper_grasp_enabled" in executor_text
    assert "def _call_grasp_gripper" in executor_text
    assert 'stage.name == "close_gripper"' in executor_text
    assert 'request.close_force = max(float(self.get_parameter("gripper_grasp_close_force").value), 0.0)' in executor_text
    assert "request.hold_force = max(float(stage.gripper_max_effort), 0.0)" in executor_text


def test_visual_grasp_launch_does_not_expose_hard_object_safety_params():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert "hard_object_safety_enabled" not in launch_text
    assert "hard_object_max_close_force" not in launch_text
    assert "hard_object_max_hold_force" not in launch_text
    assert "hard_object_require_detected_width" not in launch_text


def test_low_level_trajectory_stop_holds_current_position_immediately():
    hardware_text = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")
    stop_body = hardware_text.split("def stop_active_motion(self) -> None:", 1)[1].split("\n    def ", 1)[0]

    assert "self._endpos_ctrl._stop_send.set()" in stop_body
    assert "self.hold_current_position()" in stop_body
    assert 'self.set_state_machine("IDLE")' in stop_body


def test_follow_joint_trajectory_rechecks_stop_before_writing_next_target():
    actions_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_actions.py")
    execute_body = actions_text.split("def execute_follow_joint_trajectory(self, goal_handle):", 1)[1].split("\n    def _set_endpos_target", 1)[0]

    assert "def _trajectory_stopped" in actions_text
    assert "if self._trajectory_stopped(goal_handle, result):" in execute_body
    assert execute_body.index("if self._trajectory_stopped(goal_handle, result):") < execute_body.index("self._set_endpos_target")


def test_follow_joint_trajectory_keeps_running_until_goal_settles():
    actions_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_actions.py")
    execute_body = actions_text.split("def execute_follow_joint_trajectory(self, goal_handle):", 1)[1].split("\n    def _set_endpos_target", 1)[0]

    settle_index = execute_body.index("ok, max_error = self._wait_until_goal_reached")
    success_index = execute_body.index("goal_handle.succeed()")
    idle_index = execute_body.index('self._hardware.set_state_machine("IDLE")')

    assert settle_index < success_index < idle_index


def test_status_panel_stop_replay_falls_back_to_controller_stop():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert "self._trajectory_stop_client = self.create_client(" in panel_text
    assert 'f"/{self._arm_namespace}/trajectory_stop"' in panel_text
    assert "def _request_controller_trajectory_stop" in panel_text
    assert "controller trajectory_stop requested" in panel_text


def test_status_panel_web_stop_always_requests_controller_stop():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    stop_body = panel_text.split("def _handle_stop_execute(self) -> dict:", 1)[1].split("\n    def ", 1)[0]

    assert "stop_requested = self._request_controller_trajectory_stop" in stop_body
    assert "no active web execute goal; controller trajectory_stop requested" in stop_body
    assert "self._execute_goal_handle = None" in stop_body


def test_status_panel_web_execute_settings_are_number_inputs_only():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert 'id="execute-max-delta" type="number"' in panel_text
    assert 'id="execute-duration" type="number"' in panel_text
    assert 'id="execute-speed" type="number"' in panel_text
    assert 'id="execute-max-delta" type="range"' not in panel_text
    assert 'id="execute-duration" type="range"' not in panel_text
    assert 'id="execute-speed" type="range"' not in panel_text
    assert "bindExecuteSetting('execute-max-delta', 'maxDelta'" in panel_text


def test_status_panel_exposes_arm_service_buttons():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert 'id="arm-safe-home"' in panel_text
    assert 'id="arm-enable"' in panel_text
    assert 'id="arm-disable"' in panel_text
    assert 'id="arm-command-state"' in panel_text
    assert 'f"/{self._arm_namespace}/safe_home"' in panel_text
    assert 'f"/{self._arm_namespace}/enable"' in panel_text
    assert 'f"/{self._arm_namespace}/disable"' in panel_text
    assert "def _handle_arm_service_command" in panel_text
    assert panel_text.count("const runArmCommand = async") == 1
    assert "statusObj(data.teleop.arm_command).state" in panel_text
    assert "self._store.update_arm_status(" in panel_text
    assert "if command == \"disable\":" in panel_text
    assert "enabled = False" in panel_text


def test_status_panel_surfaces_gripper_motor_state():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    motor_body = panel_text.split("const updateMotorRows = (joints) => {", 1)[1].split(
        "const previewArmTargets = () => {", 1
    )[0]
    status_body = panel_text.split("source.addEventListener('status', (event) => {", 1)[1].split(
        "const replayStatus =", 1
    )[0]

    assert 'id="gripper-command-state"' in panel_text
    assert "joint7" in motor_body
    assert "电机7 / endjoint / gripper" not in motor_body
    assert "source.gripper = { missing: true }" in motor_body
    assert "motorOrder" in motor_body
    assert "statusObj(data.teleop.web_gripper).state" in status_body


def test_web_execute_returns_to_live_feedback_after_sending_gripper():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    execute_body = panel_text.split("const executePreviewAndGripper = async () => {", 1)[1].split(
        "const runTeachDryRun = async () => {", 1
    )[0]

    assert "const gripperResult = await setGripper({ confirm: false })" in execute_body
    assert "exitPreviewToLive" in execute_body
    assert "const exitPreviewToLive =" in panel_text
    assert "previewState.active = false" in panel_text
    assert "左侧模型和滑条改为跟随实时反馈" in execute_body
    assert "updateRobotViewer(previewState.latestJoints)" in panel_text


def test_gripper_action_aborts_when_target_is_not_reached():
    actions_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_actions.py")
    gripper_body = actions_text.split("def execute_gripper_command(self, goal_handle):", 1)[1].split(
        "\n    def ", 1
    )[0]

    assert "result.reached_goal = self._hardware.gripper_reached_target()" in gripper_body
    assert "if result.reached_goal:" in gripper_body
    assert "goal_handle.succeed()" in gripper_body
    assert "goal_handle.abort()" in gripper_body


def test_arm_service_buttons_are_interlocked_during_replay():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    run_body = panel_text.split("const runArmCommand = async (command, label) => {", 1)[1].split(
        "const bindTeachReplaySetting", 1
    )[0]
    status_body = panel_text.split("source.addEventListener('status', (event) => {", 1)[1].split(
        "attachControlCardToggles();", 1
    )[0]
    backend_body = panel_text.split("def _handle_arm_service_command(self, command: str) -> dict:", 1)[1].split(
        "\n    def _handle_teach_record_start", 1
    )[0]

    assert "isReplayArmCommandLocked()" in run_body
    assert "teach replay is still stopping; arm commands are locked" in run_body
    assert "isReplayArmCommandLocked(replayStatus)" in status_body
    assert "arm_command_is_replay_locked(replay_state)" in backend_body
    assert "arm command blocked during teach replay" in backend_body
    assert "restoreArmButtons" in run_body
    assert "const result = await response.json()" in run_body
    assert "should_stop_trajectory_before_arm_command(command)" in backend_body
    assert "trajectory_stop_requested" in backend_body
    assert "exitPreviewToLive(`${label}" in run_body


def test_safety_stop_allows_operator_recovery_controls():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    lock_body = panel_text.split("const isReplayArmCommandLocked =", 1)[1].split("const addReplayEvent", 1)[0]
    dry_run_button_body = panel_text.split("const updateTeachDryRunButton = (info) => {", 1)[1].split(
        "const refreshTeachFileInfo", 1
    )[0]
    backend_body = panel_text.split("def _handle_arm_service_command(self, command: str) -> dict:", 1)[1].split(
        "\n    def _handle_teach_record_start", 1
    )[0]

    assert "['replaying', 'cancel_requested', 'stop_requested'].includes(state)" in lock_body
    assert "safety_stop" not in lock_body
    assert "['replaying', 'cancel_requested', 'safety_stop'].includes(replayState)" in dry_run_button_body
    assert "arm_command_is_replay_locked(replay_state)" in backend_body
    assert '"safety_stop"' not in backend_body.split("arm_command_is_replay_locked(replay_state)", 1)[0]


def test_status_panel_defaults_cards_collapsed_and_removes_keyboard_sliders():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert 'class="panel collapsible-card collapsed" id="arm-status-card"' in panel_text
    assert 'class="panel slider-panel collapsible-card collapsed" id="web-teleop-card"' in panel_text
    assert 'class="panel collapsible-card collapsed" id="keyboard-teleop-card"' in panel_text
    assert 'class="panel teach-panel collapsible-card collapsed" id="teach-trajectory-card"' in panel_text
    assert 'class="panel collapsible-card collapsed" id="motor-state-card"' in panel_text
    assert 'id="keyboard-step" type="range"' not in panel_text
    assert 'id="keyboard-speed" type="range"' not in panel_text
    assert "arm-command-status" not in panel_text


def test_status_panel_right_card_order_and_simplified_teach_card():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert panel_text.index('id="arm-status-card"') < panel_text.index('id="motor-state-card"')
    assert panel_text.index('id="motor-state-card"') < panel_text.index('id="web-teleop-card"')
    assert panel_text.index('id="web-teleop-card"') < panel_text.index('id="teach-trajectory-card"')
    assert panel_text.index('id="teach-trajectory-card"') < panel_text.index('id="keyboard-teleop-card"')
    assert panel_text.index('id="robot-view"') < panel_text.index('id="arm-safe-home"')
    assert 'id="teach-record-name"' in panel_text
    assert "开始示教" in panel_text
    assert "检查轨迹" in panel_text
    assert ">回放优化轨迹<" in panel_text
    assert panel_text.index("1. 录制") < panel_text.index('id="teach-record-name"')
    assert panel_text.index("2. 检查") < panel_text.index('id="teach-record-select"')
    assert '选择已录制文件' in panel_text
    assert '<details class="teach-step" id="teach-record-step">' in panel_text
    assert '<details class="teach-step" id="teach-check-step">' in panel_text
    assert '<details class="teach-step" id="teach-replay-step">' in panel_text
    assert '<details class="teach-step" id="teach-check-step" open>' not in panel_text
    assert '<details class="teach-step" id="teach-replay-step" open>' not in panel_text
    assert '<option value="" disabled selected>选择已录制文件</option>' in panel_text
    assert "#teach-record-select:invalid" in panel_text
    assert 'Default file' not in panel_text
    assert 'id="teach-replay-speed" type="range" min="0.1" max="1.0"' in panel_text
    assert 'id="teach-align-duration"' not in panel_text
    assert 'id="teach-final-hold"' not in panel_text
    assert "Final Hold', '1.0 s'" not in panel_text
    assert "自动对齐时间" in panel_text
    replay_params_body = panel_text.split("const renderReplayParams = (info) => {", 1)[1].split("const addReplayEvent", 1)[0]
    assert "回放倍率" in replay_params_body
    assert "预计时长" in replay_params_body
    assert "轨迹点" in replay_params_body
    assert "Hardware Mode" not in replay_params_body
    assert "Panel Mode" not in replay_params_body
    assert "Direct Threshold" not in replay_params_body
    assert "Align Threshold" not in replay_params_body
    assert "Execution Gate" not in replay_params_body
    assert "Final Hold" not in replay_params_body
    assert "轨迹异常点" in panel_text
    assert "不平滑点详情" not in panel_text
    assert "安全重定时后可回放" in panel_text
    assert "Teach JSONL Schema" not in panel_text
    assert "Replay Checklist Details" not in panel_text
    assert "Recording / Gravity" not in panel_text
    assert "Default record_path" not in panel_text
    assert 'id="expand-teach-trajectory"' not in panel_text
    assert "toggleTeachTrajectoryChart" not in panel_text
    assert 'id="teach-trajectory-frame"' not in panel_text
    assert "previewTeachTrajectoryFrame" not in panel_text
    assert "Trajectory Limits" not in panel_text
    file_info_body = panel_text.split("const renderTeachFileInfo = (info) => {", 1)[1].split(
        "const renderTeachRecordSummary", 1
    )[0]
    assert "Playback Quality" not in file_info_body
    assert "Trajectory Risk" not in file_info_body
    assert "轨迹风险" not in file_info_body
    assert "Direct Threshold" not in file_info_body
    assert "Align Threshold" not in file_info_body
    assert "Max Jump" not in file_info_body
    assert "Max Velocity" not in file_info_body
    precheck_body = panel_text.split("const renderReplayPrecheckSummary = (info) => {", 1)[1].split("const replayEstimate", 1)[0]
    assert "回放质量" in precheck_body
    assert "Raw Risk" not in precheck_body
    assert "优化跳变" in precheck_body
    assert "优化速度" in precheck_body
    assert "实际倍率" in precheck_body
    assert "请求倍率" in precheck_body
    assert "大范围轨迹" in precheck_body
    assert "优化加速度" in precheck_body
    assert "优化加加速度" in precheck_body
    assert "跟踪保护" in precheck_body
    assert "large_motion?.effective_speed" in precheck_body
    assert "large_motion?.enabled" in precheck_body
    assert "lastTeachDryRun?.prepared_replay?.after_quality" in precheck_body
    assert "lastTeachDryRun?.effective_risk_level" in precheck_body
    trajectory_body = panel_text.split("const renderTeachTrajectoryDetails = (payload) => {", 1)[1].split("const drawTeachTrajectoryChart", 1)[0]
    assert "Prepared Risk" not in trajectory_body
    assert "Max Jump" not in trajectory_body
    assert "Max Velocity" not in trajectory_body
    assert "effective_risk_level" in precheck_body
    assert "prepared_risk_level" in precheck_body
    assert "prepared_replay?.after_quality" in precheck_body
    assert "teachMetric('Risk'" not in precheck_body
    assert "prepared_record_path" in panel_text
    assert "preparedPoints.length" not in panel_text
    assert "检查结果有效，可以发送优化后的真实回放轨迹。" in panel_text
    assert "await refreshTeachFileInfo({ force: true })" in panel_text
    assert "setHtml('replay-precheck-summary', renderReplayPrecheckSummary(latestTeachFileInfo));" in panel_text


def test_teach_replay_dry_run_token_survives_live_start_error_drift():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    button_body = panel_text.split("const updateTeachDryRunButton = (info) => {", 1)[1].split(
        "const refreshTeachFileInfo", 1
    )[0]
    execute_body = panel_text.split("def _handle_teach_replay_execute(self, payload: dict) -> dict:", 1)[1].split(
        "\n        decision = validate_teach_replay_execute_request", 1
    )[0]

    assert "lastTeachDryRun?.record_path === info?.path" in button_body
    assert "lastTeachDryRun?.accepted === true" in button_body
    assert "dryRunErrorMatches" not in button_body
    assert "lastTeachDryRun?.max_error" not in button_body

    assert 'str(token.get("record_path", "")) == str(info_payload.get("path", ""))' in execute_body
    assert 'token.get("settings") == settings' in execute_body
    assert "error_matches" not in execute_body
    assert 'token.get("worst_joint"' not in execute_body
    assert 'token.get("risk_level"' not in execute_body


def test_status_panel_throttles_heavy_browser_rendering():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert "FAST_RENDER_INTERVAL_MS" in panel_text
    assert "const shouldRenderFastPanels = nowMs - lastFastRenderMs > FAST_RENDER_INTERVAL_MS;" in panel_text
    assert "if (!shouldRenderFastPanels) return;" in panel_text
    assert "scheduleRobotRender" in panel_text
    assert "requestAnimationFrame(renderRobotFrame)" in panel_text
    assert "setInterval(refreshTeachFileInfo, 5000)" in panel_text


def test_status_panel_unloads_collapsed_details_and_reuses_motor_rows():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert "isDetailsOpen(" in panel_text
    assert "attachDetailsUnloaders()" in panel_text
    assert "renderOptionalDetails(latestStatusData)" in panel_text
    assert "clearOptionalDetails" in panel_text
    assert "updateMotorRows(data.joints || {})" in panel_text
    assert 'document.getElementById("joints").innerHTML = rows' not in panel_text
    assert "motorRowsByName" in panel_text


def test_status_panel_control_cards_can_collapse_to_headers():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert "collapsible-card" in panel_text
    assert "card-body" in panel_text
    assert "toggleControlCard" in panel_text
    assert "attachControlCardToggles()" in panel_text
    assert 'data-card-toggle="web-teleop-card"' in panel_text
    assert 'data-card-toggle="teach-trajectory-card"' in panel_text
    assert ".collapsible-card.collapsed .card-body" in panel_text


def test_status_panel_teach_info_accepts_record_path_alias_and_skips_collapsed_polling():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    teach_info_body = panel_text.split('if route == "/api/teach_record_info":', 1)[1].split('if route == "/api/teach_records":', 1)[0]
    refresh_body = panel_text.split("const refreshTeachFileInfo = async", 1)[1].split("const refreshTeachRecords", 1)[0]

    assert 'query.get("path", query.get("record_path", [""]))[0]' in teach_info_body
    assert "isControlCardOpen('teach-trajectory-card')" in refresh_body
    assert "refreshTeachFileInfo({ force: true })" in panel_text


def test_status_panel_compacts_large_teach_replay_payloads_for_sse():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert "def _compact_quality_payload" in panel_text
    assert "events_total" in panel_text
    assert "events_truncated" in panel_text
    assert "anomalies_total" in panel_text
    assert "result = self._compact_replay_payload(result)" in panel_text
    assert "self._store.update_teleop_status(\"replay\", result)" in panel_text


def test_status_panel_check_mode_is_read_only_for_teach_actions():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert 'self.declare_parameter("panel_mode", "control")' in panel_text
    assert '"panel_mode": str(self.get_parameter("panel_mode").value)' in panel_text
    assert "const isCheckMode = panelMode === 'check';" in panel_text
    assert "检查模式会自动 dry-run" in panel_text
    assert "检查模式只读" in panel_text


def test_status_panel_uses_workbench_cards_for_teleop_ui():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert "teleop-workbench" in panel_text
    assert "robot-workspace" in panel_text
    assert "control-cards" in panel_text
    assert "arm-status-card" in panel_text
    assert "web-teleop-card" in panel_text
    assert "keyboard-teleop-card" in panel_text
    assert "teach-trajectory-card" in panel_text
    assert "motor-state-card" in panel_text
    assert "无按键输入" in panel_text
    assert "暂无有效示教轨迹" in panel_text
    assert "执行关节 + 夹爪" in panel_text
    assert "/api/keyboard_enable" in panel_text
    assert "/api/keyboard_disable" in panel_text
    assert "/api/keyboard_key" in panel_text
    assert "KEYBOARD_TELEOP" in panel_text
    assert "/api/teach_record_start" in panel_text
    assert "/api/teach_record_stop" in panel_text
    assert 'id="show-live-sliders"' not in panel_text
    assert 'id="live-slider-pane"' not in panel_text
    assert ">Set Gripper<" not in panel_text


def test_teach_recorder_exposes_service_controlled_start_stop():
    recorder_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teach_recorder_node.py")
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")
    controller_recorder_text = _read("src/rebotarmcontroller/rebotarmcontroller/teach_recorder.py")
    teleop_launch_text = _read("src/rebotarm_bringup/launch/teleop_system.launch.py")
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    cmake_text = _read("src/rebotarm_msgs/CMakeLists.txt")

    assert 'self.declare_parameter("start_on_launch", True)' in recorder_text
    assert "self.create_service(" in recorder_text
    assert 'f"/{self._arm_namespace}/teleop/teach_record/start"' in recorder_text
    assert 'f"/{self._arm_namespace}/teleop/teach_record/stop"' in recorder_text
    assert 'f"/{self._arm_namespace}/teleop/teach_record/set_path"' in recorder_text
    assert "SetTeachRecordPath" in recorder_text
    assert "SetTeachRecordPath" in panel_text
    assert 'body: JSON.stringify({ record_path: recordName })' in panel_text
    assert 'return Path("teleop_records") / name' in recorder_text
    assert '"srv/SetTeachRecordPath.srv"' in cmake_text
    assert "def _handle_start_recording" in recorder_text
    assert "def _handle_stop_recording" in recorder_text
    assert "InternalTeachRecorder" in controller_text
    assert 'self.declare_parameter("teach_record_rate_hz", 150.0)' in controller_text
    assert 'f"/{namespace}/teleop/teach_record/start"' in controller_recorder_text
    assert 'f"/{namespace}/teleop/teach_record/stop"' in controller_recorder_text
    assert 'f"/{namespace}/teleop/teach_record/set_path"' in controller_recorder_text
    assert 'f"/{namespace}/teleop/recording_status"' in controller_recorder_text
    assert "hardware.get_joint_state()" in controller_recorder_text
    assert "hardware.get_joint_status_codes()" in controller_recorder_text
    assert '"start_on_launch": False' in teleop_launch_text
    assert "UnlessCondition(use_hardware)" in teleop_launch_text


def test_teach_replay_prepared_pipeline_defaults_to_150hz():
    replay_launch_text = _read("src/rebotarm_bringup/launch/teach_replay.launch.py")
    replay_node_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teach_replay_node.py")
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    profiles_text = _read("src/rebotarm_bringup/config/replay_profiles.yaml")

    assert 'DeclareLaunchArgument("filter_sample_rate_hz", default_value="150.0")' in replay_launch_text
    assert 'DeclareLaunchArgument("resample_rate_hz", default_value="150.0")' in replay_launch_text
    assert 'self.declare_parameter("filter_sample_rate_hz", 150.0)' in replay_node_text
    assert 'self.declare_parameter("resample_rate_hz", 150.0)' in replay_node_text
    assert 'self.declare_parameter("filter_sample_rate_hz", 150.0)' in panel_text
    assert 'self.declare_parameter("resample_rate_hz", 150.0)' in panel_text
    assert "filter_sample_rate_hz: 150.0" in profiles_text
    assert "resample_rate_hz: 150.0" in profiles_text
    assert "time_parameterization_method: auto" in profiles_text
    assert 'self.declare_parameter("time_parameterization_method", "auto")' in replay_node_text
    assert 'self.declare_parameter("time_parameterization_method", "auto")' in panel_text


def test_moveit_ompl_uses_ruckig_response_adapter_with_jerk_limits():
    ompl_text = _read("src/rebotarm_moveit_config/config/ompl_planning.yaml")
    joint_limits_text = _read("src/rebotarm_moveit_config/config/joint_limits.yaml")
    assert "default_planning_response_adapters/AddTimeOptimalParameterization" in ompl_text
    assert "default_planning_response_adapters/AddRuckigTrajectorySmoothing" in ompl_text
    assert ompl_text.index("default_planning_response_adapters/AddRuckigTrajectorySmoothing") < ompl_text.index(
        "default_planning_response_adapters/AddTimeOptimalParameterization"
    )
    assert "has_jerk_limits: true" in joint_limits_text
    assert "max_jerk: 20.0" in joint_limits_text


def test_teach_replay_executes_prepared_retimed_points_directly():
    replay_node_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teach_replay_node.py")

    assert "def _append_prepared_replay_points(" in replay_node_text
    assert "for retimed in self._prepared_replay.retimed_points:" in replay_node_text
    assert "self._append_prepared_replay_points(trajectory, elapsed=elapsed)" in replay_node_text


def test_teach_replay_has_runtime_tracking_guard_for_cli_and_web():
    replay_node_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teach_replay_node.py")
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    config_text = _read("src/rebotarm_interactive_control/config/teleop_control.yaml")

    for text in (replay_node_text, panel_text):
        assert "evaluate_replay_tracking" in text
        assert 'self.declare_parameter("replay_monitor_enabled", True)' in text
        assert 'self.declare_parameter("max_tracking_error_rad", 0.25)' in text
        assert 'self.declare_parameter("max_live_velocity_rad_s", 3.0)' in text
        assert "def _check_active_replay_tracking" in text
        assert "self._request_controller_trajectory_stop" in text
        assert "tracking_error" in text
        assert "live_velocity" in text

    assert "replay_monitor_enabled: true" in config_text
    assert "max_tracking_error_rad: 0.25" in config_text
    assert "max_live_velocity_rad_s: 3.0" in config_text


def test_status_panel_preserves_runtime_safety_stop_result_reason():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    result_body = panel_text.split("def _on_teach_replay_result", 1)[1].split(
        "\n    def _check_active_replay_tracking", 1
    )[0]

    assert "previous_replay = self._store.snapshot().teleop.get(\"replay\", {})" in result_body
    assert "self._teach_replay_monitor_stop_requested" in result_body
    assert "state = \"safety_stop\"" in result_body
    assert "action canceled after runtime monitor stop" in result_body


def test_status_panel_surfaces_time_parameterization_summary():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")

    assert "时间参数化" in panel_text
    assert "time_parameterization?.used_method" in panel_text
    assert "MoveIt 对齐" in panel_text


def test_teach_trajectory_curve_card_shows_prepared_curve_without_duplicate_check_metrics():
    panel_text = _read("src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py")
    details_body = panel_text.split("const renderTeachTrajectoryDetails = (payload) => {", 1)[1].split(
        "const drawTeachTrajectoryChart = (payload) => {", 1
    )[0]
    backend_body = panel_text.split("def _teach_trajectory(", 1)[1].split("\n    def _teach_records", 1)[0]

    assert "曲线来源" in details_body
    assert "优化轨迹点" in details_body
    assert "原始文件" in details_body
    assert "Prepared Risk" not in details_body
    assert "Max Jump" not in details_body
    assert "Max Velocity" not in details_body
    assert 'payload["curve_source"] = "prepared"' in backend_body
    assert "preview_samples = load_teach_samples(prepared_path)" in backend_body


def test_moveit_demo_standalone_publishes_fake_visual_joint_state_source():
    demo_text = _read("src/rebotarm_moveit_config/launch/demo.launch.py")
    hardware_text = _read("src/rebotarm_bringup/launch/moveit_hardware.launch.py")
    interactive_text = _read("src/rebotarm_bringup/launch/interactive_system.launch.py")

    assert 'DeclareLaunchArgument("use_fake_joint_states", default_value="true")' in demo_text
    assert 'executable="joint_state_publisher"' in demo_text
    assert 'condition=IfCondition(use_fake_joint_states)' in demo_text
    assert '"/joint_states", ["/", arm_namespace, "/joint_states"]' in demo_text
    assert '"use_fake_joint_states": "false"' in hardware_text
    assert '"use_fake_joint_states": PythonExpression' in interactive_text
    assert 'use_moveit_fake_joint_states' in interactive_text


def test_controller_shutdown_runs_safe_home_before_disable_by_default():
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")
    hardware_text = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")

    assert 'self.declare_parameter("shutdown_safe_home", True)' in controller_text
    assert "and self.hardware.enabled" in controller_text
    assert "self.hardware.endpos_ctrl.safe_home()" in controller_text
    assert "self.hardware.shutdown()" in controller_text
    assert "def connected(self) -> bool:" in hardware_text
