from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
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


def test_rebotarm_vision_exposes_grasp_console_entrypoints():
    scripts = _console_scripts("src/rebotarm_vision/setup.py")

    assert {
        "rebotarm_ordinary_grasp_node",
        "rebotarm_send_grasp_preview",
        "rebotarm_visual_grasp_executor",
        "rebotarm_grasp_tcp_frame",
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
    assert '"/api/arm_safe_home"' in panel_text
    assert '"/api/arm_enable"' in panel_text
    assert '"/api/arm_disable"' in panel_text
    assert 'f"/{self._arm_namespace}/safe_home"' in panel_text
    assert 'f"/{self._arm_namespace}/enable"' in panel_text
    assert 'f"/{self._arm_namespace}/disable"' in panel_text
    assert "def _handle_arm_service_command" in panel_text
    assert "self._store.update_arm_status(" in panel_text
    assert "if command == \"disable\":" in panel_text
    assert "enabled = False" in panel_text


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
    assert "Start Teach" in panel_text
    assert "Check Trajectory" in panel_text
    assert ">Replay Prepared<" in panel_text
    assert panel_text.index("1. Record") < panel_text.index('id="teach-record-name"')
    assert panel_text.index("2. Check") < panel_text.index('id="teach-record-select"')
    assert 'Choose recorded file' in panel_text
    assert '<details class="teach-step" id="teach-record-step">' in panel_text
    assert '<details class="teach-step" id="teach-check-step" open>' in panel_text
    assert '<details class="teach-step" id="teach-replay-step" open>' in panel_text
    assert '<option value="" disabled selected>Choose recorded file</option>' in panel_text
    assert "#teach-record-select:invalid" in panel_text
    assert 'Default file' not in panel_text
    assert 'id="teach-replay-speed" type="range" min="0.1" max="1.5"' in panel_text
    assert 'id="teach-align-duration"' not in panel_text
    assert 'id="teach-final-hold"' not in panel_text
    assert "Align Time" in panel_text
    assert "Final Hold', '1.0 s'" not in panel_text
    assert "Auto Align Time" in panel_text
    replay_params_body = panel_text.split("const renderReplayParams = (info) => {", 1)[1].split("const addReplayEvent", 1)[0]
    assert "Replay Speed" in replay_params_body
    assert "Estimated Duration" in replay_params_body
    assert "Trajectory Points" in replay_params_body
    assert "Hardware Mode" not in replay_params_body
    assert "Panel Mode" not in replay_params_body
    assert "Direct Threshold" not in replay_params_body
    assert "Align Threshold" not in replay_params_body
    assert "Execution Gate" not in replay_params_body
    assert "Final Hold" not in replay_params_body
    assert "轨迹不平滑点" in panel_text
    assert "不平滑点详情" not in panel_text
    assert "自动限速平滑后回放" in panel_text
    assert "Teach JSONL Schema" not in panel_text
    assert "Replay Checklist Details" not in panel_text
    assert "Recording / Gravity" not in panel_text
    assert "Default record_path" not in panel_text
    assert 'id="expand-teach-trajectory"' not in panel_text
    assert "toggleTeachTrajectoryChart" not in panel_text
    assert 'id="teach-trajectory-frame"' not in panel_text
    assert "previewTeachTrajectoryFrame" not in panel_text
    assert "Trajectory Limits" not in panel_text
    assert "Prepared Risk" in panel_text
    precheck_body = panel_text.split("const renderReplayPrecheckSummary = (info) => {", 1)[1].split("const replayEstimate", 1)[0]
    assert "Playback Quality" in precheck_body
    assert "Raw Risk" not in precheck_body
    assert "Prepared Jump" in precheck_body
    assert "Prepared Velocity" in precheck_body
    assert "effective_risk_level" in precheck_body
    assert "prepared_risk_level" in precheck_body
    assert "prepared_replay?.after_quality" in precheck_body
    assert "teachMetric('Risk'" not in precheck_body
    assert "prepared_record_path" in panel_text
    assert "preparedPoints.length" not in panel_text


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
    assert "Check mode uses automatic system dry-run" in panel_text
    assert "check mode is read-only" in panel_text


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
    assert "No key input" in panel_text
    assert "No valid teach trajectory" in panel_text
    assert "Execute Joints + Gripper" in panel_text
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


def test_controller_shutdown_runs_safe_home_before_disable_by_default():
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")
    hardware_text = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")

    assert 'self.declare_parameter("shutdown_safe_home", True)' in controller_text
    assert "self.hardware.endpos_ctrl.safe_home()" in controller_text
    assert "self.hardware.shutdown()" in controller_text
    assert "def connected(self) -> bool:" in hardware_text
