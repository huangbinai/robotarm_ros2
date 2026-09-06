from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import types

import numpy as np
import pytest

from rebotarmcontroller.command_arbiter import CommandArbiter
from rebotarmcontroller.gripper_safety import (
    active_gripper_failure_reason,
    is_gripper_contact_sample,
)
from rebotarmcontroller.runtime_parameters import (
    arm_namespace,
    command_arbitration,
    finite_rate_hz,
    validate_move_to_pose_goal,
)
from rebotarmcontroller.trajectory_safety import (
    ARM_JOINT_NAMES,
    TrajectorySafetyLimits,
    interpolate_trajectory,
    validate_trajectory,
)
from rebotarm_vision.freshness import FreshnessTracker


ROOT = Path(__file__).resolve().parents[1]


def _install_hardware_import_stubs() -> None:
    if "tf_transformations" not in sys.modules:
        module = types.ModuleType("tf_transformations")
        module.euler_from_quaternion = lambda _value: (0.0, 0.0, 0.0)
        module.quaternion_from_matrix = lambda _value: (0.0, 0.0, 0.0, 1.0)
        sys.modules["tf_transformations"] = module


def _point(time_sec: float, positions, *, velocities=(), accelerations=()):
    seconds = int(time_sec)
    return SimpleNamespace(
        time_from_start=SimpleNamespace(
            sec=seconds,
            nanosec=int(round((time_sec - seconds) * 1_000_000_000)),
        ),
        positions=list(positions),
        velocities=list(velocities),
        accelerations=list(accelerations),
    )


def test_trajectory_rejects_non_finite_and_out_of_range_positions():
    limits = TrajectorySafetyLimits()
    current = np.zeros(6)

    with pytest.raises(ValueError, match="finite"):
        validate_trajectory(
            ARM_JOINT_NAMES,
            [_point(0.0, current), _point(1.0, [float("nan")] * 6)],
            current,
            limits,
        )
    with pytest.raises(ValueError, match="position limit"):
        validate_trajectory(
            ARM_JOINT_NAMES,
            [_point(0.0, current), _point(40.0, [100.0] * 6)],
            current,
            limits,
        )


def test_trajectory_requires_strictly_increasing_timestamps():
    current = np.zeros(6)
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_trajectory(
            ARM_JOINT_NAMES,
            [_point(0.0, current), _point(1.0, current), _point(1.0, current)],
            current,
            TrajectorySafetyLimits(),
        )


def test_trajectory_rejects_segment_velocity_limit_violation():
    current = np.zeros(6)
    target = np.array([0.4, 0.0, 0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError, match="velocity limit"):
        validate_trajectory(
            ARM_JOINT_NAMES,
            [_point(0.0, current), _point(0.1, target)],
            current,
            TrajectorySafetyLimits(),
        )


def test_trajectory_interpolation_moves_continuously_between_points():
    result = interpolate_trajectory(
        [0.0, 1.0], [np.zeros(6), np.ones(6)], 0.25
    )
    assert result == pytest.approx(np.full(6, 0.25))


def test_command_arbiter_rejects_competing_owner_and_ignores_stale_release():
    arbiter = CommandArbiter()
    first = arbiter.acquire("arm", "trajectory-a")
    assert first is not None
    assert arbiter.acquire("arm", "trajectory-b") is None
    assert arbiter.owner("arm") == "trajectory-a"

    assert arbiter.force_release("arm") == first
    second = arbiter.acquire("arm", "trajectory-b")
    assert second is not None
    assert arbiter.release(first) is False
    assert arbiter.owner("arm") == "trajectory-b"
    assert arbiter.release(second) is True


def test_gripper_contact_rejects_empty_hard_stop_and_requires_torque():
    common = dict(
        closure_m=0.08,
        velocity_rad_s=0.0,
        torque_nm=0.4,
        min_opening_m=0.003,
        min_closure_m=0.006,
        max_velocity_rad_s=0.04,
        min_torque_nm=0.05,
    )
    assert not is_gripper_contact_sample(opening_m=0.0, **common)
    assert not is_gripper_contact_sample(
        opening_m=0.02, **{**common, "torque_nm": 0.0}
    )
    assert is_gripper_contact_sample(opening_m=0.02, **common)


def test_active_gripper_failure_reports_command_and_feedback_faults():
    assert active_gripper_failure_reason(command_error=None, status_code=1) is None
    assert active_gripper_failure_reason(
        command_error="feedback stale",
        status_code=1,
    ) == "feedback stale"
    assert active_gripper_failure_reason(
        command_error=None,
        status_code=255,
    ) == "gripper feedback status_code=255, expected 1"


def test_gripper_action_wires_immediate_failure_and_timeout_stop():
    source = (ROOT / "src/rebotarmcontroller/rebotarmcontroller/ros_actions.py").read_text(
        encoding="utf-8"
    )
    assert "active_gripper_failure_reason(" in source
    assert "self._hardware.stop_gripper_motion(failure)" in source
    assert 'self._hardware.stop_gripper_motion("gripper action timeout")' in source


def test_move_to_pose_rechecks_completion_and_requires_final_settle():
    source = (ROOT / "src/rebotarmcontroller/rebotarmcontroller/ros_actions.py").read_text(
        encoding="utf-8"
    )
    body = source.split("def execute_move_to_pose(self, goal_handle):", 1)[1].split(
        "\n    def _stop_move_to_pose_motion", 1
    )[0]

    assert body.count("self._move_to_pose_interrupted(goal_handle, result)") >= 3
    assert "motion_deadline" in body
    assert "final_target" in body
    assert "max_error <= self._goal_tolerance_rad" in body


def test_lifecycle_cleanup_and_visual_execution_are_guarded():
    hardware = (ROOT / "src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py").read_text(
        encoding="utf-8"
    )
    visual = (ROOT / "src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py").read_text(
        encoding="utf-8"
    )

    assert 'self._lifecycle_state not in ("DISABLING", "DISCONNECTED")' in hardware
    assert "self._execution_lock = threading.Lock()" in visual
    assert "with self._execution_lock:" in visual


def test_teach_replay_stop_requires_successful_controller_response():
    source = (ROOT / "src/rebotarm_teach/rebotarm_teach/teach_replay_node.py").read_text(
        encoding="utf-8"
    )
    body = source.split(
        "def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:",
        1,
    )[1].split("\n    def ", 1)[0]

    assert "response = future.result()" in body
    assert "response is not None and response.success" in body


def test_action_goals_require_ready_hardware_before_command_arbitration():
    source = (ROOT / "src/rebotarmcontroller/rebotarmcontroller/ros_actions.py").read_text(
        encoding="utf-8"
    )
    assert source.count("if not self._hardware.ready_for_motion:") == 2
    assert 'self._command_arbiter.available("arm")' in source
    assert 'self._command_arbiter.available("gripper")' in source


def test_freshness_tracker_invalidates_and_expires_values():
    now = [10.0]
    tracker = FreshnessTracker(monotonic=lambda: now[0])
    tracker.touch("plan")
    assert tracker.is_fresh("plan", 1.0)
    now[0] = 11.1
    assert not tracker.is_fresh("plan", 1.0)
    tracker.touch("plan")
    tracker.invalidate("plan")
    assert not tracker.is_fresh("plan", 1.0)


def test_runtime_rates_reject_non_finite_nonpositive_and_excessive_values():
    assert finite_rate_hz("joint_state_rate", 100.0) == 100.0
    for value in (float("nan"), float("inf"), 0.0, -1.0, 501.0):
        with pytest.raises(ValueError, match="joint_state_rate"):
            finite_rate_hz("joint_state_rate", value)


def test_namespace_and_command_arbitration_fail_fast():
    assert arm_namespace("/robot/arm/") == "robot/arm"
    assert command_arbitration(" PREEMPT ") == "preempt"
    for value in ("", "///", "robot//arm", "robot-arm"):
        with pytest.raises(ValueError, match="arm_namespace"):
            arm_namespace(value)
    for value in ("", "latest", None):
        with pytest.raises(ValueError, match="cmd_arbitration"):
            command_arbitration(value)


def test_move_to_pose_goal_rejects_invalid_duration_pose_and_quaternion():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    goal = SimpleNamespace(target_pose=pose, duration=2.0)
    assert validate_move_to_pose_goal(goal) == 2.0

    for duration in (float("nan"), 0.0, -1.0, 301.0):
        with pytest.raises(ValueError, match="duration"):
            validate_move_to_pose_goal(
                SimpleNamespace(target_pose=pose, duration=duration)
            )

    pose.position.x = float("nan")
    with pytest.raises(ValueError, match="finite"):
        validate_move_to_pose_goal(goal)
    pose.position.x = 0.1
    pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="quaternion"):
        validate_move_to_pose_goal(goal)


def test_hardware_feedback_reports_missing_and_aging_samples(monkeypatch):
    _install_hardware_import_stubs()
    import rebotarmcontroller.hardware_manager as hardware_module

    manager = object.__new__(hardware_module.HardwareManager)
    manager._arm = SimpleNamespace(control_loop_active=False)
    manager.get_joint_state = lambda: (np.zeros(6), np.zeros(6), np.zeros(6))
    manager._arm_feedback_updated_monotonic = None
    assert manager.feedback().age_sec == float("inf")

    manager._arm_feedback_updated_monotonic = 90.0
    monkeypatch.setattr(hardware_module.time, "monotonic", lambda: 100.0)
    assert manager.feedback().age_sec == pytest.approx(10.0)


def test_hardware_safe_home_propagates_timeout_and_holds():
    _install_hardware_import_stubs()
    from rebotarmcontroller.hardware_manager import HardwareManager

    manager = object.__new__(HardwareManager)
    calls = []
    manager.stop_gravity_compensation = lambda: calls.append("stop_gravity")
    manager.ensure_pos_vel_control = lambda: calls.append("position_mode")
    manager.hold_current_position = lambda: calls.append("hold")
    manager._endpos_ctrl = SimpleNamespace(safe_home=lambda **_kwargs: False)
    manager._error_codes = []

    assert manager.safe_home() is False
    assert calls == ["stop_gravity", "position_mode", "hold"]
    assert manager._error_codes == ["SAFE_HOME_TIMEOUT"]


def test_all_hardware_launches_load_controller_safety_config():
    launch_names = (
        "bringup.launch.py",
        "driver_only.launch.py",
        "interactive_basic.launch.py",
        "interactive_system.launch.py",
        "moveit_hardware.launch.py",
        "teleop_keyboard.launch.py",
        "visual_ready_hold.launch.py",
    )
    for name in launch_names:
        text = (ROOT / "src/rebotarm_bringup/launch" / name).read_text(encoding="utf-8")
        assert "controller_safety.yaml" in text, name
