"""RViz execution policy; fake hardware only, no ROS node or motor connection."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from control_msgs.action import FollowJointTrajectory
from rclpy.action import GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint

from rebotarmcontroller.command_arbiter import CommandArbiter
from rebotarmcontroller.hardware_manager import HardwareManager
from rebotarmcontroller.ros_actions import ArmActions
from rebotarmcontroller.trajectory_safety import ARM_JOINT_NAMES, TrajectorySafetyLimits


class Hardware:
    prepare_trajectory_execution = HardwareManager.prepare_trajectory_execution

    def __init__(self):
        self._motor_lifecycle_lock = threading.RLock()
        self.connected = True
        self.enabled = False
        self.ready_for_motion = False
        self.lifecycle_state = "CONNECTED_DISABLED"
        self.state_machine = "IDLE"
        self.error_codes = []
        self.joint_names = list(ARM_JOINT_NAMES)
        self.command_arbiter = CommandArbiter()
        self.endpos_ctrl = SimpleNamespace(_q_target=np.zeros(6))
        self.enable_calls = 0
        self.enable_error = None

    def enable(self):
        self.enable_calls += 1
        if self.enable_error:
            raise RuntimeError(self.enable_error)
        self.enabled = self.ready_for_motion = True
        self.lifecycle_state = "ENABLED_HOLD"

    def ensure_pos_vel_control(self):
        assert self.ready_for_motion

    def set_state_machine(self, state):
        self.state_machine = state

    def get_joint_state(self):
        return self.endpos_ctrl._q_target.copy(), np.zeros(6), np.zeros(6)

    def hold_current_position(self):
        pass

    def stop_active_motion(self):
        self.state_machine = "IDLE"


def fixture(enabled_policy=True):
    actions = object.__new__(ArmActions)
    actions._hardware = Hardware()
    actions._node = Mock()
    actions._command_arbiter = actions._hardware.command_arbiter
    actions._enable_on_trajectory = enabled_policy
    actions._trajectory_limits = TrajectorySafetyLimits()
    actions._goal_tolerance_rad = 0.03
    actions._settle_timeout_sec = 0.1
    actions._sample_period_sec = 0.005
    actions._goal_leases = {}
    request = FollowJointTrajectory.Goal()
    request.trajectory.joint_names = list(ARM_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 6
    request.trajectory.points = [point]
    handle = Mock(request=request, is_cancel_requested=False)
    return actions, handle


def test_goal_acceptance_does_not_enable_and_execute_holds_after_success():
    actions, handle = fixture()
    assert actions.trajectory_goal_callback(handle.request) == GoalResponse.ACCEPT
    assert actions._hardware.enable_calls == 0
    result = actions._execute_follow_joint_trajectory_exclusive(handle)
    assert result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    handle.succeed.assert_called_once()
    assert actions._hardware.enable_calls == 1
    assert actions._hardware.enabled
    assert actions._hardware.state_machine == "IDLE"
    assert actions._command_arbiter.available("arm")


@pytest.mark.parametrize("bad", [float("nan"), 100.0, 0.2])
def test_invalid_or_distant_start_never_enables(bad):
    actions, handle = fixture()
    handle.request.trajectory.points[0].positions[0] = bad
    assert actions.trajectory_goal_callback(handle.request) == GoalResponse.REJECT
    result = actions._execute_follow_joint_trajectory_exclusive(handle)
    assert result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    assert actions._hardware.enable_calls == 0


def test_default_and_other_motion_interfaces_still_require_enable():
    actions, handle = fixture(False)
    assert actions.trajectory_goal_callback(handle.request) == GoalResponse.REJECT
    actions._enable_on_trajectory = True
    assert actions.arm_goal_callback(handle.request) == GoalResponse.REJECT
    assert actions.gripper_goal_callback(handle.request) == GoalResponse.REJECT
    assert actions._hardware.enable_calls == 0


@pytest.mark.parametrize("attribute,value", [
    ("connected", False), ("lifecycle_state", "DISABLING"),
    ("error_codes", ["feedback stale"]),
])
def test_faults_do_not_trigger_enable(attribute, value):
    actions, handle = fixture()
    setattr(actions._hardware, attribute, value)
    assert actions.trajectory_goal_callback(handle.request) == GoalResponse.REJECT
    result = actions._execute_follow_joint_trajectory_exclusive(handle)
    assert result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    assert actions._hardware.enable_calls == 0


def test_cancel_before_execute_does_not_enable():
    actions, handle = fixture()
    handle.is_cancel_requested = True
    actions._execute_follow_joint_trajectory_exclusive(handle)
    handle.canceled.assert_called_once()
    assert actions._hardware.enable_calls == 0


def test_enable_failure_aborts_instead_of_reporting_success():
    actions, handle = fixture()
    actions._hardware.enable_error = "enable failed and was rolled back"
    result = actions._execute_follow_joint_trajectory_exclusive(handle)
    assert result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    assert "rolled back" in result.error_string
    handle.succeed.assert_not_called()
    assert not actions._hardware.enabled


def test_busy_trajectory_cannot_share_another_goals_lease():
    actions, handle = fixture()
    actions._command_arbiter.acquire("arm", "follow_joint_trajectory:other")
    actions._execute_follow_joint_trajectory_exclusive(handle)
    assert actions._hardware.enable_calls == 0
    handle.abort.assert_called_once()


def test_preempted_transition_cannot_reenable():
    actions, _ = fixture()
    with pytest.raises(RuntimeError, match="preempted"):
        actions._hardware.prepare_trajectory_execution(allow_enable=True, is_current=lambda: False)
    assert actions._hardware.enable_calls == 0


def test_feedback_is_revalidated_after_acceptance():
    actions, handle = fixture()
    assert actions.trajectory_goal_callback(handle.request) == GoalResponse.ACCEPT
    actions._hardware.endpos_ctrl._q_target[:] = 0.2
    actions._execute_follow_joint_trajectory_exclusive(handle)
    assert actions._hardware.enable_calls == 0
    handle.abort.assert_called_once()


def test_cancellation_during_enable_never_starts_trajectory():
    actions, handle = fixture()
    enable = actions._hardware.enable
    def cancel_during_enable():
        enable()
        handle.is_cancel_requested = True
    actions._hardware.enable = cancel_during_enable
    result = actions._execute_follow_joint_trajectory_exclusive(handle)
    assert result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    assert actions._hardware.state_machine == "IDLE"
    handle.succeed.assert_not_called()


def test_launches_use_native_moveit_and_one_backend_without_dashboard():
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node
    root = Path(__file__).resolve().parents[1]
    def load(name):
        spec = importlib.util.spec_from_file_location(
            name, root / 'src/rebotarm_bringup/launch' / (name + '.launch.py')
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    real = load('rviz_ee_drag_real')
    context = LaunchContext()
    context.launch_configurations.update(channel='can0', shutdown_safe_home='false')
    entities = real._include_system(context)
    assert not any(isinstance(item, Node) for item in entities)
    real_args = dict(next(item for item in entities if isinstance(item, IncludeLaunchDescription)).launch_arguments)
    assert real_args['hardware_mode'] == 'real'
    assert real_args['enable_on_trajectory'] == 'true'
    assert real_args['use_moveit_fake_joint_states'] == 'false'
    sim = load('rviz_ee_drag_sim').generate_launch_description()
    nodes = [item for item in sim.entities if isinstance(item, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_executable == 'rebotarm_rviz_fake_controller'
    sim_args = dict(next(item for item in sim.entities if isinstance(item, IncludeLaunchDescription)).launch_arguments)
    assert sim_args['hardware_mode'] == 'sim'
    assert sim_args['use_hardware'] == 'false'
    assert sim_args['use_moveit_fake_joint_states'] == 'false'
    assert sim_args['start_passive_joint_state_publisher'] == 'false'
