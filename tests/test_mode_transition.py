from __future__ import annotations

from collections import deque
from pathlib import Path
import sys
import types

import numpy as np
import yaml


geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")


class _Pose:
    pass


geometry_msgs_msg.Pose = _Pose
geometry_msgs.msg = geometry_msgs_msg
sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

tf_transformations = types.ModuleType("tf_transformations")
tf_transformations.euler_from_quaternion = lambda _q: (0.0, 0.0, 0.0)
tf_transformations.quaternion_from_matrix = lambda _m: (0.0, 0.0, 0.0, 1.0)
sys.modules.setdefault("tf_transformations", tf_transformations)


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "src" / "rebotarmcontroller"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += float(duration)


class FakeHardware:
    def __init__(self, mode="pos_vel", velocity_samples=None):
        self.mode = mode
        self.enabled = True
        self.calls = []
        self.position = np.array([0.2, -0.1], dtype=np.float64)
        samples = velocity_samples or [np.zeros(2)]
        self.velocity_samples = deque(np.asarray(v, dtype=np.float64) for v in samples)
        self.last_velocity = self.velocity_samples[-1].copy()
        self.feedback_valid = True
        self.switch_ok = True

    def feedback(self):
        from rebotarmcontroller.mode_transition_policy import FeedbackSample

        if not self.feedback_valid:
            return FeedbackSample(
                positions=np.array([np.nan, np.nan]),
                velocities=np.zeros(2),
            )
        if self.velocity_samples:
            self.last_velocity = self.velocity_samples.popleft()
        return FeedbackSample(self.position.copy(), self.last_velocity.copy())

    def gravity_torque(self, _positions):
        return np.array([1.0, 2.0])

    def preload_position_hold(self, target):
        self.calls.append(("preload_position_hold", np.asarray(target).copy()))

    def stop_control_loop(self):
        self.calls.append(("stop_control_loop",))

    def switch_mode(self, mode, *, kp=None, kd=None):
        self.calls.append(("switch_mode", mode, kp, kd))
        if not self.switch_ok:
            raise RuntimeError("mode switch failed")
        self.mode = mode

    def send_mit(self, *, position, kp, kd, torque):
        self.calls.append(
            ("send_mit", np.asarray(position).copy(), float(kp), float(kd), np.asarray(torque).copy())
        )

    def start_gravity_loop(self, target):
        self.calls.append(("start_gravity_loop", np.asarray(target).copy()))

    def finish_gravity_compensation(self):
        self.calls.append(("finish_gravity_compensation",))

    def start_position_hold(self, target, *, zero_velocity_limit=False):
        self.calls.append(
            ("start_position_hold", np.asarray(target).copy(), bool(zero_velocity_limit))
        )

    def restore_position_velocity_limit(self):
        self.calls.append(("restore_position_velocity_limit",))

    def disable_immediately(self):
        self.calls.append(("disable_immediately",))
        self.enabled = False


class _ArmWithMode:
    def __init__(self, mode="pos_vel"):
        self.mode = mode


class _TransitionState:
    def __init__(self, in_progress=False):
        self.in_progress = in_progress


def _config(**overrides):
    from dataclasses import replace

    from rebotarmcontroller.mode_transition_policy import ModeTransitionConfig

    return replace(
        ModeTransitionConfig(),
        enter_ramp_duration_sec=0.04,
        exit_damping_duration_sec=0.02,
        exit_blend_duration_sec=0.04,
        pos_vel_settle_duration_sec=0.02,
        exit_velocity_wait_timeout_sec=0.06,
        transition_timeout_sec=0.5,
        **overrides,
    )


def test_enter_gravity_compensation_runs_ordered_stages():
    from rebotarmcontroller.mode_transition import ModeTransitionCoordinator

    hardware = FakeHardware(mode="pos_vel")
    clock = FakeClock()
    stages = []
    coordinator = ModeTransitionCoordinator(
        hardware,
        _config(),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        control_period_sec=0.02,
        on_stage=stages.append,
    )

    result = coordinator.enter_gravity_compensation()

    assert result.success is True
    assert stages == ["ENTERING_GRAVITY_COMP", "GRAVITY_COMP"]
    assert hardware.mode == "mit"
    assert any(call[0] == "start_gravity_loop" for call in hardware.calls)


def test_exit_gravity_compensation_runs_damping_blend_and_settle():
    from rebotarmcontroller.mode_transition import ModeTransitionCoordinator

    hardware = FakeHardware(
        mode="mit",
        velocity_samples=[np.array([0.2, 0.0]), np.array([0.01, 0.0])],
    )
    clock = FakeClock()
    stages = []
    coordinator = ModeTransitionCoordinator(
        hardware,
        _config(),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        control_period_sec=0.02,
        on_stage=stages.append,
    )

    result = coordinator.exit_gravity_compensation()

    assert result.success is True
    assert stages == [
        "EXIT_DAMPING",
        "EXIT_BLENDING",
        "POS_VEL_SETTLING",
        "POS_VEL_HOLD",
    ]
    assert hardware.mode == "pos_vel"
    assert hardware.calls[-1][0] == "restore_position_velocity_limit"


def test_enter_rejects_moving_joints_before_switching_mode():
    from rebotarmcontroller.mode_transition import ModeTransitionCoordinator

    hardware = FakeHardware(mode="pos_vel", velocity_samples=[np.array([0.2, 0.0])])
    coordinator = ModeTransitionCoordinator(hardware, _config())

    result = coordinator.enter_gravity_compensation()

    assert result.success is False
    assert "velocity" in result.failure_reason
    assert not any(call[0] == "switch_mode" for call in hardware.calls)


def test_exit_times_out_when_joint_never_slows_down():
    from rebotarmcontroller.mode_transition import ModeTransitionCoordinator

    hardware = FakeHardware(
        mode="mit",
        velocity_samples=[np.array([0.2, 0.0])] * 10,
    )
    clock = FakeClock()
    coordinator = ModeTransitionCoordinator(
        hardware,
        _config(),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        control_period_sec=0.02,
    )

    result = coordinator.exit_gravity_compensation()

    assert result.success is False
    assert result.stage == "EXIT_DAMPING"
    assert "velocity" in result.failure_reason


def test_feedback_loss_uses_immediate_disable_fallback():
    from rebotarmcontroller.mode_transition import ModeTransitionCoordinator

    hardware = FakeHardware(mode="pos_vel")
    hardware.feedback_valid = False
    coordinator = ModeTransitionCoordinator(hardware, _config())

    result = coordinator.enter_gravity_compensation()

    assert result.success is False
    assert hardware.enabled is False
    assert hardware.calls[-1][0] == "disable_immediately"


def test_mode_switch_failure_falls_back_to_position_hold():
    from rebotarmcontroller.mode_transition import ModeTransitionCoordinator

    hardware = FakeHardware(mode="pos_vel")
    hardware.switch_ok = False
    coordinator = ModeTransitionCoordinator(hardware, _config())

    result = coordinator.enter_gravity_compensation()

    assert result.success is False
    assert any(call[0] == "start_position_hold" for call in hardware.calls)


def test_concurrent_transition_request_is_rejected():
    from rebotarmcontroller.mode_transition import ModeTransitionCoordinator

    hardware = FakeHardware(mode="pos_vel")
    coordinator = ModeTransitionCoordinator(hardware, _config())
    assert coordinator._lock.acquire(blocking=False)
    try:
        result = coordinator.enter_gravity_compensation()
    finally:
        coordinator._lock.release()

    assert result.success is False
    assert result.failure_reason == "mode transition already in progress"


def test_mode_transition_yaml_contains_approved_defaults():
    config_path = ROOT / "src/rebotarm_bringup/config/mode_transition.yaml"
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = payload["reBotArmController"]["ros__parameters"]

    assert params["mode_transition.enabled"] is True
    assert params["mode_transition.allow_velocity_mode"] is False
    assert params["mode_transition.enter.ramp_duration_sec"] == 0.35
    assert params["mode_transition.exit.blend_duration_sec"] == 0.35
    assert params["mode_transition.mit.gravity_kp"] == 7.0
    assert params["mode_transition.mit.hold_kp"] == 12.0
    assert params["mode_transition.safety.transition_timeout_sec"] == 2.0


def test_real_hardware_launches_load_mode_transition_yaml():
    for relative in (
        "src/rebotarm_bringup/launch/bringup.launch.py",
        "src/rebotarm_bringup/launch/driver_only.launch.py",
        "src/rebotarm_bringup/launch/interactive_system.launch.py",
    ):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert '"config", "mode_transition.yaml"' in text, relative
        assert "mode_transition_params" in text, relative
        assert "parameters=[mode_transition_params," in text, relative


def test_controller_builds_transition_config_from_ros_parameters():
    text = (
        ROOT
        / "src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter("mode_transition.enabled", True)' in text
    assert 'declare_parameter("mode_transition.allow_velocity_mode", False)' in text
    assert "ModeTransitionConfig(" in text
    assert "mode_transition_config=mode_transition_config" in text


def test_hardware_manager_rejects_velocity_mode_by_default():
    from rebotarmcontroller.hardware_manager import HardwareManager

    manager = HardwareManager.__new__(HardwareManager)
    manager._arm = _ArmWithMode("pos_vel")
    manager._gravity_comp_active = False
    manager._mode_transition_config = _config(allow_velocity_mode=False)
    manager._mode_transition = _TransitionState()

    try:
        manager.set_mode("vel")
    except ValueError as exc:
        assert "VEL mode is disabled" in str(exc)
    else:
        raise AssertionError("VEL mode should be rejected")


def test_hardware_manager_rejects_direct_mit_mode_entry():
    from rebotarmcontroller.hardware_manager import HardwareManager

    manager = HardwareManager.__new__(HardwareManager)
    manager._arm = _ArmWithMode("pos_vel")
    manager._gravity_comp_active = False
    manager._mode_transition_config = _config()
    manager._mode_transition = _TransitionState()

    try:
        manager.set_mode("mit")
    except ValueError as exc:
        assert "gravity compensation" in str(exc)
    else:
        raise AssertionError("direct MIT entry should be rejected")


def test_lowlevel_velocity_command_is_rejected_by_default():
    from types import SimpleNamespace

    from rebotarmcontroller.hardware_manager import HardwareManager

    manager = HardwareManager.__new__(HardwareManager)
    manager._mode_transition_config = _config(allow_velocity_mode=False)
    manager._mode_transition = _TransitionState()

    try:
        manager.send_joint_motor_cmd("joint1", SimpleNamespace(mode=2))
    except ValueError as exc:
        assert "VEL mode is disabled" in str(exc)
    else:
        raise AssertionError("low-level VEL command should be rejected")


def test_lowlevel_command_is_rejected_during_transition():
    from types import SimpleNamespace

    from rebotarmcontroller.hardware_manager import HardwareManager

    manager = HardwareManager.__new__(HardwareManager)
    manager._mode_transition_config = _config()
    manager._mode_transition = _TransitionState(in_progress=True)

    try:
        manager.send_joint_motor_cmd("joint1", SimpleNamespace(mode=1))
    except RuntimeError as exc:
        assert "transition" in str(exc)
    else:
        raise AssertionError("low-level command should be rejected during transition")


def test_position_control_is_rejected_during_transition():
    from rebotarmcontroller.hardware_manager import HardwareManager

    manager = HardwareManager.__new__(HardwareManager)
    manager._mode_transition = _TransitionState(in_progress=True)

    try:
        manager.ensure_pos_vel_control()
    except RuntimeError as exc:
        assert "transition" in str(exc)
    else:
        raise AssertionError("position control should be rejected during transition")


def test_position_control_smoothly_exits_gravity_compensation_first():
    from types import SimpleNamespace

    from rebotarmcontroller.hardware_manager import HardwareManager

    calls = []
    arm = SimpleNamespace(mode="mit", control_loop_active=False)
    manager = HardwareManager.__new__(HardwareManager)
    manager._arm = arm
    manager._enabled = True
    manager._gravity_comp_active = True
    manager._mode_transition = _TransitionState()

    def stop_gravity_compensation():
        calls.append("stop_gravity_compensation")
        manager._gravity_comp_active = False
        arm.mode = "pos_vel"

    manager.stop_gravity_compensation = stop_gravity_compensation
    manager._start_pos_vel_loop = lambda: calls.append("start_pos_vel_loop")

    manager.ensure_pos_vel_control()

    assert calls == ["stop_gravity_compensation", "start_pos_vel_loop"]


def test_disable_still_disables_when_smooth_exit_fails():
    from types import SimpleNamespace

    from rebotarmcontroller.hardware_manager import HardwareManager

    calls = []
    manager = HardwareManager.__new__(HardwareManager)
    manager._arm = SimpleNamespace(disable=lambda: calls.append("disable"))
    manager._enabled = True
    manager.stop_gravity_compensation = lambda: (_ for _ in ()).throw(
        RuntimeError("transition failed")
    )
    manager._stop_control_loop = lambda: calls.append("stop_control_loop")
    manager.set_state_machine = lambda state: calls.append(state)

    try:
        manager.disable()
    except RuntimeError as exc:
        assert "transition failed" in str(exc)

    assert "disable" in calls
    assert manager._enabled is False


def test_shutdown_still_disables_when_smooth_exit_fails():
    from types import SimpleNamespace

    from rebotarmcontroller.hardware_manager import HardwareManager

    calls = []
    manager = HardwareManager.__new__(HardwareManager)
    manager._connected = True
    manager._enabled = True
    manager._arm = SimpleNamespace(
        disable=lambda: calls.append("disable"),
        disconnect=lambda: calls.append("disconnect"),
    )
    manager._endpos_ctrl = SimpleNamespace(_running=False)
    manager._stop_gripper_loop = lambda: calls.append("stop_gripper_loop")
    manager.stop_gravity_compensation = lambda: (_ for _ in ()).throw(
        RuntimeError("transition failed")
    )
    manager._stop_control_loop = lambda: calls.append("stop_control_loop")

    manager.shutdown()

    assert "disable" in calls
    assert "disconnect" in calls
    assert manager._connected is False
    assert manager._enabled is False
