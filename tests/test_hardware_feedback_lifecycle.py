from __future__ import annotations

from types import SimpleNamespace
import threading
import sys
import types
import math

import numpy as np
import pytest

if "rebotarmcontroller.conversions" not in sys.modules:
    conversions = types.ModuleType("rebotarmcontroller.conversions")
    conversions.fk_to_pose = lambda *args, **kwargs: (args, kwargs)
    sys.modules["rebotarmcontroller.conversions"] = conversions
if "motorbridge" not in sys.modules:
    motorbridge = types.ModuleType("motorbridge")
    motorbridge.Mode = SimpleNamespace(MIT=0)
    sys.modules["motorbridge"] = motorbridge

from rebotarmcontroller.hardware_manager import HardwareManager


@pytest.mark.parametrize(
    ("connected", "enabled", "lifecycle", "expected"),
    [
        (False, False, "DISCONNECTED", False),
        (True, False, "CONNECTED_DISABLED", False),
        (True, True, "ENABLING", False),
        (True, True, "ENABLED_HOLD", True),
        (True, True, "TRAJECTORY_RUNNING", True),
        (True, True, "DISABLING", False),
    ],
)
def test_ready_for_motion_requires_enabled_motion_lifecycle(
    connected, enabled, lifecycle, expected
) -> None:
    manager = HardwareManager.__new__(HardwareManager)
    manager._connected = connected
    manager._enabled = enabled
    manager._lifecycle_state = lifecycle

    assert manager.ready_for_motion is expected


@pytest.mark.parametrize("joint_name", ["joint2", "joint3"])
def test_joint2_and_joint3_feedback_accepts_positive_endpoint_margin(joint_name) -> None:
    manager = HardwareManager.__new__(HardwareManager)
    accepted = SimpleNamespace(pos=0.02, vel=0.0, torq=0.0)
    rejected = SimpleNamespace(pos=0.020001, vel=0.0, torq=0.0)

    manager._validate_feedback_state(joint_name, accepted)
    with pytest.raises(RuntimeError, match="outside feedback range"):
        manager._validate_feedback_state(joint_name, rejected)


class _Motor:
    def __init__(self, state) -> None:
        self.state = state
        self.sequence = 7
        self.requested = False

    def get_state_with_sequence(self):
        return self.state, self.sequence

    def request_feedback(self) -> None:
        self.requested = True


class _Controller:
    def __init__(self, motors, *, advance=True) -> None:
        self._bus_lock = threading.RLock()
        self.motors = motors
        self.advance = advance
        self.poll_count = 0

    def poll_feedback_once(self) -> None:
        self.poll_count += 1
        if self.advance:
            for motor in self.motors:
                if motor.requested:
                    motor.sequence += 1
                    motor.requested = False


def _feedback_manager(*, advance=True):
    manager = object.__new__(HardwareManager)
    names = [f"joint{i}" for i in range(1, 7)]
    state = SimpleNamespace(pos=0.0, vel=0.0, torq=0.0, status_code=0)
    motors = {name: _Motor(state) for name in names}
    controller = _Controller(list(motors.values()), advance=advance)
    joints = [SimpleNamespace(name=name, vendor="damiao") for name in names]
    manager._arm = SimpleNamespace(
        joint_names=names,
        _joints=joints,
        _motor_map=motors,
        _ctrl_map={"damiao": controller},
        _ctrl_thread=None,
        control_loop_active=False,
    )
    manager._gripper_mot = None
    manager._gripper_ctrl = None
    manager._feedback_lock = threading.RLock()
    manager._gripper_lock = threading.RLock()
    manager._verified_feedback_by_label = {}
    manager._feedback_request_baseline_by_label = {}
    manager._feedback_request_deadline_by_label = {}
    manager._feedback_error_by_label = {}
    manager._feedback_next_refresh_monotonic = None
    manager._hardware_feedback_period_sec = 0.02
    manager._feedback_stale_timeout_sec = 0.15
    manager._arm_feedback_updated_monotonic = None
    manager._arm_feedback_error = "arm feedback not received"
    return manager, controller


def _gripper_zero_manager():
    manager, controller = _feedback_manager()
    gripper = _Motor(
        SimpleNamespace(pos=-1.0, vel=0.0, torq=0.0, status_code=0)
    )
    controller.motors.append(gripper)
    manager._gripper_mot = gripper
    manager._gripper_ctrl = controller
    manager._gripper_pos = -1.0
    manager._gripper_vel = 0.0
    manager._gripper_torque = 0.0
    manager._gripper_feedback_updated_monotonic = None
    manager._gripper_feedback_error = "gripper feedback not received"
    manager._gripper_zero_error = None
    manager._connected = True
    manager._enabled = False
    manager._lifecycle_state = "CONNECTED_DISABLED"
    manager._state_machine = "IDLE"
    manager._motor_lifecycle_lock = threading.RLock()
    manager._stop_control_loop = lambda: None
    manager._stop_gripper_loop = lambda: None
    manager._error_codes = []
    return manager, gripper


def test_shared_feedback_batch_accepts_only_advanced_sequences() -> None:
    manager, controller = _feedback_manager()
    assert manager.refresh_feedback_if_due(force=True)
    assert controller.poll_count == 1
    assert {sample.sequence for sample in manager._verified_feedback_by_label.values()} == {8}
    assert manager._arm_feedback_error is None


def test_forced_feedback_rejects_replayed_cache() -> None:
    manager, controller = _feedback_manager(advance=False)
    with pytest.raises(RuntimeError, match="sequence did not advance"):
        manager.refresh_feedback_if_due(force=True)
    assert controller.poll_count == 3
    assert manager._verified_feedback_by_label == {}


def test_delayed_sequence_is_accepted_on_later_feedback_batch() -> None:
    manager, controller = _feedback_manager(advance=False)
    now = 10.0

    assert manager.refresh_feedback_if_due(now=now)
    assert manager._verified_feedback_by_label == {}
    assert manager._feedback_request_baseline_by_label

    for motor in controller.motors:
        motor.sequence += 1
        motor.requested = False

    assert manager.refresh_feedback_if_due(now=now + 0.02)
    assert {sample.sequence for sample in manager._verified_feedback_by_label.values()} == {8}
    assert manager._arm_feedback_error is None


def test_feedback_deadline_expiry_records_sequence_error() -> None:
    manager, _controller = _feedback_manager(advance=False)
    now = 10.0

    assert manager.refresh_feedback_if_due(now=now)
    assert manager.refresh_feedback_if_due(now=now + 0.08)

    assert "deadline expired" in manager._arm_feedback_error
    assert manager._verified_feedback_by_label == {}


def test_hardware_tick_reraises_non_feedback_callback_failure() -> None:
    manager = object.__new__(HardwareManager)
    manager.refresh_feedback_if_due = lambda: True
    manager._arm_feedback_failure_reason = lambda: None
    manager._gripper_tick = lambda: None
    manager._protective_disable_from_hardware_loop = lambda _reason: pytest.fail(
        "programming errors must not be mislabeled as feedback failures"
    )

    with pytest.raises(RuntimeError, match="callback bug"):
        manager._hardware_control_tick(
            SimpleNamespace(),
            0.002,
            lambda _arm, _dt: (_ for _ in ()).throw(RuntimeError("callback bug")),
        )


def test_hardware_tick_protects_on_confirmed_feedback_failure() -> None:
    manager = object.__new__(HardwareManager)
    manager.refresh_feedback_if_due = lambda: True
    manager._arm_feedback_failure_reason = lambda: "joint3 feedback stale"
    reasons = []
    manager._protective_disable_from_hardware_loop = reasons.append

    manager._hardware_control_tick(
        SimpleNamespace(),
        0.002,
        lambda _arm, _dt: pytest.fail("commands must not run after feedback loss"),
    )

    assert reasons == ["joint3 feedback stale"]


def test_connect_does_not_enable_or_start_a_command_loop() -> None:
    manager = object.__new__(HardwareManager)
    events = []
    manager._connected = False
    manager._enabled = False
    manager._lifecycle_state = "DISCONNECTED"
    manager._gripper_cfg_path = "gripper.yaml"
    manager._gripper_mot = None
    manager._arm = SimpleNamespace(connect=lambda: events.append("connect"))
    manager.init_gripper = lambda _path: events.append("init_gripper")
    manager._validated_joint_feedback = lambda **_kwargs: (
        np.zeros(6), np.zeros(6), np.zeros(6), [0] * 6
    )
    manager._disconnect_after_failed_connect = lambda: events.append("rollback")
    manager._disable_all_motors = lambda: events.append("disable")

    manager.connect()

    assert events == ["connect", "init_gripper"]
    assert manager.connected and not manager.enabled
    assert manager.lifecycle_state == "CONNECTED_DISABLED"


def test_enable_preloads_fresh_position_before_enabling() -> None:
    manager = object.__new__(HardwareManager)
    events = []
    positions = np.arange(6, dtype=np.float64) / 10.0
    manager._connected = True
    manager._enabled = False
    manager._lifecycle_state = "CONNECTED_DISABLED"
    manager._state_machine = "IDLE"
    manager._motor_lifecycle_lock = threading.RLock()
    manager._gripper_mot = None
    manager._endpos_ctrl = SimpleNamespace(_q_target=np.zeros(6))
    manager._arm = SimpleNamespace(
        mode_pos_vel=lambda: events.append("mode") or True,
        enable=lambda: events.append("enable"),
    )

    def feedback(*, expected_status=None, **_kwargs):
        events.append(f"feedback:{expected_status}")
        return positions, np.zeros(6), np.zeros(6), [expected_status] * 6

    manager._validated_joint_feedback = feedback
    manager._validated_gripper_status = lambda **_kwargs: None
    manager._start_pos_vel_loop = lambda target=None: events.append(
        "start:" + repr(np.asarray(target).tolist())
    )

    manager.enable()

    assert events[:3] == ["feedback:0", "mode", "enable"]
    assert events[3] == "feedback:1"
    assert np.array_equal(manager._endpos_ctrl._q_target, positions)
    assert manager.enabled
    assert manager.lifecycle_state == "ENABLED_HOLD"


def test_gripper_positive_feedback_is_only_accepted_within_closed_tolerance() -> None:
    manager = object.__new__(HardwareManager)
    manager._gripper_lock = threading.RLock()
    manager._gripper_pos = 0.01
    assert manager.gripper_position_m() == pytest.approx(0.0)

    manager._gripper_pos = 0.06
    assert math.isnan(manager.gripper_position_m())


def test_protective_disable_is_finalised_only_by_fresh_disabled_status() -> None:
    manager = object.__new__(HardwareManager)
    manager._motor_lifecycle_lock = threading.RLock()
    manager._connected = True
    manager._enabled = True
    manager._lifecycle_state = "DISABLING"
    manager._arm = SimpleNamespace(control_loop_active=False)
    manager.refresh_feedback_if_due = lambda: True
    values = (np.zeros(6), np.zeros(6), np.zeros(6), [0] * 6)
    manager._validated_joint_feedback = lambda **_kwargs: values
    manager._cached_gripper_status = lambda: 0

    manager.get_joint_state()

    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"


def test_unverified_protective_disable_does_not_claim_motors_are_disabled() -> None:
    manager = object.__new__(HardwareManager)
    manager._arm = SimpleNamespace(
        _running=True,
        _ctrl_map={"damiao": SimpleNamespace(disable_all=lambda: None)},
    )
    manager._endpos_ctrl = SimpleNamespace(
        _running=True,
        _stop_send=threading.Event(),
        _moving=True,
    )
    manager._gravity_comp_active = True
    manager._gripper_lock = threading.RLock()
    manager._gripper_active = False
    manager._gripper_mode = "idle"
    manager._state_machine = "TRAJ_RUNNING"
    manager._lifecycle_state = "TRAJECTORY_RUNNING"
    manager._enabled = True
    manager._error_codes = []

    manager._protective_disable_from_hardware_loop("feedback lost")

    assert manager.enabled is True
    assert manager.lifecycle_state == "DISABLING"
    assert "FEEDBACK_PROTECTIVE_DISABLE" in manager.error_codes


def test_action_cleanup_cannot_overwrite_disabling_lifecycle() -> None:
    manager = object.__new__(HardwareManager)
    manager._connected = True
    manager._enabled = True
    manager._state_machine = "TRAJ_RUNNING"
    manager._lifecycle_state = "DISABLING"

    manager.set_state_machine("IDLE")

    assert manager.state_machine == "IDLE"
    assert manager.lifecycle_state == "DISABLING"
    assert manager.ready_for_motion is False


def test_shutdown_reports_unverified_disable_without_claiming_disabled() -> None:
    manager = object.__new__(HardwareManager)
    manager._connected = True
    manager._enabled = True
    manager._lifecycle_state = "ENABLED_HOLD"
    manager._error_codes = []
    manager.stop_gravity_compensation = lambda: None
    manager.disable_immediately = lambda: False
    manager._endpos_ctrl = SimpleNamespace(_running=True)
    manager._arm = SimpleNamespace(disconnect=lambda: None)

    assert manager.shutdown() is False
    assert manager.connected is False
    assert manager.enabled is True
    assert manager.lifecycle_state == "DISCONNECTED"
    assert "SHUTDOWN_DISABLE_UNVERIFIED" in manager.error_codes


@pytest.mark.parametrize("position_m", [-0.001, 0.085001, 1.0])
def test_gripper_position_command_rejects_out_of_range_request(position_m) -> None:
    manager = object.__new__(HardwareManager)
    manager._connected = True
    manager._enabled = True
    manager._arm = SimpleNamespace(control_loop_active=True)
    manager._gripper_mot = object()
    manager._gripper_feedback_failure_reason = lambda: None

    with pytest.raises(ValueError, match="gripper position must be within"):
        manager.set_gripper_target(position_m)


def test_unverified_emergency_disable_does_not_claim_disabled() -> None:
    manager = object.__new__(HardwareManager)
    manager._motor_lifecycle_lock = threading.RLock()
    manager._connected = True
    manager._enabled = True
    manager._lifecycle_state = "ENABLED_HOLD"
    manager._error_codes = []
    manager._stop_control_loop = lambda: None
    manager._disable_all_motors = lambda: None
    manager._validated_joint_feedback = lambda **_kwargs: (_ for _ in ()).throw(
        RuntimeError("feedback timeout")
    )
    manager._validated_gripper_status = lambda **_kwargs: None

    assert manager.disable_immediately() is False
    assert manager.enabled is True
    assert manager.lifecycle_state == "DISABLING"
    assert any("EMERGENCY_DISABLE_UNVERIFIED" in code for code in manager._error_codes)


def test_emergency_disable_requires_fresh_disabled_feedback_before_finalising() -> None:
    manager = object.__new__(HardwareManager)
    manager._motor_lifecycle_lock = threading.RLock()
    manager._connected = True
    manager._enabled = True
    manager._lifecycle_state = "ENABLED_HOLD"
    manager._state_machine = "MODE_TRANSITION"
    manager._error_codes = []
    manager._stop_control_loop = lambda: None
    manager._disable_all_motors = lambda: None
    manager._validated_joint_feedback = lambda **_kwargs: (
        np.zeros(6),
        np.zeros(6),
        np.zeros(6),
        [0] * 6,
    )
    manager._validated_gripper_status = lambda **_kwargs: 0

    assert manager.disable_immediately() is True
    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.state_machine == "IDLE"


def test_gripper_zero_requires_disabled_lifecycle() -> None:
    manager, gripper = _gripper_zero_manager()
    writes = []
    gripper.set_zero_position = lambda: writes.append(True)
    manager._enabled = True
    manager._lifecycle_state = "ENABLED_HOLD"

    with pytest.raises(RuntimeError, match="CONNECTED_DISABLED"):
        manager.set_zero("gripper")

    assert writes == []


def test_gripper_zero_accepts_three_new_disabled_near_zero_frames() -> None:
    manager, gripper = _gripper_zero_manager()

    def set_zero_position() -> None:
        gripper.state = SimpleNamespace(
            pos=0.0,
            vel=0.0,
            torq=0.0,
            status_code=0,
        )

    gripper.set_zero_position = set_zero_position
    before = gripper.sequence

    assert manager.set_zero("gripper") is True
    assert gripper.sequence >= before + 4
    assert manager._gripper_zero_error is None
    assert manager.lifecycle_state == "CONNECTED_DISABLED"


def test_failed_gripper_zero_remains_visible_and_blocks_position_use() -> None:
    manager, gripper = _gripper_zero_manager()
    gripper.set_zero_position = lambda: (_ for _ in ()).throw(
        RuntimeError("serial write failed")
    )

    with pytest.raises(RuntimeError, match="serial write failed"):
        manager.set_zero("gripper")

    assert "serial write failed" in manager._gripper_zero_error
    assert math.isnan(manager.gripper_position_m())
    assert any(
        "GRIPPER_FEEDBACK" in code and "serial write failed" in code
        for code in manager.error_codes
    )
