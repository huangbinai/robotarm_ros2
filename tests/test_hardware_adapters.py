from __future__ import annotations

import threading
from types import SimpleNamespace

import yaml

from rebotarmcontroller.bus_synchronization import patch_arm_bus_lock
from rebotarmcontroller.hardware_runtime_config import HardwareRuntimeConfig
from rebotarmcontroller.hardware_sdk_runtime import create_hardware_sdk_runtime
from rebotarmcontroller.sdk_runtime import RebotSdkLocator


class _Controller:
    def __init__(self) -> None:
        self.calls = []

    def poll_feedback_once(self):
        self.calls.append("poll")

    def enable_all(self):
        self.calls.append("enable")

    def disable_all(self):
        self.calls.append("disable")


class _Motor:
    def __init__(self) -> None:
        self.calls = []

    def send_mit(self, *values):
        self.calls.append(("mit", values))

    def request_feedback(self):
        self.calls.append(("feedback", ()))


class _JointConfig:
    name = "joint1"
    vendor = "vendor"


class _Arm:
    def __init__(self) -> None:
        self.controller = _Controller()
        self.motor = _Motor()
        self._ctrl_map = {"vendor": self.controller}
        self._motor_map = {"joint1": self.motor}
        self._joints = [_JointConfig()]


def test_sdk_locator_accepts_canonical_and_main_suffix_directories(tmp_path) -> None:
    workspace = tmp_path / "workspace"
    sdk_root = workspace / "third_party" / "reBotArm_control_py-main"
    (sdk_root / "reBotArm_control_py").mkdir(parents=True)
    import_path = []
    locator = RebotSdkLocator(
        workspace_root=workspace,
        cwd=tmp_path / "cwd",
        user_home=tmp_path / "home",
        import_path=import_path,
    )

    selected = locator.ensure_importable()

    assert selected == sdk_root
    assert import_path == [str(sdk_root)]
    assert locator.arm_config(selected) == sdk_root / "config" / "arm.yaml"
    assert locator.gripper_config(selected) == sdk_root / "config" / "gripper.yaml"


def test_sdk_channel_override_is_runtime_only(tmp_path) -> None:
    config_path = tmp_path / "arm.yaml"
    config_path.write_text("channel: can0\nrate: 500\n", encoding="utf-8")

    unchanged = RebotSdkLocator.with_channel_override(
        config_path,
        "auto",
        temporary_directory=tmp_path / "runtime",
    )
    override = RebotSdkLocator.with_channel_override(
        config_path,
        "can1",
        temporary_directory=tmp_path / "runtime",
    )

    assert unchanged == config_path
    assert yaml.safe_load(config_path.read_text(encoding="utf-8"))["channel"] == "can0"
    assert yaml.safe_load(override.read_text(encoding="utf-8")) == {
        "channel": "can1",
        "rate": 500,
    }


def test_hardware_sdk_runtime_uses_default_paths_and_defers_endpos(tmp_path) -> None:
    workspace = tmp_path / "workspace"
    sdk_root = workspace / "third_party" / "reBotArm_control_py-main"
    (sdk_root / "reBotArm_control_py").mkdir(parents=True)
    locator = RebotSdkLocator(
        workspace_root=workspace,
        cwd=tmp_path / "cwd",
        user_home=tmp_path / "home",
        import_path=[],
    )
    events = []

    class FakeArm:
        def __init__(self, *, cfg_path):
            self.cfg_path = cfg_path
            events.append(("arm", cfg_path))

    class FakeModel:
        def createData(self):
            events.append(("create_data",))
            return "gravity-data"

        def getFrameId(self, frame):
            events.append(("frame", frame))
            return 17

    def load_robot_model():
        events.append(("load_model",))
        return FakeModel()

    def create_endpos(arm):
        events.append(("endpos", arm))
        return ("endpos-controller", arm)

    def compute_generalized_gravity(*, q):
        return q

    def compute_fk(model, positions):
        return model, positions, "transform"

    pinocchio = object()
    modules = {
        "reBotArm_control_py.actuator": SimpleNamespace(RobotArm=FakeArm),
        "reBotArm_control_py.controllers": SimpleNamespace(ArmEndPos=create_endpos),
        "reBotArm_control_py.kinematics": SimpleNamespace(
            load_robot_model=load_robot_model,
            compute_fk=compute_fk,
        ),
        "reBotArm_control_py.dynamics": SimpleNamespace(
            compute_generalized_gravity=compute_generalized_gravity
        ),
        "pinocchio": pinocchio,
    }

    runtime = create_hardware_sdk_runtime(
        module_path=tmp_path / "unused.py",
        arm_config=None,
        gripper_config=None,
        channel="auto",
        end_effector_frame="end_link",
        locator=locator,
        module_importer=modules.__getitem__,
    )

    expected_arm_config = sdk_root / "config" / "arm.yaml"
    assert runtime.sdk_root == sdk_root
    assert runtime.arm_config_path == expected_arm_config
    assert runtime.arm.cfg_path == str(expected_arm_config)
    assert runtime.gripper_config_path == sdk_root / "config" / "gripper.yaml"
    assert runtime.gravity_dynamics.data == "gravity-data"
    assert runtime.gravity_dynamics.end_effector_frame_id == 17
    assert (
        runtime.gravity_dynamics.compute_generalized_gravity
        is compute_generalized_gravity
    )
    assert runtime.gravity_dynamics.pinocchio is pinocchio
    assert runtime.forward_kinematics("model", "positions") == (
        "model",
        "positions",
        "transform",
    )
    assert [event[0] for event in events] == ["arm", "load_model", "create_data", "frame"]

    endpos = runtime.create_endpos_controller()

    assert endpos == ("endpos-controller", runtime.arm)
    assert events[-1] == ("endpos", runtime.arm)


def test_hardware_sdk_runtime_prefers_explicit_configs_and_channel_override(
    tmp_path,
) -> None:
    sdk_root = tmp_path / "sdk"
    arm_config = tmp_path / "explicit-arm.yaml"
    gripper_config = tmp_path / "explicit-gripper.yaml"
    override_config = tmp_path / "runtime-arm.yaml"
    calls = []

    class FakeLocator:
        def ensure_importable(self):
            calls.append(("ensure_importable",))
            return sdk_root

        def arm_config(self, root):
            raise AssertionError(f"default arm config requested for {root}")

        def gripper_config(self, root):
            raise AssertionError(f"default gripper config requested for {root}")

        def with_channel_override(self, path, channel):
            calls.append(("channel_override", path, channel))
            return override_config

    class FakeArm:
        def __init__(self, *, cfg_path):
            calls.append(("arm", cfg_path))

    class FakeModel:
        def createData(self):
            return object()

        def getFrameId(self, frame):
            return frame

    modules = {
        "reBotArm_control_py.actuator": SimpleNamespace(RobotArm=FakeArm),
        "reBotArm_control_py.controllers": SimpleNamespace(ArmEndPos=lambda arm: arm),
        "reBotArm_control_py.kinematics": SimpleNamespace(
            load_robot_model=FakeModel,
            compute_fk=lambda model, positions: (model, positions, None),
        ),
        "reBotArm_control_py.dynamics": SimpleNamespace(
            compute_generalized_gravity=lambda **kwargs: kwargs
        ),
        "pinocchio": object(),
    }

    runtime = create_hardware_sdk_runtime(
        module_path=tmp_path / "unused.py",
        arm_config=arm_config,
        gripper_config=gripper_config,
        channel="can1",
        end_effector_frame="end_link",
        locator=FakeLocator(),
        module_importer=modules.__getitem__,
    )

    assert calls == [
        ("ensure_importable",),
        ("channel_override", arm_config, "can1"),
        ("arm", str(override_config)),
    ]
    assert runtime.arm_config_path == override_config
    assert runtime.gripper_config_path == gripper_config


def test_bus_synchronization_wraps_controller_and_motor_once() -> None:
    arm = _Arm()

    patch_arm_bus_lock(arm)
    first_controller_method = arm.controller.poll_feedback_once
    first_motor_method = arm.motor.send_mit
    patch_arm_bus_lock(arm)

    arm.controller.poll_feedback_once()
    arm.motor.send_mit(1.0, 2.0)

    assert isinstance(arm.controller._bus_lock, type(threading.RLock()))
    assert arm.controller.poll_feedback_once == first_controller_method
    assert arm.motor.send_mit == first_motor_method
    assert arm.controller.calls == ["poll"]
    assert arm.motor.calls == [("mit", (1.0, 2.0))]
    assert arm._bus_lock_patched is True


def test_hardware_runtime_config_validates_without_loading_sdk() -> None:
    config = HardwareRuntimeConfig.validate(
        hardware_feedback_rate_hz=50.0,
        feedback_stale_timeout_sec=0.15,
        gripper_position_torque_cap_nm=1.0,
        gripper_position_max_speed_rad_s=0.5,
        gripper_position_timeout_margin_sec=1.5,
        grasp_hold_timeout_sec=30.0,
        gripper_contact_torque_min_nm=0.0,
    )

    assert config.hardware_feedback_period_sec == 0.02


def test_hardware_runtime_config_rejects_nonfinite_and_out_of_range_values() -> None:
    valid = {
        "hardware_feedback_rate_hz": 50.0,
        "feedback_stale_timeout_sec": 0.15,
        "gripper_position_torque_cap_nm": 1.0,
        "gripper_position_max_speed_rad_s": 0.5,
        "gripper_position_timeout_margin_sec": 1.5,
        "grasp_hold_timeout_sec": 30.0,
        "gripper_contact_torque_min_nm": 0.0,
    }
    for name, value in (
        ("hardware_feedback_rate_hz", 10.0),
        ("feedback_stale_timeout_sec", float("nan")),
        ("gripper_position_torque_cap_nm", 2.0),
        ("gripper_position_max_speed_rad_s", 0.0),
        ("gripper_position_timeout_margin_sec", 20.0),
        ("grasp_hold_timeout_sec", 0.0),
        ("gripper_contact_torque_min_nm", -0.1),
    ):
        values = {**valid, name: value}
        try:
            HardwareRuntimeConfig.validate(**values)
        except ValueError as exc:
            assert name in str(exc)
        else:
            raise AssertionError(f"{name} accepted invalid value {value}")
