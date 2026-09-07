from __future__ import annotations

import threading

import yaml

from rebotarmcontroller.bus_synchronization import patch_arm_bus_lock
from rebotarmcontroller.hardware_runtime_config import HardwareRuntimeConfig
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
