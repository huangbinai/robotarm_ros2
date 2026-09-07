from __future__ import annotations

import sys
from types import ModuleType, SimpleNamespace

import pytest

tf_transformations = ModuleType("tf_transformations")
tf_transformations.euler_from_quaternion = lambda _q: (0.0, 0.0, 0.0)
tf_transformations.quaternion_from_matrix = lambda _m: (0.0, 0.0, 0.0, 1.0)
sys.modules.setdefault("tf_transformations", tf_transformations)

from rebotarmcontroller.hardware_manager import HardwareManager
from rebotarmcontroller.gripper_sdk_adapter import GripperSdkAdapter


@pytest.mark.parametrize(
    ("vendor", "method_name"),
    (
        ("damiao", "add_damiao_motor"),
        ("myactuator", "add_myactuator_motor"),
        ("robstride", "add_robstride_motor"),
    ),
)
def test_gripper_sdk_adapter_creates_supported_vendor_motor(
    vendor: str,
    method_name: str,
) -> None:
    calls = []
    motor = object()

    def create_motor(*args):
        calls.append(args)
        return motor

    controller = SimpleNamespace(**{method_name: create_motor})
    config = SimpleNamespace(
        vendor=vendor,
        motor_id=7,
        feedback_id=23,
        model="4310",
    )
    adapter = GripperSdkAdapter(SimpleNamespace(_ctrl_map={vendor: controller}))

    selected_controller = adapter.controller_for(config)
    selected_motor = adapter.create_motor(selected_controller, config)

    assert selected_controller is controller
    assert selected_motor is motor
    assert calls == [(7, 23, "4310")]


def test_gripper_sdk_adapter_preserves_vendor_failures() -> None:
    config = SimpleNamespace(
        vendor="unknown",
        motor_id=7,
        feedback_id=23,
        model="4310",
    )
    missing_controller_adapter = GripperSdkAdapter(SimpleNamespace(_ctrl_map={}))

    with pytest.raises(RuntimeError, match="cannot share the arm Controller"):
        missing_controller_adapter.controller_for(config)

    controller = SimpleNamespace()
    configured_adapter = GripperSdkAdapter(
        SimpleNamespace(_ctrl_map={"unknown": controller})
    )
    with pytest.raises(ValueError, match="unsupported gripper vendor"):
        configured_adapter.create_motor(controller, config)


def test_gripper_initialization_uses_shared_controller_bus_helpers(
    monkeypatch,
) -> None:
    motor = SimpleNamespace(send_mit=lambda *_args: None)
    controller = SimpleNamespace(
        add_damiao_motor=lambda *_args: motor,
        poll_feedback_once=lambda: None,
        enable_all=lambda: None,
        disable_all=lambda: None,
    )
    config = SimpleNamespace(
        vendor="damiao",
        motor_id=7,
        feedback_id=23,
        model="4310",
    )
    gripper_module = ModuleType("reBotArm_control_py.actuator.gripper")
    gripper_module.load_cfg = lambda _path: {"gripper": config}
    actuator_module = ModuleType("reBotArm_control_py.actuator")
    actuator_module.gripper = gripper_module
    root_module = ModuleType("reBotArm_control_py")
    root_module.actuator = actuator_module
    monkeypatch.setitem(sys.modules, "reBotArm_control_py", root_module)
    monkeypatch.setitem(sys.modules, "reBotArm_control_py.actuator", actuator_module)
    monkeypatch.setitem(
        sys.modules,
        "reBotArm_control_py.actuator.gripper",
        gripper_module,
    )

    manager = object.__new__(HardwareManager)
    manager._arm = SimpleNamespace(_ctrl_map={"damiao": controller})

    manager.init_gripper("gripper.yaml")

    assert manager._gripper_cfg is config
    assert manager._gripper_mot is motor
    assert manager._gripper_ctrl is controller
    assert hasattr(controller, "_bus_lock")
    assert motor.send_mit._rebotarm_locked is True
