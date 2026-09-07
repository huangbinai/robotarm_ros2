from __future__ import annotations

import sys
from types import ModuleType, SimpleNamespace

tf_transformations = ModuleType("tf_transformations")
tf_transformations.euler_from_quaternion = lambda _q: (0.0, 0.0, 0.0)
tf_transformations.quaternion_from_matrix = lambda _m: (0.0, 0.0, 0.0, 1.0)
sys.modules.setdefault("tf_transformations", tf_transformations)

from rebotarmcontroller.hardware_manager import HardwareManager


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
