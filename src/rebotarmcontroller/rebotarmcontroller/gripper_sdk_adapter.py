from __future__ import annotations

from typing import Any

from .bus_synchronization import patch_controller_bus, wrap_motor_bus


class GripperSdkAdapter:
    """Create a gripper motor on the arm controller's shared SDK bus."""

    def __init__(self, arm: Any) -> None:
        self._arm = arm

    @staticmethod
    def load_config(config_path: str) -> Any:
        from reBotArm_control_py.actuator.gripper import load_cfg

        return load_cfg(config_path)["gripper"]

    def controller_for(self, config: Any) -> Any:
        vendor = config.vendor
        if vendor not in self._arm._ctrl_map:
            raise RuntimeError(
                f"gripper vendor={vendor!r} cannot share the arm Controller"
            )
        return self._arm._ctrl_map[vendor]

    @staticmethod
    def create_motor(controller: Any, config: Any) -> Any:
        vendor = config.vendor
        motor_args = (config.motor_id, config.feedback_id, config.model)
        if vendor == "damiao":
            return controller.add_damiao_motor(*motor_args)
        if vendor == "myactuator":
            return controller.add_myactuator_motor(*motor_args)
        if vendor == "robstride":
            return controller.add_robstride_motor(*motor_args)
        raise ValueError(f"unsupported gripper vendor: {vendor!r}")

    @staticmethod
    def share_controller_bus(controller: Any, motor: Any) -> None:
        patch_controller_bus(controller)
        wrap_motor_bus(motor, controller._bus_lock)
