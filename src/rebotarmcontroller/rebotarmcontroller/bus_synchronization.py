from __future__ import annotations

import threading
from collections.abc import Callable
from functools import wraps
from typing import Any


_CONTROLLER_METHODS = ("poll_feedback_once", "enable_all", "disable_all")
_MOTOR_METHODS = (
    "send_pos_vel",
    "send_mit",
    "send_vel",
    "request_feedback",
    "enable",
    "disable",
    "ensure_mode",
    "write_register_f32",
    "set_zero_position",
)


def _locked_call(method: Callable[..., Any], lock: threading.RLock):
    @wraps(method)
    def locked(*args, **kwargs):
        with lock:
            return method(*args, **kwargs)

    locked._rebotarm_locked = True
    return locked


def patch_controller_bus(controller: Any) -> None:
    if not hasattr(controller, "_bus_lock"):
        controller._bus_lock = threading.RLock()
    if hasattr(controller, "_bus_lock_patched"):
        return
    for name in _CONTROLLER_METHODS:
        if hasattr(controller, name):
            setattr(
                controller,
                name,
                _locked_call(getattr(controller, name), controller._bus_lock),
            )
    controller._bus_lock_patched = True


def wrap_motor_bus(motor: Any, lock: threading.RLock) -> None:
    for name in _MOTOR_METHODS:
        if not hasattr(motor, name):
            continue
        method = getattr(motor, name)
        if hasattr(method, "_rebotarm_locked"):
            continue
        setattr(motor, name, _locked_call(method, lock))


def patch_arm_bus_lock(arm: Any) -> None:
    for controller in arm._ctrl_map.values():
        patch_controller_bus(controller)
    if hasattr(arm, "_bus_lock_patched"):
        return
    for joint_config in arm._joints:
        motor = arm._motor_map[joint_config.name]
        controller = arm._ctrl_map[joint_config.vendor]
        wrap_motor_bus(motor, controller._bus_lock)
    arm._bus_lock_patched = True
