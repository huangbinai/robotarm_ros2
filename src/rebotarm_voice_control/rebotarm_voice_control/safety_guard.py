from __future__ import annotations

import math
from typing import Any

from .models import IntentCommand, SafetyViolationError, VoiceControlConfig


class SafetyGuard:
    def __init__(self, config: VoiceControlConfig):
        self._config = config
        self._allowed_commands = {
            "enable_arm",
            "disable_arm",
            "safe_home",
            "set_gripper",
            "move_named_pose",
            "move_relative",
            "pick_object",
            "place_object",
            "inspect_workspace",
            "confirm_action",
            "cancel_task",
            "run_task_template",
            "stop_motion",
        }

    def validate(self, command: IntentCommand) -> IntentCommand:
        if command.command not in self._allowed_commands:
            raise SafetyViolationError(f"command is not whitelisted: {command.command}")

        if command.command == "move_named_pose":
            self._validate_named_pose(command)

        if command.command == "run_task_template":
            name = command.params.get("name")
            if name not in self._config.task_templates:
                raise SafetyViolationError(f"task template does not exist: {name}")

        if command.command == "move_relative":
            distance_m = abs(self._finite_number(
                command.params.get("distance_m", 0.0), "move_relative distance_m"
            ))
            if distance_m > 0.05:
                raise SafetyViolationError("move_relative distance exceeds 0.05 m")
            if command.params.get("axis") not in {"x", "y", "z"}:
                raise SafetyViolationError("move_relative axis must be x, y, or z")

        if command.command == "set_gripper":
            position = self._finite_number(
                command.params.get("position"), "gripper position"
            )
            max_effort = self._finite_number(
                command.params.get("max_effort", 0.0), "gripper max_effort"
            )
            if not 0.0 <= position <= 0.085:
                raise SafetyViolationError("gripper position must be within [0.0, 0.085] m")
            if not 0.0 <= max_effort <= 1.5:
                raise SafetyViolationError("gripper max_effort must be within [0.0, 1.5]")

        return command

    def _validate_named_pose(self, command: IntentCommand) -> None:
        name = command.params.get("name")
        allowed = set(self._config.safety_limits.get("allowed_named_poses", []))
        if name not in allowed:
            raise SafetyViolationError(f"named pose is not allowed: {name}")
        pose = self._config.named_poses.get(str(name))
        if pose is None:
            raise SafetyViolationError(f"named pose does not exist: {name}")
        self._validate_workspace(str(name), pose)

    def _validate_workspace(self, name: str, pose: dict[str, Any]) -> None:
        position = pose.get("position")
        if not isinstance(position, list) or len(position) != 3:
            raise SafetyViolationError(f"named pose has invalid position: {name}")

        workspace = self._config.safety_limits.get("workspace", {})
        for axis, value in zip(("x", "y", "z"), position):
            bounds = workspace.get(axis)
            if not isinstance(bounds, list) or len(bounds) != 2:
                raise SafetyViolationError(f"workspace bounds missing for axis {axis}")
            low = self._finite_number(bounds[0], f"workspace {axis} lower bound")
            high = self._finite_number(bounds[1], f"workspace {axis} upper bound")
            coordinate = self._finite_number(value, f"named pose {name} {axis}")
            if low >= high:
                raise SafetyViolationError(f"workspace bounds are invalid for axis {axis}")
            if coordinate < low or coordinate > high:
                raise SafetyViolationError(
                    f"named pose {name} is outside workspace on {axis}: {value}"
                )

    @staticmethod
    def _finite_number(value: Any, label: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise SafetyViolationError(f"{label} must be numeric") from exc
        if not math.isfinite(number):
            raise SafetyViolationError(f"{label} must be finite")
        return number
