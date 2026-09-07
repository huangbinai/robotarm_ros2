from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from .sdk_runtime import RebotSdkLocator


ModuleImporter = Callable[[str], Any]


@dataclass(frozen=True)
class HardwareSdkRuntime:
    """SDK objects created once for a hardware-manager instance."""

    sdk_root: Path
    arm_config_path: Path
    arm: Any
    gripper_config_path: Path
    gravity_model: Any
    gravity_data: Any
    gravity_end_effector_frame_id: Any
    compute_generalized_gravity: Callable[..., Any]
    pinocchio: Any
    _endpos_controller_factory: Callable[[Any], Any] = field(repr=False)

    def create_endpos_controller(self) -> Any:
        """Create the controller only when manager initialization reaches that stage."""

        return self._endpos_controller_factory(self.arm)


def create_hardware_sdk_runtime(
    *,
    module_path: str | Path,
    arm_config: str | Path | None,
    gripper_config: str | Path | None,
    channel: str,
    end_effector_frame: str,
    locator: RebotSdkLocator | None = None,
    module_importer: ModuleImporter = importlib.import_module,
) -> HardwareSdkRuntime:
    """Locate and assemble SDK dependencies without exposing imports to the manager."""

    sdk_locator = locator or RebotSdkLocator.for_module(module_path)
    sdk_root = sdk_locator.ensure_importable()

    actuator_module = module_importer("reBotArm_control_py.actuator")
    controllers_module = module_importer("reBotArm_control_py.controllers")
    kinematics_module = module_importer("reBotArm_control_py.kinematics")
    dynamics_module = module_importer("reBotArm_control_py.dynamics")
    pinocchio_module = module_importer("pinocchio")

    arm_config_path = (
        Path(arm_config).expanduser()
        if arm_config
        else sdk_locator.arm_config(sdk_root)
    )
    arm_config_path = sdk_locator.with_channel_override(arm_config_path, channel)
    arm = actuator_module.RobotArm(cfg_path=str(arm_config_path))

    gravity_model = kinematics_module.load_robot_model()
    gravity_data = gravity_model.createData()
    gravity_end_effector_frame_id = gravity_model.getFrameId(end_effector_frame)

    gripper_config_path = (
        Path(gripper_config).expanduser()
        if gripper_config
        else sdk_locator.gripper_config(sdk_root)
    )
    return HardwareSdkRuntime(
        sdk_root=sdk_root,
        arm_config_path=arm_config_path,
        arm=arm,
        gripper_config_path=gripper_config_path,
        gravity_model=gravity_model,
        gravity_data=gravity_data,
        gravity_end_effector_frame_id=gravity_end_effector_frame_id,
        compute_generalized_gravity=dynamics_module.compute_generalized_gravity,
        pinocchio=pinocchio_module,
        _endpos_controller_factory=controllers_module.ArmEndPos,
    )
