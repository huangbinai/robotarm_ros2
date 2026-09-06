"""Portable resource configuration for the MuJoCo runtime."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import sys

from .model_contract import DEFAULT_SCENE_RESOURCE


def _package_resource(package_name: str, relative_path: str) -> Path:
    relative = Path(relative_path)
    package_project = Path(__file__).resolve().parents[1]
    candidates = [
        package_project.parent / package_name / relative,
        Path(sys.prefix) / "share" / package_name / relative,
    ]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "share" / package_name / relative)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        f"Could not locate {package_name}/{relative.as_posix()}; searched: {searched}"
    )


@dataclass(frozen=True)
class SimulationConfig:
    """All filesystem inputs required to construct a simulation.

    Supplying this value makes the core independent of a repository root.  The
    default resolver supports both a source workspace and installed ROS package
    shares; applications may inject arbitrary paths for tests or deployment.
    """

    model_path: Path
    arm_config_path: Path
    gripper_config_path: Path
    motor_calibration_path: Path
    robot_urdf_path: Path

    def __post_init__(self) -> None:
        for field_name in (
            "model_path",
            "arm_config_path",
            "gripper_config_path",
            "motor_calibration_path",
            "robot_urdf_path",
        ):
            object.__setattr__(self, field_name, Path(getattr(self, field_name)).resolve())

    @classmethod
    def default(cls, model_path: str | os.PathLike[str] | None = None) -> "SimulationConfig":
        return cls(
            model_path=(
                Path(model_path).resolve()
                if model_path is not None
                else _package_resource("rebotarm_simulation", DEFAULT_SCENE_RESOURCE)
            ),
            arm_config_path=_package_resource("rebotarm_bringup", "config/arm.yaml"),
            gripper_config_path=_package_resource("rebotarm_bringup", "config/gripper.yaml"),
            motor_calibration_path=_package_resource(
                "rebotarm_simulation", "config/motor_control_calibration.yaml"
            ),
            robot_urdf_path=_package_resource(
                "rebotarm_moveit_config", "config/rebotarm.urdf"
            ),
        )
