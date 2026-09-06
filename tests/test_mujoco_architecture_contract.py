from __future__ import annotations

from pathlib import Path

import pytest

from rebotarm_simulation.model_contract import (
    ARM_JOINT_NAMES,
    FINGER_JOINT_NAMES,
    HOME_JOINT_POSITIONS,
    JOINT_NAMES,
    actuator_name_for_joint,
)
from rebotarm_simulation.simulation_config import SimulationConfig
from rebotarm_simulation.simulation_protocol import SimulationProtocol


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"


def test_model_contract_has_one_canonical_joint_order_and_actuator_mapping() -> None:
    assert JOINT_NAMES == ARM_JOINT_NAMES + FINGER_JOINT_NAMES
    assert len(ARM_JOINT_NAMES) == len(HOME_JOINT_POSITIONS) == 6
    assert tuple(actuator_name_for_joint(name) for name in JOINT_NAMES) == (
        "joint1_torque",
        "joint2_torque",
        "joint3_torque",
        "joint4_torque",
        "joint5_torque",
        "joint6_torque",
        "left_finger_force",
        "right_finger_force",
    )
    with pytest.raises(ValueError, match="unknown reBotArm joint"):
        actuator_name_for_joint("joint7")


def test_default_simulation_config_resolves_source_workspace_resources() -> None:
    config = SimulationConfig.default(SCENE)

    assert config.model_path == SCENE.resolve()
    assert config.arm_config_path.name == "arm.yaml"
    assert config.gripper_config_path.name == "gripper.yaml"
    assert config.motor_calibration_path.name == "motor_control_calibration.yaml"
    assert config.robot_urdf_path.name == "rebotarm.urdf"
    assert all(path.is_file() for path in (
        config.model_path,
        config.arm_config_path,
        config.gripper_config_path,
        config.motor_calibration_path,
        config.robot_urdf_path,
    ))


def test_runtime_satisfies_backend_neutral_protocol_and_adapters_expire() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.mujoco_sim import RebotArmMujoco

    simulation = RebotArmMujoco(SCENE)
    render = simulation.render_adapter
    kinematics = simulation.kinematics_adapter
    assert isinstance(simulation, SimulationProtocol)
    assert render.model is kinematics.model
    assert render.data is kinematics.data

    simulation.close()
    with pytest.raises(RuntimeError, match="closed"):
        render.handles()
    with pytest.raises(RuntimeError, match="closed"):
        kinematics.handles()


def test_runtime_does_not_import_model_converter() -> None:
    source = (
        ROOT / "src/rebotarm_simulation/rebotarm_simulation/mujoco_sim.py"
    ).read_text(encoding="utf-8")
    assert "urdf_to_mjcf" not in source
    assert "parents[3]" not in source
