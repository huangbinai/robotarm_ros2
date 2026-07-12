from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from rebotarm_simulation.urdf_to_mjcf import (
    authoritative_urdf_path,
    generate_mjcf_bytes,
    stage_urdf,
)


ROOT = Path(__file__).resolve().parents[1]


def test_authoritative_urdf_is_moveit_model() -> None:
    assert authoritative_urdf_path(ROOT) == (
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    )


def test_stage_urdf_rewrites_package_meshes_without_modifying_source(tmp_path) -> None:
    source = authoritative_urdf_path(ROOT)
    before = source.read_bytes()

    staged = stage_urdf(source, ROOT, tmp_path)

    text = staged.read_text(encoding="utf-8")
    assert "package://" not in text
    assert "assets/base_link.STL" in text
    assert (tmp_path / "assets/base_link.STL").is_file()
    assert source.read_bytes() == before


def test_generation_is_byte_deterministic() -> None:
    assert generate_mjcf_bytes(ROOT) == generate_mjcf_bytes(ROOT)


def _generated_root() -> ET.Element:
    return ET.fromstring(generate_mjcf_bytes(ROOT))


def test_generated_model_preserves_named_bodies_and_joint_interface() -> None:
    root = _generated_root()
    body_names = {body.attrib["name"] for body in root.findall("worldbody//body")}
    joint_names = {joint.attrib["name"] for joint in root.findall("worldbody//joint")}

    assert body_names == {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "end_link", "left_finger_link", "right_finger_link",
    }
    assert joint_names == {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        "left_finger_joint", "right_finger_joint",
    }


def test_generated_model_separates_original_mesh_visuals_and_collisions() -> None:
    root = _generated_root()
    expected_meshes = {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "gripper_base", "left_finger", "right_finger",
    }
    visuals = root.findall('.//geom[@class="visual"]')
    collisions = root.findall('.//geom[@class="collision"]')

    assert {geom.attrib["mesh"] for geom in visuals} == expected_meshes
    assert {geom.attrib["mesh"] for geom in collisions} == expected_meshes
    assert all(geom.attrib.get("contype") == "0" for geom in visuals)
    assert all(geom.attrib.get("conaffinity") == "0" for geom in visuals)
    assert all(geom.attrib.get("contype") == "1" for geom in collisions)
    assert all(geom.attrib.get("conaffinity") == "1" for geom in collisions)


def test_generated_model_adds_mujoco_control_and_observation_contract() -> None:
    root = _generated_root()
    joints = [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        "left_finger_joint", "right_finger_joint",
    ]
    actuators = root.findall("actuator/position")
    assert [actuator.attrib["joint"] for actuator in actuators] == joints
    assert all(float(actuator.attrib["kp"]) > 0 for actuator in actuators)
    assert all(float(actuator.attrib["kv"]) > 0 for actuator in actuators)

    coupling = root.find("equality/joint")
    assert coupling is not None
    assert coupling.attrib["joint1"] == "right_finger_joint"
    assert coupling.attrib["joint2"] == "left_finger_joint"
    assert coupling.attrib["polycoef"] == "0 -1 0 0 0"

    exclusions = root.findall("contact/exclude")
    assert len(exclusions) == 9
    assert root.find('.//site[@name="ee_site"]') is not None
    assert root.find('.//site[@name="wrist_camera_mount"]') is not None
    assert len(root.findall("sensor/jointpos")) == 8
    assert len(root.findall("sensor/jointvel")) == 8
    assert len(root.findall("sensor/actuatorfrc")) == 8
    assert root.find('sensor/framepos[@objname="ee_site"]') is not None
    assert root.find('sensor/framequat[@objname="ee_site"]') is not None
