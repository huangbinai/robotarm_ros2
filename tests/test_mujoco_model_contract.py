from __future__ import annotations

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"
MODEL_DIR = ROOT / "src/rebotarm_simulation/models/rebotarm"
ROBOT_PATH = MODEL_DIR / "robot.xml"
SCENE_PATH = MODEL_DIR / "scene.xml"

ARM_JOINTS = [f"joint{index}" for index in range(1, 7)]
FINGER_JOINTS = ["left_finger_joint", "right_finger_joint"]
JOINTS = ARM_JOINTS + FINGER_JOINTS
LINKS = ["base_link"] + [f"link{index}" for index in range(1, 7)] + ["end_link"]


def _numbers(value: str) -> tuple[float, ...]:
    return tuple(float(part) for part in value.split())


def _assert_close(actual: str, expected: str) -> None:
    assert _numbers(actual) == pytest.approx(_numbers(expected), abs=1e-7)


def _robot_root() -> ET.Element:
    return ET.parse(ROBOT_PATH).getroot()


def _urdf_root() -> ET.Element:
    return ET.parse(URDF_PATH).getroot()


def _inertia_components(inertia: ET.Element) -> tuple[float, ...]:
    return tuple(
        float(inertia.attrib[name])
        for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    )


def _is_positive_definite(components: tuple[float, ...]) -> bool:
    ixx, iyy, izz, ixy, ixz, iyz = components
    determinant = (
        ixx * iyy * izz
        + 2 * ixy * ixz * iyz
        - ixx * iyz * iyz
        - iyy * ixz * ixz
        - izz * ixy * ixy
    )
    return ixx > 0 and ixx * iyy - ixy * ixy > 0 and determinant > 0


def _matmul(left: tuple[tuple[float, ...], ...], right: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _urdf_origin_matrix(origin: ET.Element) -> tuple[tuple[float, ...], ...]:
    x, y, z = _numbers(origin.attrib["xyz"])
    roll, pitch, yaw = _numbers(origin.attrib["rpy"])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def test_robot_model_uses_urdf_extrinsic_xyz_euler_convention() -> None:
    compiler = _robot_root().find("compiler")
    assert compiler is not None
    assert compiler.attrib["angle"] == "radian"
    assert compiler.attrib["eulerseq"] == "XYZ"


def test_robot_model_preserves_canonical_joint_chain_transforms_axes_and_ranges() -> None:
    robot = _robot_root()
    urdf = _urdf_root()
    urdf_joints = {joint.attrib["name"]: joint for joint in urdf.findall("joint")}
    mjcf_joints = {joint.attrib["name"]: joint for joint in robot.findall("worldbody//joint")}

    assert list(mjcf_joints) == JOINTS
    for name in JOINTS:
        source = urdf_joints[name]
        target = mjcf_joints[name]
        body = next(body for body in robot.findall(".//body") if target in list(body))
        _assert_close(body.attrib["pos"], source.find("origin").attrib["xyz"])
        _assert_close(body.attrib["euler"], source.find("origin").attrib["rpy"])
        _assert_close(target.attrib["axis"], source.find("axis").attrib["xyz"])
        limit = source.find("limit")
        _assert_close(target.attrib["range"], f'{limit.attrib["lower"]} {limit.attrib["upper"]}')
        assert target.attrib["type"] == ("slide" if name in FINGER_JOINTS else "hinge")
        assert target.attrib["limited"] == "true"
        assert "damping" not in target.attrib
        assert "armature" not in target.attrib


def test_robot_model_preserves_link_masses_and_has_valid_explicit_finger_inertials() -> None:
    robot = _robot_root()
    urdf = _urdf_root()
    source_masses = {
        link.attrib["name"]: float(link.find("inertial/mass").attrib["value"])
        for link in urdf.findall("link")
        if link.find("inertial/mass") is not None
    }
    bodies = {body.attrib["name"]: body for body in robot.findall(".//body")}

    for link_name in LINKS:
        inertial = bodies[link_name].find("inertial")
        assert inertial is not None
        assert float(inertial.attrib["mass"]) == pytest.approx(source_masses[link_name])
        source = next(link for link in urdf.findall("link") if link.attrib["name"] == link_name)
        expected_tensor = _inertia_components(source.find("inertial/inertia"))
        actual_tensor = _numbers(inertial.attrib["fullinertia"])
        assert actual_tensor == pytest.approx(expected_tensor, abs=1e-12)
        assert _is_positive_definite(actual_tensor)

    for finger in ("left_finger_link", "right_finger_link"):
        inertial = bodies[finger].find("inertial")
        assert inertial is not None
        assert float(inertial.attrib["mass"]) > 0
        diagonal = _numbers(inertial.attrib["diaginertia"])
        assert all(value > 0 for value in diagonal)
        assert _is_positive_definite((*diagonal, 0.0, 0.0, 0.0))


def test_robot_model_couples_finger_joints_with_equal_and_opposite_motion() -> None:
    coupling = _robot_root().find("equality/joint")
    assert coupling is not None
    assert coupling.attrib == {
        "name": "finger_coupling",
        "joint1": "right_finger_joint",
        "joint2": "left_finger_joint",
        "polycoef": "0 -1 0 0 0",
    }


def test_robot_model_separates_mesh_visuals_from_primitive_collisions() -> None:
    robot = _robot_root()
    assets = {mesh.attrib["name"]: mesh for mesh in robot.findall("asset/mesh")}
    expected_assets = {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "gripper_base", "left_finger", "right_finger",
    }
    assert set(assets) == expected_assets
    mesh_dir = robot.find("compiler").attrib["meshdir"]
    for mesh in assets.values():
        path = MODEL_DIR / mesh_dir / mesh.attrib["file"]
        assert path.is_file()

    visual_geoms = robot.findall('.//geom[@class="visual"]')
    collision_geoms = robot.findall('.//geom[@class="collision"]')
    assert {geom.attrib["mesh"] for geom in visual_geoms} == expected_assets
    assert collision_geoms
    assert all("mesh" not in geom.attrib for geom in collision_geoms)
    assert all(geom.attrib.get("contype") == "1" for geom in collision_geoms)
    assert all(geom.attrib.get("conaffinity") == "1" for geom in collision_geoms)


def test_robot_model_filters_adjacent_chain_and_gripper_parent_contacts() -> None:
    robot = _robot_root()
    excluded_pairs = {
        frozenset((exclude.attrib["body1"], exclude.attrib["body2"]))
        for exclude in robot.findall("contact/exclude")
    }
    expected_pairs = {
        frozenset(pair)
        for pair in (
            ("base_link", "link1"),
            ("link1", "link2"),
            ("link2", "link3"),
            ("link3", "link4"),
            ("link4", "link5"),
            ("link5", "link6"),
            ("link6", "end_link"),
            ("end_link", "left_finger_link"),
            ("end_link", "right_finger_link"),
        )
    }

    assert excluded_pairs == expected_pairs


def test_robot_model_has_eight_position_actuators_end_effector_site_and_state_sensors() -> None:
    robot = _robot_root()
    actuators = robot.findall("actuator/position")
    assert [actuator.attrib["joint"] for actuator in actuators] == JOINTS
    assert all(float(actuator.attrib["kp"]) > 0 for actuator in actuators)

    ee_site = robot.find('.//site[@name="ee_site"]')
    assert ee_site is not None
    sensors = robot.find("sensor")
    assert sensors is not None
    assert {element.attrib["joint"] for element in sensors.findall("jointpos")} == set(JOINTS)
    assert {element.attrib["joint"] for element in sensors.findall("jointvel")} == set(JOINTS)
    assert sensors.find('framepos[@objname="ee_site"]') is not None
    assert sensors.find('framequat[@objname="ee_site"]') is not None


def test_position_actuator_force_limits_match_urdf_effort_limits() -> None:
    robot = _robot_root()
    urdf = _urdf_root()
    efforts = {
        joint.attrib["name"]: float(joint.find("limit").attrib["effort"])
        for joint in urdf.findall("joint")
        if joint.attrib["name"] in JOINTS
    }

    actuators = {actuator.attrib["joint"]: actuator for actuator in robot.findall("actuator/position")}
    assert set(actuators) == set(JOINTS)
    for joint_name, effort in efforts.items():
        actuator = actuators[joint_name]
        assert actuator.attrib["forcelimited"] == "true"
        assert _numbers(actuator.attrib["forcerange"]) == pytest.approx((-effort, effort))


def test_scene_defines_world_fixture_free_cube_camera_and_simulation_options() -> None:
    scene = ET.parse(SCENE_PATH).getroot()
    include = scene.find("include")
    assert include is not None and include.attrib["file"] == "robot.xml"
    option = scene.find("option")
    _assert_close(option.attrib["gravity"], "0 0 -9.81")
    assert float(option.attrib["timestep"]) == pytest.approx(0.002)
    world = scene.find("worldbody")
    assert world.find('geom[@name="floor"]') is not None
    assert world.find('body[@name="table"]/geom') is not None
    cube = world.find('body[@name="test_cube"]')
    assert cube is not None and cube.find("freejoint") is not None
    assert cube.find("geom").attrib["type"] == "box"
    assert world.find('light[@name="key_light"]') is not None
    assert world.find('camera[@name="fixed_camera"]') is not None


def test_scene_compiles_with_mujoco_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert model.nu == 8
    assert data.time == pytest.approx(100 * model.opt.timestep)
    assert all(math.isfinite(float(value)) for value in data.qpos)
    assert all(math.isfinite(float(value)) for value in data.qvel)
    assert all(math.isfinite(float(value)) for value in data.actuator_force)
    assert max((abs(float(value)) for value in data.qvel), default=0.0) < 1_000.0


def test_mujoco_zero_pose_fk_matches_independent_urdf_rpy_math_when_runtime_is_available() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    urdf = _urdf_root()
    identity = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    transforms = {"base_link": identity}
    remaining = list(urdf.findall("joint"))
    while remaining:
        progressed = False
        for joint in remaining[:]:
            parent = joint.find("parent").attrib["link"]
            if parent not in transforms:
                continue
            child = joint.find("child").attrib["link"]
            transforms[child] = _matmul(transforms[parent], _urdf_origin_matrix(joint.find("origin")))
            remaining.remove(joint)
            progressed = True
        assert progressed, "URDF joint graph must be connected"

    for body_name, expected in transforms.items():
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        assert body_id >= 0
        assert tuple(float(value) for value in data.xpos[body_id]) == pytest.approx(
            (expected[0][3], expected[1][3], expected[2][3]), abs=1e-6
        )
        expected_rotation = tuple(expected[row][column] for row in range(3) for column in range(3))
        assert tuple(float(value) for value in data.xmat[body_id]) == pytest.approx(expected_rotation, abs=1e-6)
