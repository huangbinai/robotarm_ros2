from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tempfile
from typing import Sequence
import xml.etree.ElementTree as ET

import mujoco


PACKAGE_MESH_PREFIX = "package://rebotarm_bringup/description/meshes/"
JOINTS = [f"joint{index}" for index in range(1, 7)] + [
    "left_finger_joint",
    "right_finger_joint",
]
GAINS = {
    "joint1": (480, 1),
    "joint2": (480, 1),
    "joint3": (480, 0.75),
    "joint4": (240, 0.5),
    "joint5": (200, 0.375),
    "joint6": (160, 0.25),
    "left_finger_joint": (250, 0.125),
    "right_finger_joint": (250, 0.125),
}


def authoritative_urdf_path(repo_root: Path) -> Path:
    return repo_root / "src/rebotarm_moveit_config/config/rebotarm.urdf"


def _model_directory(repo_root: Path) -> Path:
    return repo_root / "src/rebotarm_simulation/models/rebotarm"


def stage_urdf(source: Path, repo_root: Path, temporary_dir: Path) -> Path:
    root = ET.parse(source).getroot()
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        {"discardvisual": "false", "fusestatic": "false", "strippath": "false"},
    )
    source_assets = _model_directory(repo_root) / "assets"
    staged_assets = temporary_dir / "assets"
    staged_assets.mkdir(parents=True, exist_ok=True)

    for link in root.findall("link"):
        for role in ("visual", "collision"):
            for index, element in enumerate(link.findall(role)):
                element.set("name", f'{link.attrib["name"]}_{role}_{index}')

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename.startswith(PACKAGE_MESH_PREFIX):
            raise ValueError(f"unsupported URDF mesh URI: {filename}")
        basename = filename.removeprefix(PACKAGE_MESH_PREFIX)
        source_mesh = source_assets / basename
        if not source_mesh.is_file():
            raise FileNotFoundError(source_mesh)
        shutil.copy2(source_mesh, staged_assets / basename)
        mesh.set("filename", f"assets/{basename}")

    staged = temporary_dir / source.name
    ET.ElementTree(root).write(staged, encoding="utf-8", xml_declaration=True)
    return staged


def generate_mjcf_bytes(repo_root: Path) -> bytes:
    source = authoritative_urdf_path(repo_root)
    with tempfile.TemporaryDirectory(prefix="rebotarm-urdf-") as directory:
        staged = stage_urdf(source, repo_root, Path(directory))
        spec = mujoco.MjSpec.from_file(str(staged))
        spec.compile()
        root = ET.fromstring(spec.to_xml())
        urdf_root = ET.parse(source).getroot()
        _configure_compiler(root)
        _classify_geoms(root)
        _add_contact_exclusions(root)
        _add_finger_coupling(root)
        _add_sites(root)
        _add_actuators(root, urdf_root)
        _add_sensors(root)
    return _canonicalize(root)


def _configure_compiler(root: ET.Element) -> None:
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    default = ET.Element("default")
    ET.SubElement(
        default,
        "geom",
        {"solref": "0.01 1", "solimp": "0.9 0.95 0.001", "friction": "0.8 0.02 0.001"},
    )
    visual = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(visual, "geom", {"contype": "0", "conaffinity": "0", "group": "2"})
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(collision, "geom", {"contype": "1", "conaffinity": "1", "group": "3"})
    compiler_index = list(root).index(compiler)
    root.insert(compiler_index + 1, default)


def _classify_geoms(root: ET.Element) -> None:
    for body in root.findall("worldbody//body"):
        body_name = body.attrib["name"]
        mesh_counts: dict[str, int] = {}
        for geom in body.findall("geom"):
            mesh = geom.attrib["mesh"]
            index = mesh_counts.get(mesh, 0)
            mesh_counts[mesh] = index + 1
            is_visual = geom.attrib.get("contype") == "0"
            role = "visual" if is_visual else "collision"
            geom.set("name", f"{body_name}_{mesh}_{role}")
            geom.set("class", role)
            geom.set("contype", "0" if is_visual else "1")
            geom.set("conaffinity", "0" if is_visual else "1")
            geom.set("group", "2" if is_visual else "3")
            geom.attrib.pop("density", None)
            if not is_visual and mesh in {"left_finger", "right_finger"}:
                geom.set("friction", "1.2 0.02 0.001")


def _add_contact_exclusions(root: ET.Element) -> None:
    contact = ET.SubElement(root, "contact")
    for body1, body2 in (
        ("base_link", "link1"),
        ("link1", "link2"),
        ("link2", "link3"),
        ("link3", "link4"),
        ("link4", "link5"),
        ("link5", "link6"),
        ("link6", "end_link"),
        ("end_link", "left_finger_link"),
        ("end_link", "right_finger_link"),
        ("link2", "link4"),
        ("link4", "link6"),
        ("left_finger_link", "right_finger_link"),
    ):
        ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})


def _add_finger_coupling(root: ET.Element) -> None:
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "finger_coupling",
            "joint1": "right_finger_joint",
            "joint2": "left_finger_joint",
            "polycoef": "0 -1 0 0 0",
        },
    )


def _add_sites(root: ET.Element) -> None:
    end_link = root.find('.//body[@name="end_link"]')
    if end_link is None:
        raise ValueError("converted MJCF is missing end_link")
    ET.SubElement(end_link, "site", {"name": "ee_site", "pos": "-0.105 0 0", "size": "0.008"})
    ET.SubElement(
        end_link,
        "site",
        {"name": "wrist_camera_mount", "pos": "-0.04 0 0.04", "quat": "1 0 0 0", "size": "0.005"},
    )


def _add_actuators(root: ET.Element, urdf_root: ET.Element) -> None:
    limits = {
        joint.attrib["name"]: joint.find("limit")
        for joint in urdf_root.findall("joint")
        if joint.attrib.get("name") in JOINTS
    }
    actuator = ET.SubElement(root, "actuator")
    for joint_name in JOINTS:
        limit = limits[joint_name]
        if limit is None:
            raise ValueError(f"URDF joint has no limit: {joint_name}")
        kp, kv = GAINS[joint_name]
        effort = limit.attrib["effort"]
        ET.SubElement(
            actuator,
            "position",
            {
                "name": f"{joint_name.removesuffix('_joint')}_position",
                "joint": joint_name,
                "kp": str(kp),
                "kv": str(kv),
                "ctrlrange": f'{limit.attrib["lower"]} {limit.attrib["upper"]}',
                "ctrllimited": "true",
                "forcelimited": "true",
                "forcerange": f"-{effort} {effort}",
            },
        )


def _add_sensors(root: ET.Element) -> None:
    sensor = ET.SubElement(root, "sensor")
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(sensor, "jointpos", {"name": f"{prefix}_pos", "joint": joint_name})
        ET.SubElement(sensor, "jointvel", {"name": f"{prefix}_vel", "joint": joint_name})
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(
            sensor,
            "actuatorfrc",
            {"name": f"{prefix}_force", "actuator": f"{prefix}_position"},
        )
    ET.SubElement(sensor, "framepos", {"name": "ee_position", "objtype": "site", "objname": "ee_site"})
    ET.SubElement(sensor, "framequat", {"name": "ee_orientation", "objtype": "site", "objname": "ee_site"})


def _canonicalize(root: ET.Element) -> bytes:
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    warning = "<!-- AUTO-GENERATED from rebotarm.urdf; do not edit robot.xml manually. -->\n"
    return (warning + xml.replace("\r\n", "\n").rstrip() + "\n").encode("utf-8")


def check_generated_model(repo_root: Path, output: Path) -> bool:
    return output.is_file() and output.read_bytes() == generate_mjcf_bytes(repo_root)


def write_generated_model(repo_root: Path, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    try:
        temporary.write_bytes(generate_mjcf_bytes(repo_root))
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)


def _default_repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_output(repo_root: Path) -> Path:
    return _model_directory(repo_root) / "robot.xml"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate reBotArm MJCF from the authoritative URDF")
    parser.add_argument("--repo-root", type=Path, default=_default_repo_root())
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    output = (args.output or _default_output(repo_root)).resolve()

    if args.check:
        if check_generated_model(repo_root, output):
            print(f"MJCF is up to date: {output}")
            return 0
        print(f"MJCF is stale; regenerate with rebotarm_urdf_to_mjcf --repo-root \"{repo_root}\"")
        return 1

    write_generated_model(repo_root, output)
    print(f"Generated MJCF: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
