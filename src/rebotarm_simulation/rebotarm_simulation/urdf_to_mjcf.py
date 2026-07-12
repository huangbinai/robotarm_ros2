from __future__ import annotations

from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import mujoco


PACKAGE_MESH_PREFIX = "package://rebotarm_bringup/description/meshes/"


def authoritative_urdf_path(repo_root: Path) -> Path:
    return repo_root / "src/rebotarm_moveit_config/config/rebotarm.urdf"


def _model_directory(repo_root: Path) -> Path:
    return repo_root / "src/rebotarm_simulation/models/rebotarm"


def stage_urdf(source: Path, repo_root: Path, temporary_dir: Path) -> Path:
    root = ET.parse(source).getroot()
    source_assets = _model_directory(repo_root) / "assets"
    staged_assets = temporary_dir / "assets"
    staged_assets.mkdir(parents=True, exist_ok=True)

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
        xml = spec.to_xml()
    return (xml.replace("\r\n", "\n").rstrip() + "\n").encode("utf-8")
