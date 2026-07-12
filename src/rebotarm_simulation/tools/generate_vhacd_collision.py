#!/usr/bin/env python3
"""Generate and verify deterministic convex collision assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import struct
import sys
import tempfile
import uuid
from pathlib import Path

EXPECTED_GENERATOR = {"package": "vhacdx", "version": "0.0.10"}
EXPECTED_COMMON = {
    "resolution": 400000,
    "minimum_volume_percent_error_allowed": 1.0,
    "max_recursion_depth": 10,
    "shrink_wrap": True,
    "fill_mode": "flood",
    "max_num_vertices_per_hull": 64,
    "async_acd": False,
    "min_edge_length": 2,
    "find_best_plane": False,
}
EXPECTED_PARTS = {
    "base_link": ("base_link.STL", 8),
    "link1": ("link1.STL", 8),
    "link2": ("link2.STL", 8),
    "link3": ("link3.STL", 8),
    "link4": ("link4.STL", 8),
    "link5": ("link5.STL", 8),
    "link6": ("link6.STL", 8),
    "gripper_base": ("gripper_base.stl", 12),
    "left_finger": ("left_finger.stl", 20),
    "right_finger": ("right_finger.stl", 20),
}


class Config(dict):
    """Configuration mapping which remembers its source without serializing it."""

    config_path: Path | None = None


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rounded(values):
    return tuple(round(float(value), 12) for value in values)


def hull_sort_key(mesh):
    geometry_hash = hashlib.sha256(canonical_stl_bytes(mesh)).hexdigest()
    return (-round(float(mesh.volume), 12), _rounded(mesh.centroid),
            _rounded(mesh.bounds.reshape(-1)), geometry_hash)


def load_config(path):
    path = Path(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid config: {exc}") from exc
    if not isinstance(value, dict) or set(value) != {"schema_version", "generator", "common", "parts"}:
        raise ValueError("config top-level keys are invalid")
    if type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("config schema_version must be 1")
    if value.get("generator") != EXPECTED_GENERATOR:
        raise ValueError("config generator must be vhacdx 0.0.10")
    common = value.get("common")
    parts = value.get("parts")
    if (not isinstance(common, dict) or common != EXPECTED_COMMON
            or any(type(common.get(key)) is not type(expected)
                   for key, expected in EXPECTED_COMMON.items())):
        raise ValueError("config common must match the fixed VHACD parameter contract")
    if not isinstance(parts, dict) or set(parts) != set(EXPECTED_PARTS):
        raise ValueError("config parts must contain exactly the ten fixed parts")
    for name, part in parts.items():
        expected_source, expected_budget = EXPECTED_PARTS[name]
        if (not isinstance(part, dict) or set(part) != {"source", "max_convex_hulls"}
                or not isinstance(part["source"], str) or not part["source"]
                or Path(part["source"]).name != part["source"]
                or Path(part["source"]).suffix.lower() != ".stl"
                or type(part["max_convex_hulls"]) is not int
                or part["max_convex_hulls"] <= 0
                or part["source"] != expected_source
                or part["max_convex_hulls"] != expected_budget):
            raise ValueError(f"invalid config part structure: {name!r}")
    result = Config(value)
    result.config_path = path.resolve()
    return result


def _finite_mesh(mesh, label):
    import numpy as np
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    if vertices.ndim != 2 or vertices.shape[1:] != (3,) or not len(vertices):
        raise ValueError(f"{label} has empty or invalid vertices")
    if faces.ndim != 2 or faces.shape[1:] != (3,) or not len(faces):
        raise ValueError(f"{label} has empty or non-triangular faces")
    if not np.isfinite(vertices).all() or not np.isfinite(faces).all():
        raise ValueError(f"{label} contains non-finite data")


def decompose_part(source, settings):
    import numpy as np
    import trimesh
    import vhacdx

    mesh = trimesh.load_mesh(source, process=False)
    if isinstance(mesh, trimesh.Scene):
        raise ValueError(f"source must be one mesh, not a Scene: {source}")
    _finite_mesh(mesh, "source mesh")
    triangles = np.asarray(mesh.faces, dtype=np.uint32)
    packed_faces = np.column_stack((np.full(len(triangles), 3, dtype=np.uint32), triangles)).reshape(-1)
    common = settings
    result = vhacdx.compute_vhacd(
        np.asarray(mesh.vertices, dtype=np.float64), packed_faces,
        maxConvexHulls=common["max_convex_hulls"], resolution=common["resolution"],
        minimumVolumePercentErrorAllowed=common["minimum_volume_percent_error_allowed"],
        maxRecursionDepth=common["max_recursion_depth"], shrinkWrap=common["shrink_wrap"],
        fillMode=common["fill_mode"], maxNumVerticesPerCH=common["max_num_vertices_per_hull"],
        asyncACD=common["async_acd"], minEdgeLength=common["min_edge_length"],
        findBestPlane=common["find_best_plane"],
    )
    hulls = []
    for index, item in enumerate(result):
        if isinstance(item, trimesh.Trimesh):
            hull = item.copy()
        else:
            hull = trimesh.Trimesh(vertices=item[0], faces=item[1], process=False)
        hull.remove_unreferenced_vertices()
        _finite_mesh(hull, f"hull {index}")
        if not hull.is_convex:
            raise ValueError(f"hull {index} is not convex")
        hulls.append(hull)
    if not hulls:
        raise ValueError("VHACD returned no hulls")
    return sorted(hulls, key=hull_sort_key)


def canonical_stl_bytes(mesh):
    import numpy as np
    vertices = np.asarray(mesh.vertices, dtype=np.float32)
    vertices[vertices == 0] = np.float32(0.0)
    faces = np.asarray(mesh.faces)
    if faces.ndim != 2 or faces.shape[1:] != (3,):
        raise ValueError("STL output requires triangular faces")
    _finite_mesh(mesh, "STL mesh")
    records = []
    for face in faces:
        triangle = vertices[face]
        rotations = [np.roll(triangle, -offset, axis=0) for offset in range(3)]
        triangle = min(rotations, key=lambda item: tuple(float(value) for value in item.reshape(-1)))
        normal = np.cross(triangle[1].astype(np.float64) - triangle[0],
                          triangle[2].astype(np.float64) - triangle[0])
        length = np.linalg.norm(normal)
        normal = np.asarray(normal / length if length else np.zeros(3), dtype=np.float32)
        normal[normal == 0] = np.float32(0.0)
        payload = struct.pack("<12fH", *(normal.tolist() + triangle.reshape(-1).tolist()), 0)
        records.append(payload)
    records.sort()
    return b"reBotArm deterministic STL".ljust(80, b"\0") + struct.pack("<I", len(records)) + b"".join(records)


def _inside(path, root, label):
    path = Path(path)
    root = Path(root).resolve()
    try:
        resolved = path.resolve()
        resolved.relative_to(root)
    except (OSError, ValueError) as exc:
        raise ValueError(f"unsafe {label} path: {path}") from exc
    return resolved


def _config_hash(config):
    clean = {key: value for key, value in config.items() if not key.startswith("_")}
    payload = json.dumps(clean, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _normalized_text_sha256(path):
    text = Path(path).read_text(encoding="utf-8")
    payload = text.replace("\r\n", "\n").replace("\r", "\n").encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _mesh_metadata(path):
    import trimesh
    mesh = trimesh.load_mesh(path, process=False)
    if isinstance(mesh, trimesh.Scene):
        raise ValueError(f"output is a Scene: {path}")
    _finite_mesh(mesh, "output mesh")
    return {
        "vertices": int(len(mesh.vertices)), "faces": int(len(mesh.faces)),
        "volume": round(float(abs(mesh.volume)), 12),
        "bounds": [[round(float(x), 12) for x in row] for row in mesh.bounds],
    }


def _atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _validate_manifest_schema(manifest):
    top_keys = {"schema_version", "generator", "generator_script_sha256", "config_sha256",
                "parameters", "parts"}
    if not isinstance(manifest, dict) or set(manifest) != top_keys:
        raise ValueError("invalid manifest top-level schema")
    if not isinstance(manifest.get("parameters"), dict):
        raise ValueError("invalid manifest parameters type")
    if (type(manifest["schema_version"]) is not int
            or not isinstance(manifest["generator"], dict)
            or not isinstance(manifest["generator_script_sha256"], str)
            or not isinstance(manifest["config_sha256"], str)
            or not isinstance(manifest["parts"], dict)):
        raise ValueError("invalid manifest top-level types")
    for name, part in manifest["parts"].items():
        if not isinstance(name, str) or not isinstance(part, dict) or set(part) != {"input", "outputs"}:
            raise ValueError("invalid manifest part schema")
        input_record = part["input"]
        outputs = part["outputs"]
        if (not isinstance(input_record, dict) or set(input_record) != {"path", "sha256"}
                or not all(isinstance(input_record[key], str) for key in input_record)
                or not isinstance(outputs, list)):
            raise ValueError("invalid manifest input or outputs schema")
        for output in outputs:
            expected = {"path", "sha256", "vertices", "faces", "volume", "bounds"}
            if not isinstance(output, dict) or set(output) != expected:
                raise ValueError("invalid manifest output schema")
            if (not isinstance(output["path"], str) or not isinstance(output["sha256"], str)
                    or type(output["vertices"]) is not int or type(output["faces"]) is not int
                    or output["vertices"] <= 0 or output["faces"] <= 0
                    or type(output["volume"]) not in {int, float}
                    or not isinstance(output["bounds"], list) or len(output["bounds"]) != 2
                    or any(not isinstance(row, list) or len(row) != 3 for row in output["bounds"])
                    or any(type(value) not in {int, float} for row in output["bounds"] for value in row)):
                raise ValueError("invalid manifest output types")


def build_manifest(model_dir, config, output_root):
    model_dir = Path(model_dir).resolve()
    output_root = _inside(output_root, model_dir, "output root")
    if output_root != model_dir / "collision_vhacd":
        raise ValueError("output root must be model_dir/collision_vhacd")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".collision_vhacd.tmp.", dir=output_root.parent)).resolve()
    backup = output_root.parent / f".collision_vhacd.backup.{uuid.uuid4().hex}"
    manifest_path = model_dir / "collision_vhacd_manifest.json"
    temporary_manifest = model_dir / f".collision_vhacd_manifest.tmp.{uuid.uuid4().hex}.json"
    backup_manifest = model_dir / f".collision_vhacd_manifest.backup.{uuid.uuid4().hex}.json"
    _inside(temporary, output_root.parent, "temporary output")
    _inside(backup, output_root.parent, "backup output")
    manifest = {
        "schema_version": 1,
        "generator": EXPECTED_GENERATOR,
        "generator_script_sha256": _normalized_text_sha256(__file__),
        "config_sha256": _config_hash(config),
        "parameters": config["common"],
        "parts": {},
    }
    old_moved = False
    old_manifest_moved = False
    try:
        for name, part in config["parts"].items():
            if Path(name).name != name:
                raise ValueError(f"unsafe part name: {name}")
            if Path(part["source"]).is_absolute() or ".." in Path(part["source"]).parts:
                raise ValueError(f"unsafe input path for {name}")
            source = _inside(model_dir / "assets" / part["source"], model_dir, "input")
            part_dir = _inside(temporary / name, temporary, "part output")
            part_dir.mkdir(parents=True)
            settings = {**config["common"], "max_convex_hulls": part["max_convex_hulls"]}
            outputs = []
            for number, hull in enumerate(decompose_part(source, settings)):
                path = part_dir / f"hull_{number:03d}.stl"
                path.write_bytes(canonical_stl_bytes(hull))
                final_path = output_root / name / path.name
                relative = final_path.relative_to(model_dir).as_posix()
                outputs.append({"path": relative, "sha256": sha256_file(path), **_mesh_metadata(path)})
            manifest["parts"][name] = {
                "input": {"path": source.relative_to(model_dir).as_posix(), "sha256": sha256_file(source)},
                "outputs": outputs,
            }
        _atomic_json(temporary_manifest, manifest)
        if output_root.exists():
            os.replace(output_root, backup)
            old_moved = True
        if manifest_path.exists():
            os.replace(manifest_path, backup_manifest)
            old_manifest_moved = True
        os.replace(temporary, output_root)
        os.replace(temporary_manifest, manifest_path)
        if old_moved:
            shutil.rmtree(backup)
        if old_manifest_moved:
            backup_manifest.unlink()
        return manifest
    except Exception:
        if old_manifest_moved and not manifest_path.exists() and backup_manifest.exists():
            os.replace(backup_manifest, manifest_path)
        if old_moved and not output_root.exists() and backup.exists():
            os.replace(backup, output_root)
        raise
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
        if backup.exists() and output_root.exists():
            shutil.rmtree(backup)
        if temporary_manifest.exists():
            temporary_manifest.unlink()
        if backup_manifest.exists() and manifest_path.exists():
            backup_manifest.unlink()


def check_outputs(model_dir, config, manifest_path):
    model_dir = Path(model_dir).resolve()
    manifest_path = _inside(manifest_path, model_dir, "manifest")
    if manifest_path != model_dir / "collision_vhacd_manifest.json":
        raise ValueError("manifest path must be model_dir/collision_vhacd_manifest.json")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid manifest: {exc}") from exc
    _validate_manifest_schema(manifest)
    if manifest.get("schema_version") != 1 or manifest.get("generator") != EXPECTED_GENERATOR:
        raise ValueError("manifest schema or generator mismatch")
    if manifest.get("generator_script_sha256") != _normalized_text_sha256(__file__):
        raise ValueError("generator script hash mismatch")
    if manifest.get("parameters") != config["common"]:
        raise ValueError("manifest parameters mismatch")
    if manifest.get("config_sha256") != _config_hash(config):
        raise ValueError("config hash mismatch")
    expected_root = (model_dir / "collision_vhacd").resolve()
    if set(manifest.get("parts", {})) != set(config["parts"]):
        raise ValueError("manifest parts mismatch")
    expected_stls = set()
    for name, part in config["parts"].items():
        record = manifest["parts"][name]
        input_path = record.get("input", {}).get("path")
        if (not isinstance(input_path, str) or Path(input_path).is_absolute()
                or ".." in Path(input_path).parts):
            raise ValueError(f"unsafe input path for {name}")
        source = _inside(model_dir / input_path, model_dir, "input")
        expected_source = _inside(model_dir / "assets" / part["source"], model_dir, "input")
        if source != expected_source or not source.is_file() or sha256_file(source) != record["input"]["sha256"]:
            raise ValueError(f"input hash or path mismatch for {name}")
        part_dir = _inside(expected_root / name, expected_root, "part")
        outputs = record.get("outputs")
        if not isinstance(outputs, list) or not outputs:
            raise ValueError(f"missing outputs for {name}")
        expected_names = [f"hull_{i:03d}.stl" for i in range(len(outputs))]
        for expected_name, output in zip(expected_names, outputs):
            raw = output.get("path")
            if not isinstance(raw, str) or Path(raw).is_absolute() or ".." in Path(raw).parts:
                raise ValueError(f"unsafe output path for {name}")
            path = _inside(model_dir / raw, part_dir, "output")
            if path.name != expected_name or path.parent != part_dir:
                raise ValueError(f"invalid output path or numbering for {name}")
            if not path.is_file() or sha256_file(path) != output.get("sha256"):
                raise ValueError(f"output hash mismatch: {path}")
            expected_stls.add(path)
            actual = _mesh_metadata(path)
            if any(output.get(key) != actual[key] for key in actual):
                raise ValueError(f"output metadata mismatch: {path}")
        actual_names = sorted(path.name for path in part_dir.glob("*.stl"))
        if actual_names != expected_names:
            raise ValueError(f"extra or missing hull STL for {name}")
    actual_stls = {path.resolve() for path in expected_root.rglob("*.stl")}
    if actual_stls != expected_stls:
        raise ValueError("extra or missing hull STL in collision_vhacd output root")
    return manifest


def main(argv=None):
    script = Path(__file__).resolve()
    simulation = script.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=simulation / "config" / "vhacd_collision.json")
    parser.add_argument("--model-dir", type=Path, default=simulation / "models" / "rebotarm")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        output = args.model_dir / "collision_vhacd"
        manifest = args.model_dir / "collision_vhacd_manifest.json"
        if args.check:
            check_outputs(args.model_dir, config, manifest)
            print("VHACD collision outputs are valid")
        else:
            build_manifest(args.model_dir, config, output)
            print(f"Wrote {manifest}")
        return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
