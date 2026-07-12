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
REPAIR_PROFILE = {
    "method": "voxel_marching_cubes", "pitch_m": 0.00025,
    "fallback_pitch_m": 0.00020, "closing_iterations": 1,
    "fill_holes": True, "seed": 20260712, "samples": 6000,
    "p95_m": 0.00035, "max_m": 0.00075, "bounds_m": 0.0005,
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
        expected_keys = {"source", "max_convex_hulls", "repair"} if name.endswith("finger") else {"source", "max_convex_hulls"}
        if (not isinstance(part, dict) or set(part) != expected_keys
                or not isinstance(part["source"], str) or not part["source"]
                or Path(part["source"]).name != part["source"]
                or Path(part["source"]).suffix.lower() != ".stl"
                or type(part["max_convex_hulls"]) is not int
                or part["max_convex_hulls"] <= 0
                or part["source"] != expected_source
                or part["max_convex_hulls"] != expected_budget
                or (name.endswith("finger") and part.get("repair") != REPAIR_PROFILE)):
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


def _component_sort_key(mesh):
    """Stable ordering independent of the source face/component order."""
    return (_rounded(mesh.bounds.reshape(-1)), _rounded(mesh.centroid),
            hashlib.sha256(canonical_stl_bytes(mesh)).hexdigest())


def repair_mesh(mesh, profile):
    """Repair disconnected/open triangle surfaces on one global voxel lattice."""
    import numpy as np
    import trimesh
    from scipy import ndimage
    from skimage import measure

    _finite_mesh(mesh, "repair source")
    pitch = float(profile["pitch_m"])
    if not np.isfinite(pitch) or pitch <= 0:
        raise ValueError("repair pitch must be positive")
    source = mesh.copy()
    source.process(validate=True)
    components = sorted(source.split(only_watertight=False), key=_component_sort_key)
    global_min = np.min([part.bounds[0] for part in components], axis=0)
    global_max = np.max([part.bounds[1] for part in components], axis=0)
    lattice_origin = np.floor(global_min / pitch) * pitch
    padding = 2
    origin = lattice_origin - padding * pitch
    shape = np.ceil((global_max - lattice_origin) / pitch).astype(int) + 1 + 2 * padding
    occupancy = np.zeros(tuple(shape), dtype=bool)
    for component in components:
        points = np.asarray(component.voxelized(pitch).points, dtype=float)
        indices_float = (points - origin) / pitch
        indices = np.rint(indices_float).astype(np.int64)
        if not np.allclose(indices_float, indices, atol=1e-6, rtol=0):
            raise ValueError("component voxels do not align with global lattice")
        if np.any(indices < 0) or np.any(indices >= shape):
            raise ValueError("component voxel lies outside global lattice")
        occupancy[tuple(indices.T)] = True
    occupancy = ndimage.binary_closing(
        occupancy, iterations=int(profile["closing_iterations"])
    )
    if profile["fill_holes"]:
        occupancy = ndimage.binary_fill_holes(occupancy)
    if not occupancy.any():
        raise ValueError("repair produced empty occupancy")
    vertices, faces, _, _ = measure.marching_cubes(
        occupancy.astype(np.uint8), level=0.5, spacing=(pitch, pitch, pitch)
    )
    vertices += origin
    result = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    result.process(validate=True)
    result.remove_unreferenced_vertices()
    result.fix_normals(multibody=True)
    _finite_mesh(result, "repaired mesh")
    if (not result.is_watertight or not result.is_winding_consistent
            or not result.nondegenerate_faces().all()):
        raise ValueError("repair result failed topology validation")
    return result


def _sample_surface(mesh, count, seed):
    import trimesh
    points, _ = trimesh.sample.sample_surface(mesh, count, seed=seed)
    return points


def _closest_distances(mesh, points, chunk=32):
    """Exact accelerated triangle distances in bounded-memory chunks."""
    import numpy as np
    import trimesh
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,) or not np.isfinite(points).all():
        raise ValueError("closest-point query contains invalid points")
    values = []
    try:
        for start in range(0, len(points), chunk):
            _, distance, _ = trimesh.proximity.closest_point(mesh, points[start:start + chunk])
            if not np.isfinite(distance).all():
                raise ValueError("closest-point query returned non-finite distances")
            values.append(distance)
    except ValueError:
        raise
    except Exception as exc:
        raise ValueError(f"closest-point query failed: {exc}") from exc
    return np.concatenate(values) if values else np.empty(0)


def _contains_points_exact(mesh, points, chunk=32):
    import numpy as np
    points = np.asarray(points, dtype=float)
    if not mesh.is_watertight:
        raise ValueError("containment mesh must be watertight")
    if points.ndim != 2 or points.shape[1:] != (3,) or not np.isfinite(points).all():
        raise ValueError("containment query contains invalid points")
    values = []
    try:
        for start in range(0, len(points), chunk):
            values.append(np.asarray(mesh.contains(points[start:start + chunk]), dtype=bool))
    except Exception as exc:
        raise ValueError(f"containment query failed: {exc}") from exc
    return np.concatenate(values) if values else np.empty(0, dtype=bool)


def fidelity_metrics(original, repaired, profile):
    """Deterministic bidirectional collision-source fidelity measurements."""
    import numpy as np
    count = max(6000, int(profile["samples"]))
    seed = int(profile["seed"])
    original_points = _sample_surface(original, count, seed)
    inside = _contains_points_exact(repaired, original_points)
    a = np.zeros(count, dtype=float)
    if (~inside).any():
        a[~inside] = _closest_distances(repaired, original_points[~inside])
    repaired_points = _sample_surface(repaired, count, seed + 1)
    b = _closest_distances(original, repaired_points)

    def summarize(values):
        return {"samples": count, "p95_m": float(np.percentile(values, 95)),
                "max_m": float(np.max(values))}
    return {
        "original_to_repaired": summarize(a),
        "repaired_to_original": summarize(b),
        "bounds_max_abs_m": float(np.max(np.abs(original.bounds - repaired.bounds))),
    }


def _topology(mesh):
    return {"vertices": int(len(mesh.vertices)), "faces": int(len(mesh.faces)),
            "watertight": bool(mesh.is_watertight),
            "winding_consistent": bool(mesh.is_winding_consistent)}


def _normalized_mesh_sha256(mesh):
    return hashlib.sha256(canonical_stl_bytes(mesh)).hexdigest()


def _fidelity_passes(metrics, profile):
    return (metrics["original_to_repaired"]["p95_m"] <= profile["p95_m"]
            and metrics["original_to_repaired"]["max_m"] <= profile["max_m"]
            and metrics["repaired_to_original"]["p95_m"] <= profile["p95_m"]
            and metrics["repaired_to_original"]["max_m"] <= profile["max_m"]
            and metrics["bounds_max_abs_m"] <= profile["bounds_m"])


def _build_repairs(model_dir, config, temporary_root, final_root):
    import trimesh
    repair_parts = [(name, part) for name, part in config["parts"].items() if "repair" in part]
    if not repair_parts:
        return {}
    attempts = [repair_parts[0][1]["repair"]["pitch_m"],
                repair_parts[0][1]["repair"]["fallback_pitch_m"]]
    failures = []
    for pitch in attempts:
        records = {}
        try:
            for name, part in repair_parts:
                profile = {**part["repair"], "pitch_m": pitch}
                source = _inside(model_dir / "assets" / part["source"], model_dir, "repair input")
                original = trimesh.load_mesh(source, process=False)
                if isinstance(original, trimesh.Scene):
                    raise ValueError(f"repair source is a Scene: {source}")
                repaired = repair_mesh(original, profile)
                path = temporary_root / f"{name}.stl"
                path.write_bytes(canonical_stl_bytes(repaired))
                repaired = trimesh.load_mesh(path, process=True)
                metrics = fidelity_metrics(original, repaired, profile)
                if not _fidelity_passes(metrics, profile):
                    raise ValueError(f"{name} fidelity failed at pitch {pitch}: {metrics}")
                final_path = final_root / path.name
                records[name] = {
                    "mesh": repaired, "path": path,
                    "record": {
                        "original": {"path": source.relative_to(model_dir).as_posix(),
                                     "raw_sha256": sha256_file(source),
                                     "normalized_sha256": _normalized_mesh_sha256(original)},
                        "repaired": {"path": final_path.relative_to(model_dir).as_posix(),
                                     "raw_sha256": sha256_file(path),
                                     "normalized_sha256": _normalized_mesh_sha256(repaired)},
                        "selected_profile": profile,
                        "topology": {"before": _topology(original), "after": _topology(repaired)},
                        "fidelity": metrics,
                    },
                }
            return records
        except Exception as exc:
            failures.append(str(exc))
            for path in temporary_root.glob("*.stl"):
                path.unlink()
    raise ValueError("finger repair failed at both pitches: " + " | ".join(failures))


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
        if (not isinstance(name, str) or not isinstance(part, dict)
                or set(part) not in ({"input", "outputs"}, {"input", "repair", "outputs"})):
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
        if "repair" in part:
            repair = part["repair"]
            if not isinstance(repair, dict) or set(repair) != {
                    "original", "repaired", "selected_profile", "topology", "fidelity"}:
                raise ValueError("invalid manifest repair schema")
            for key in ("original", "repaired"):
                if (not isinstance(repair[key], dict)
                        or set(repair[key]) != {"path", "raw_sha256", "normalized_sha256"}
                        or not all(isinstance(value, str) for value in repair[key].values())):
                    raise ValueError("invalid manifest repair source schema")
            if (not isinstance(repair["selected_profile"], dict)
                    or not isinstance(repair["topology"], dict)
                    or not isinstance(repair["fidelity"], dict)):
                raise ValueError("invalid manifest repair record types")


def _cleanup_artifact(path):
    """Best-effort cleanup after commit; cleanup errors never roll back outputs."""
    try:
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()
    except OSError as exc:
        print(f"warning: could not clean transaction artifact {path}: {exc}", file=sys.stderr)


def _cleanup_stale_backups(model_dir, final_path, pattern):
    """Remove only generator-named stale backups when the final artifact exists."""
    if not final_path.exists():
        return
    for path in model_dir.glob(pattern):
        if path.parent == model_dir and ".backup." in path.name:
            _cleanup_artifact(path)


def build_manifest(model_dir, config, output_root):
    model_dir = Path(model_dir).resolve()
    output_root = _inside(output_root, model_dir, "output root")
    if output_root != model_dir / "collision_vhacd":
        raise ValueError("output root must be model_dir/collision_vhacd")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    repaired_root = model_dir / "collision_sources_repaired"
    manifest_path = model_dir / "collision_vhacd_manifest.json"
    _cleanup_stale_backups(model_dir, output_root, ".collision_vhacd.backup.*")
    _cleanup_stale_backups(model_dir, repaired_root, ".collision_sources_repaired.backup.*")
    _cleanup_stale_backups(model_dir, manifest_path, ".collision_vhacd_manifest.backup.*.json")
    temporary = Path(tempfile.mkdtemp(prefix=".collision_vhacd.tmp.", dir=output_root.parent)).resolve()
    temporary_repaired = Path(tempfile.mkdtemp(prefix=".collision_sources_repaired.tmp.", dir=model_dir)).resolve()
    backup = output_root.parent / f".collision_vhacd.backup.{uuid.uuid4().hex}"
    backup_repaired = model_dir / f".collision_sources_repaired.backup.{uuid.uuid4().hex}"
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
    old_repaired_moved = False
    old_manifest_moved = False
    output_installed = False
    repaired_installed = False
    manifest_installed = False
    try:
        repairs = _build_repairs(model_dir, config, temporary_repaired, repaired_root)
        for name, part in config["parts"].items():
            if Path(name).name != name:
                raise ValueError(f"unsafe part name: {name}")
            if Path(part["source"]).is_absolute() or ".." in Path(part["source"]).parts:
                raise ValueError(f"unsafe input path for {name}")
            source = _inside(model_dir / "assets" / part["source"], model_dir, "input")
            part_dir = _inside(temporary / name, temporary, "part output")
            part_dir.mkdir(parents=True)
            decomposition_source = repairs[name]["path"] if name in repairs else source
            settings = {**config["common"], "max_convex_hulls": part["max_convex_hulls"]}
            outputs = []
            for number, hull in enumerate(decompose_part(decomposition_source, settings)):
                path = part_dir / f"hull_{number:03d}.stl"
                path.write_bytes(canonical_stl_bytes(hull))
                final_path = output_root / name / path.name
                relative = final_path.relative_to(model_dir).as_posix()
                outputs.append({"path": relative, "sha256": sha256_file(path), **_mesh_metadata(path)})
            manifest["parts"][name] = {
                "input": {"path": source.relative_to(model_dir).as_posix(), "sha256": sha256_file(source)},
                "outputs": outputs,
            }
            if name in repairs:
                manifest["parts"][name]["repair"] = repairs[name]["record"]
        _atomic_json(temporary_manifest, manifest)
        if output_root.exists():
            os.replace(output_root, backup)
            old_moved = True
        if manifest_path.exists():
            os.replace(manifest_path, backup_manifest)
            old_manifest_moved = True
        if repairs and repaired_root.exists():
            os.replace(repaired_root, backup_repaired)
            old_repaired_moved = True
        os.replace(temporary, output_root)
        output_installed = True
        if repairs:
            os.replace(temporary_repaired, repaired_root)
            repaired_installed = True
        os.replace(temporary_manifest, manifest_path)
        manifest_installed = True
    except Exception:
        if manifest_installed and manifest_path.exists():
            manifest_path.unlink()
        if repaired_installed and repaired_root.exists():
            shutil.rmtree(repaired_root)
        if output_installed and output_root.exists():
            shutil.rmtree(output_root)
        if old_manifest_moved and backup_manifest.exists():
            os.replace(backup_manifest, manifest_path)
        if old_moved and backup.exists():
            os.replace(backup, output_root)
        if old_repaired_moved and backup_repaired.exists():
            os.replace(backup_repaired, repaired_root)
        raise
    finally:
        _cleanup_artifact(temporary)
        _cleanup_artifact(temporary_repaired)
        _cleanup_artifact(temporary_manifest)
    if old_moved:
        _cleanup_artifact(backup)
    if old_manifest_moved:
        _cleanup_artifact(backup_manifest)
    if old_repaired_moved:
        _cleanup_artifact(backup_repaired)
    return manifest


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
    expected_repaired = set()
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
        if "repair" in part:
            repair = record.get("repair")
            if not isinstance(repair, dict):
                raise ValueError(f"missing repair record for {name}")
            repaired_path = repair["repaired"]["path"]
            if (repair["original"]["path"] != input_path
                    or repair["original"]["raw_sha256"] != sha256_file(source)):
                raise ValueError(f"repair original hash or path mismatch for {name}")
            if (not isinstance(repaired_path, str) or Path(repaired_path).is_absolute()
                    or ".." in Path(repaired_path).parts):
                raise ValueError(f"unsafe repaired path for {name}")
            repaired = _inside(model_dir / repaired_path,
                               model_dir / "collision_sources_repaired", "repaired source")
            if (repaired.name != f"{name}.stl" or not repaired.is_file()
                    or sha256_file(repaired) != repair["repaired"]["raw_sha256"]):
                raise ValueError(f"repaired hash or path mismatch for {name}")
            expected_repaired.add(repaired)
            import trimesh
            original_mesh = trimesh.load_mesh(source, process=False)
            repaired_mesh = trimesh.load_mesh(repaired, process=True)
            if (_normalized_mesh_sha256(original_mesh) != repair["original"]["normalized_sha256"]
                    or _normalized_mesh_sha256(repaired_mesh) != repair["repaired"]["normalized_sha256"]
                    or repair["topology"] != {"before": _topology(original_mesh),
                                               "after": _topology(repaired_mesh)}):
                raise ValueError(f"repair normalized hash or topology mismatch for {name}")
            profile = repair["selected_profile"]
            if profile not in ({**part["repair"], "pitch_m": part["repair"]["pitch_m"]},
                               {**part["repair"], "pitch_m": part["repair"]["fallback_pitch_m"]}):
                raise ValueError(f"repair profile mismatch for {name}")
            actual_fidelity = fidelity_metrics(original_mesh, repaired_mesh, profile)
            if actual_fidelity != repair["fidelity"] or not _fidelity_passes(actual_fidelity, profile):
                raise ValueError(f"repair fidelity mismatch for {name}")
        elif "repair" in record:
            raise ValueError(f"unexpected repair record for {name}")
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
    repaired_root = model_dir / "collision_sources_repaired"
    actual_repaired = ({path.resolve() for path in repaired_root.rglob("*.stl")}
                       if repaired_root.exists() else set())
    if actual_repaired != expected_repaired:
        raise ValueError("extra or missing STL in collision_sources_repaired")
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
