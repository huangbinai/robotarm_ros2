import json
import sys
from pathlib import Path

import numpy as np
import pytest
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "rebotarm_simulation" / "tools"))
import generate_vhacd_collision as generator


ROOT = Path(__file__).resolve().parents[1]
SIMULATION = ROOT / "src" / "rebotarm_simulation"
CONFIG_PATH = SIMULATION / "config" / "vhacd_collision.json"
REQUIREMENTS_PATH = SIMULATION / "requirements-vhacd.txt"
ASSETS = SIMULATION / "models" / "rebotarm" / "assets"


def _config():
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def test_vhacd_config_locks_generator_and_part_contract():
    config = _config()

    assert config["schema_version"] == 1
    assert config["generator"] == {"package": "vhacdx", "version": "0.0.10"}
    assert config["parts"] == {
        "base_link": {"source": "base_link.STL", "max_convex_hulls": 8},
        "link1": {"source": "link1.STL", "max_convex_hulls": 8},
        "link2": {"source": "link2.STL", "max_convex_hulls": 8},
        "link3": {"source": "link3.STL", "max_convex_hulls": 8},
        "link4": {"source": "link4.STL", "max_convex_hulls": 8},
        "link5": {"source": "link5.STL", "max_convex_hulls": 8},
        "link6": {"source": "link6.STL", "max_convex_hulls": 8},
        "gripper_base": {"source": "gripper_base.stl", "max_convex_hulls": 12},
        "left_finger": {"source": "left_finger.stl", "max_convex_hulls": 20},
        "right_finger": {"source": "right_finger.stl", "max_convex_hulls": 20},
    }

    for name, part in config["parts"].items():
        assert part["source"].lower().endswith(".stl")
        assert (ASSETS / part["source"]).is_file(), name
        upper_bound = 20 if name.endswith("finger") else 16
        assert 4 <= part["max_convex_hulls"] <= upper_bound


def test_vhacd_config_locks_common_parameters():
    assert _config()["common"] == {
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


def test_vhacd_requirements_are_exactly_pinned():
    assert REQUIREMENTS_PATH.read_text(encoding="utf-8").splitlines() == [
        "numpy==2.4.2",
        "trimesh==4.12.2",
        "vhacdx==0.0.10",
    ]


def test_sha256_file_streams(tmp_path):
    path = tmp_path / "payload"
    path.write_bytes(b"abc" * 500_000)
    assert generator.sha256_file(path) == "206c8387a6fd09a951f4945cb98bd65ae2b9a59d4069f70e00a9d9acd7c86515"


def test_hull_sort_key_orders_volume_then_geometry():
    small = trimesh.creation.box(extents=(1, 1, 1))
    shifted = small.copy()
    shifted.apply_translation((2, 0, 0))
    large = trimesh.creation.box(extents=(2, 1, 1))
    assert sorted([shifted, small, large], key=generator.hull_sort_key) == [large, small, shifted]


def test_load_config_validates_contract(tmp_path):
    valid = _config()
    path = tmp_path / "config.json"
    path.write_text(json.dumps(valid), encoding="utf-8")
    assert generator.load_config(path) == valid
    valid["generator"]["version"] = "latest"
    path.write_text(json.dumps(valid), encoding="utf-8")
    with pytest.raises(ValueError, match="vhacdx 0.0.10"):
        generator.load_config(path)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda value: value.update({"extra": True}),
        lambda value: value["common"].update({"async_acd": True}),
        lambda value: value["common"].update({"resolution": True}),
        lambda value: value["common"].update({"resolution": 0}),
        lambda value: value["parts"]["link1"].update({"source": "../link1.STL"}),
        lambda value: value["parts"]["link1"].update({"max_convex_hulls": True}),
        lambda value: value["parts"].pop("link1"),
    ],
)
def test_load_config_rejects_contract_drift(tmp_path, mutate):
    value = _config()
    mutate(value)
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError, match="config"):
        generator.load_config(path)


def test_canonical_stl_bytes_are_deterministic_and_reject_non_triangles():
    mesh = trimesh.creation.box()
    assert generator.canonical_stl_bytes(mesh) == generator.canonical_stl_bytes(mesh.copy())
    bad = type("Mesh", (), {"vertices": mesh.vertices, "faces": np.array([[0, 1, 2, 3]])})()
    with pytest.raises(ValueError, match="triangular"):
        generator.canonical_stl_bytes(bad)


def test_canonical_stl_normalizes_cyclic_faces_order_and_signed_zero():
    mesh = trimesh.creation.box()
    changed = mesh.copy()
    changed.vertices[changed.vertices == 0] = -0.0
    changed.faces = np.roll(changed.faces[::-1], 1, axis=1)
    assert generator.canonical_stl_bytes(changed) == generator.canonical_stl_bytes(mesh)


def test_hull_sort_key_has_geometry_hash_tie_breaker():
    class Mesh:
        volume = 1.0
        centroid = np.zeros(3)
        bounds = np.array([[-1, -1, -1], [1, 1, 1]])

        def __init__(self, vertices):
            self.vertices = np.asarray(vertices, dtype=float)
            self.faces = np.array([[0, 1, 2]])

    first = Mesh([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    second = Mesh([[0, 0, 0], [1, 0, 0], [0, 0, 1]])
    assert generator.hull_sort_key(first) != generator.hull_sort_key(second)


def test_decompose_part_flattens_faces_and_passes_locked_parameters(tmp_path, monkeypatch):
    source = tmp_path / "source.stl"
    trimesh.creation.box().export(source)
    calls = {}

    def fake_compute(vertices, faces, **kwargs):
        calls.update(vertices=vertices, faces=faces, kwargs=kwargs)
        hull = trimesh.creation.box()
        return [(hull.vertices, hull.faces)]

    monkeypatch.setitem(sys.modules, "vhacdx", type("V", (), {"compute_vhacd": staticmethod(fake_compute)}))
    settings = {**_config()["common"], "max_convex_hulls": 8}
    hulls = generator.decompose_part(source, settings)
    assert len(hulls) == 1 and hulls[0].is_convex
    assert calls["faces"].dtype == np.uint32
    assert calls["faces"].ndim == 1
    assert np.all(calls["faces"].reshape((-1, 4))[:, 0] == 3)
    assert calls["kwargs"] == {
        "maxConvexHulls": 8, "resolution": 400000,
        "minimumVolumePercentErrorAllowed": 1.0, "maxRecursionDepth": 10,
        "shrinkWrap": True, "fillMode": "flood", "maxNumVerticesPerCH": 64,
        "asyncACD": False, "minEdgeLength": 2, "findBestPlane": False,
    }


def _small_config(tmp_path):
    source = tmp_path / "assets" / "part.stl"
    source.parent.mkdir()
    trimesh.creation.box().export(source)
    config = _config()
    config["parts"] = {"part": {"source": "part.stl", "max_convex_hulls": 2}}
    config_path = tmp_path / "config.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")
    config["_config_path"] = config_path
    return config


def test_build_and_check_manifest_detect_tampering_extra_and_traversal(tmp_path, monkeypatch):
    config = _small_config(tmp_path)
    monkeypatch.setattr(generator, "decompose_part", lambda *_: [trimesh.creation.box()])
    manifest = generator.build_manifest(tmp_path, config, tmp_path / "collision_vhacd")
    manifest_path = tmp_path / "collision_vhacd_manifest.json"
    assert manifest_path.is_file()
    assert manifest["parameters"] == config["common"]
    assert manifest["generator_script_sha256"] == generator._normalized_text_sha256(generator.__file__)
    assert generator.check_outputs(tmp_path, config, manifest_path) == manifest

    hull = tmp_path / "collision_vhacd" / "part" / "hull_000.stl"
    original = hull.read_bytes()
    hull.write_bytes(original + b"tamper")
    with pytest.raises(ValueError, match="hash"):
        generator.check_outputs(tmp_path, config, manifest_path)
    hull.write_bytes(original)
    (hull.parent / "hull_999.stl").write_bytes(original)
    with pytest.raises(ValueError, match="extra"):
        generator.check_outputs(tmp_path, config, manifest_path)
    (hull.parent / "hull_999.stl").unlink()
    data = json.loads(manifest_path.read_text())
    data["parts"]["part"]["outputs"][0]["path"] = "../escape.stl"
    manifest_path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="path"):
        generator.check_outputs(tmp_path, config, manifest_path)


def test_check_rejects_stl_in_unknown_directory(tmp_path, monkeypatch):
    config = _small_config(tmp_path)
    monkeypatch.setattr(generator, "decompose_part", lambda *_: [trimesh.creation.box()])
    generator.build_manifest(tmp_path, config, tmp_path / "collision_vhacd")
    root = tmp_path / "collision_vhacd"
    unknown = root / "unknown" / "nested" / "extra.stl"
    unknown.parent.mkdir(parents=True)
    unknown.write_bytes((root / "part" / "hull_000.stl").read_bytes())
    with pytest.raises(ValueError, match="extra"):
        generator.check_outputs(tmp_path, config, tmp_path / "collision_vhacd_manifest.json")


@pytest.mark.parametrize("field", ["parameters", "generator_script_sha256"])
def test_check_rejects_manifest_provenance_tampering(tmp_path, monkeypatch, field):
    config = _small_config(tmp_path)
    monkeypatch.setattr(generator, "decompose_part", lambda *_: [trimesh.creation.box()])
    generator.build_manifest(tmp_path, config, tmp_path / "collision_vhacd")
    path = tmp_path / "collision_vhacd_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest[field] = "tampered"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="parameter|script"):
        generator.check_outputs(tmp_path, config, path)


def test_check_is_read_only_and_does_not_import_vhacdx(tmp_path, monkeypatch):
    config = _small_config(tmp_path)
    monkeypatch.setattr(generator, "decompose_part", lambda *_: [trimesh.creation.box()])
    generator.build_manifest(tmp_path, config, tmp_path / "collision_vhacd")
    manifest_path = tmp_path / "collision_vhacd_manifest.json"
    before = {p: (p.stat().st_mtime_ns, p.read_bytes()) for p in tmp_path.rglob("*") if p.is_file()}
    sys.modules.pop("vhacdx", None)
    generator.check_outputs(tmp_path, config, manifest_path)
    after = {p: (p.stat().st_mtime_ns, p.read_bytes()) for p in tmp_path.rglob("*") if p.is_file()}
    assert before == after and "vhacdx" not in sys.modules


def test_build_failure_preserves_existing_tree_without_transaction_debris(tmp_path, monkeypatch):
    config = _small_config(tmp_path)
    second = tmp_path / "assets" / "second.stl"
    trimesh.creation.box().export(second)
    config["parts"]["second"] = {"source": "second.stl", "max_convex_hulls": 2}
    config["_config_path"].write_text(json.dumps({k: v for k, v in config.items() if not k.startswith("_")}))
    monkeypatch.setattr(generator, "decompose_part", lambda *_: [trimesh.creation.box()])
    generator.build_manifest(tmp_path, config, tmp_path / "collision_vhacd")
    before = {p.relative_to(tmp_path): p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    calls = iter(([trimesh.creation.box()], RuntimeError("injected second-part failure")))

    def fail_second(*_):
        result = next(calls)
        if isinstance(result, Exception):
            raise result
        return result

    monkeypatch.setattr(generator, "decompose_part", fail_second)
    with pytest.raises(RuntimeError, match="injected"):
        generator.build_manifest(tmp_path, config, tmp_path / "collision_vhacd")
    after = {p.relative_to(tmp_path): p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    assert after == before
    assert not list(tmp_path.glob(".collision_vhacd.*"))


def test_provenance_hashes_normalize_json_and_newlines(tmp_path):
    value = _config()
    compact = json.dumps(value, sort_keys=True, separators=(",", ":"))
    pretty = json.dumps(value, indent=2).replace("\n", "\r\n")
    lf_config = tmp_path / "lf.json"
    crlf_config = tmp_path / "crlf.json"
    lf_config.write_text(compact, encoding="utf-8", newline="\n")
    crlf_config.write_text(pretty, encoding="utf-8", newline="")
    assert generator._config_hash(generator.load_config(lf_config)) == generator._config_hash(
        generator.load_config(crlf_config)
    )
    lf_script = tmp_path / "lf.py"
    crlf_script = tmp_path / "crlf.py"
    lf_script.write_bytes(b"one\ntwo\n")
    crlf_script.write_bytes(b"one\r\ntwo\r\n")
    assert generator._normalized_text_sha256(lf_script) == generator._normalized_text_sha256(crlf_script)


@pytest.mark.parametrize("mutation", ["top", "part", "output", "nested"])
def test_check_rejects_malformed_manifest_as_value_error(tmp_path, monkeypatch, mutation):
    config = _small_config(tmp_path)
    monkeypatch.setattr(generator, "decompose_part", lambda *_: [trimesh.creation.box()])
    generator.build_manifest(tmp_path, config, tmp_path / "collision_vhacd")
    path = tmp_path / "collision_vhacd_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "top":
        manifest["extra"] = 1
    elif mutation == "part":
        manifest["parts"]["part"]["extra"] = 1
    elif mutation == "output":
        manifest["parts"]["part"]["outputs"][0]["extra"] = 1
    else:
        manifest["parts"]["part"]["input"] = []
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="manifest"):
        generator.check_outputs(tmp_path, config, path)
