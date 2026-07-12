from __future__ import annotations

from pathlib import Path

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
