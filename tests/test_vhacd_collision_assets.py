import json
from pathlib import Path


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
