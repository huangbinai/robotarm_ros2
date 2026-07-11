from __future__ import annotations

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/rebotarm_simulation"


def _setup_keywords() -> dict[str, ast.expr]:
    tree = ast.parse((PACKAGE / "setup.py").read_text(encoding="utf-8"))
    setup_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "setup"
    ]
    assert len(setup_calls) == 1
    return {
        keyword.arg: keyword.value
        for keyword in setup_calls[0].keywords
        if keyword.arg is not None
    }


def test_mujoco_dependency_ranges_define_supported_compatibility() -> None:
    requirements = (PACKAGE / "requirements-mujoco.txt").read_text(encoding="utf-8")

    assert "mujoco>=3.3,<4" in requirements
    assert "numpy>=1.26" in requirements


def test_normal_package_install_includes_mujoco_runtime_dependencies() -> None:
    install_requires = ast.literal_eval(_setup_keywords()["install_requires"])

    assert "mujoco>=3.3,<4" in install_requires
    assert "numpy>=1.26" in install_requires


def test_setup_installs_mujoco_resources_and_entrypoints() -> None:
    keywords = _setup_keywords()
    resource_patterns = {
        ast.literal_eval(call.args[0])
        for call in ast.walk(keywords["data_files"])
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "install_resources"
        and len(call.args) == 1
    }
    assert resource_patterns == {
        "models/**/*.xml",
        "models/**/*.[sS][tT][lL]",
        "config/*.yaml",
        "launch/*.launch.py",
    }

    entry_points = ast.literal_eval(keywords["entry_points"])
    console_scripts = entry_points["console_scripts"]

    for required in (
        "rebotarm_mujoco_health",
        "rebotarm_mujoco_cli",
        "rebotarm_mujoco_viewer",
        "rebotarm_mujoco_node",
    ):
        assert any(script.startswith(f"{required} = ") for script in console_scripts)


def test_package_declares_ros_adapter_dependencies() -> None:
    root = ET.parse(PACKAGE / "package.xml").getroot()
    exec_dependencies = {
        element.text for element in root.findall("exec_depend") if element.text
    }

    for dependency in (
        "ament_index_python",
        "control_msgs",
        "launch",
        "launch_ros",
        "rclpy",
        "rebotarm_msgs",
        "sensor_msgs",
        "std_srvs",
        "trajectory_msgs",
    ):
        assert dependency in exec_dependencies
