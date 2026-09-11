"""Architecture guard regression tests; standard library only, no ROS/hardware."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("architecture_guard", ROOT / "tools/check_architecture.py")
GUARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GUARD)


class ArchitectureGuardTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.rules = {
            "packages": {
                "rebotarm_msgs": {"imports": [], "resources": []},
                "rebotarm_motion": {"imports": ["rebotarm_msgs"], "resources": []},
            },
            "hardware_sdk_modules": ["motorbridge", "reBotArm_control_py"],
            "hardware_owner": "rebotarmcontroller",
            "compatibility_package": "rebotarm_interactive_control",
            "undeclared_resource_debt": [],
        }
        self.package("rebotarm_msgs")
        self.package("rebotarm_motion", deps=("rebotarm_msgs",))

    def package(self, name, deps=(), tag="exec_depend"):
        folder = self.root / "src" / name
        folder.mkdir(parents=True, exist_ok=True)
        declarations = "".join(f"<{tag}>{dep}</{tag}>" for dep in deps)
        (folder / "package.xml").write_text(f"<package><name>{name}</name>{declarations}</package>", encoding="utf-8")

    def source(self, name, text, filename="node.py"):
        path = self.root / "src" / name / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def check(self):
        return GUARD.check_repository(self.root, self.rules)

    def test_valid_graph_and_bom_source(self):
        self.source("rebotarm_motion", "\ufefffrom rebotarm_msgs.msg import ArmStatus\n")
        report = self.check()
        self.assertTrue(report["ok"], report)
        self.assertTrue(report["deployment_ready"])
        self.assertEqual(report["imports"]["rebotarm_motion"], ["rebotarm_msgs"])

    def test_reverse_import_and_cycle_are_rejected(self):
        self.source("rebotarm_motion", "import rebotarm_msgs")
        self.source("rebotarm_msgs", "import rebotarm_motion")
        errors = "\n".join(self.check()["errors"])
        self.assertIn("forbidden import", errors)
        self.assertIn("Python import cycle", errors)

    def test_import_needs_runtime_dependency_not_build_only(self):
        self.package("rebotarm_motion", deps=("rebotarm_msgs",), tag="build_depend")
        self.source("rebotarm_motion", "from rebotarm_msgs import msg")
        self.assertIn("needs depend/exec_depend", "\n".join(self.check()["errors"]))

    def test_hardware_sdk_static_and_dynamic_imports_are_rejected(self):
        snippets = (
            "import motorbridge",
            "from reBotArm_control_py.actuator import Arm",
            "import importlib as loader\nloader.import_module('motorbridge')",
            "from importlib import import_module as load\nload('motorbridge')",
            "__import__('motorbridge')",
        )
        for snippet in snippets:
            with self.subTest(snippet=snippet):
                self.source("rebotarm_motion", snippet)
                self.assertIn("hardware SDK", "\n".join(self.check()["errors"]))

    def test_unknown_package_and_misspelled_import_are_rejected(self):
        self.package("rebotarm_new")
        self.source("rebotarm_motion", "import rebotarm_mssgs")
        errors = "\n".join(self.check()["errors"])
        self.assertIn("inventory differs", errors)
        self.assertIn("forbidden import", errors)

    def test_resource_lookups_require_declared_dependencies(self):
        self.package("rebotarm_motion")
        for snippet in (
            "FindPackageShare('rebotarm_msgs')",
            "Node(package='rebotarm_msgs')",
            "_package_resource('rebotarm_msgs', 'config.yaml')",
        ):
            with self.subTest(snippet=snippet):
                self.source("rebotarm_motion", snippet)
                self.assertIn("resource package", "\n".join(self.check()["errors"]))

    def test_mesh_uri_is_a_resource_dependency(self):
        self.package("rebotarm_motion")
        path = self.root / "src/rebotarm_motion/model.urdf"
        path.write_text('<mesh filename="package://rebotarm_msgs/mesh.stl"/>', encoding="utf-8")
        self.assertIn("resource package", "\n".join(self.check()["errors"]))

    def test_known_debt_does_not_mean_deployment_ready_and_cannot_hide_imports(self):
        self.package("rebotarm_motion")
        self.rules["undeclared_resource_debt"] = [{
            "source": "rebotarm_motion", "target": "rebotarm_msgs",
            "issue": "ARCH-test", "exit_gate": "S1", "reason": "fixture",
        }]
        self.source("rebotarm_motion", "FindPackageShare('rebotarm_msgs')")
        report = self.check()
        self.assertTrue(report["ok"], report)
        self.assertFalse(report["deployment_ready"])
        self.source("rebotarm_motion", "FindPackageShare('rebotarm_msgs')\nimport rebotarm_msgs")
        self.assertFalse(self.check()["ok"])

    def test_resolved_exception_must_be_removed(self):
        self.rules["undeclared_resource_debt"] = [{
            "source": "rebotarm_motion", "target": "rebotarm_msgs",
            "issue": "ARCH-test", "exit_gate": "S1", "reason": "fixture",
        }]
        self.assertIn("stale resource exception", "\n".join(self.check()["errors"]))

    def test_sdk_exception_is_limited_to_exact_file_and_module(self):
        self.rules["sdk_import_debt"] = [{
            "path": "src/rebotarm_motion/node.py", "module": "reBotArm_control_py.kinematics",
            "issue": "ARCH-test", "exit_gate": "S1", "reason": "fixture",
        }]
        self.source("rebotarm_motion", "from reBotArm_control_py.kinematics import compute_fk")
        self.assertTrue(self.check()["ok"])
        self.assertFalse(self.check()["deployment_ready"])
        self.source("rebotarm_motion", "import reBotArm_control_py.actuator", filename="other.py")
        self.assertIn("hardware SDK", "\n".join(self.check()["errors"]))

    def test_manifest_cycle_is_rejected(self):
        self.rules["packages"]["rebotarm_msgs"]["resources"] = ["rebotarm_motion"]
        self.package("rebotarm_msgs", deps=("rebotarm_motion",))
        self.assertIn("package.xml cycle", "\n".join(self.check()["errors"]))

    def test_compatibility_package_cannot_grow_implementations(self):
        self.rules["packages"]["rebotarm_interactive_control"] = copy.deepcopy(self.rules["packages"]["rebotarm_msgs"])
        self.package("rebotarm_interactive_control")
        self.source("rebotarm_interactive_control", "class NewBusinessLogic: pass")
        self.assertIn("must only forward", "\n".join(self.check()["errors"]))

    def test_external_ros_import_needs_a_direct_dependency(self):
        self.rules["external_modules"] = {"moveit_msgs": "moveit_msgs"}
        self.source("rebotarm_motion", "from moveit_msgs.srv import GetMotionPlan")
        self.assertIn("external module moveit_msgs", "\n".join(self.check()["errors"]))
        self.package("rebotarm_motion", deps=("rebotarm_msgs", "moveit_msgs"))
        self.assertTrue(self.check()["ok"])

    def test_external_launch_node_needs_a_direct_dependency(self):
        self.source("rebotarm_motion", "Node(package='robot_state_publisher')")
        self.assertIn("external resource package", "\n".join(self.check()["errors"]))
        self.package("rebotarm_motion", deps=("rebotarm_msgs", "robot_state_publisher"))
        self.assertTrue(self.check()["ok"])

    def test_recording_service_cannot_move_into_another_package(self):
        self.rules["service_owners"] = {"teleop/teach_record/": ["rebotarm_teach"]}
        self.source("rebotarm_motion", "node.create_service(Trigger, f'/{namespace}/teleop/teach_record/start', callback)")
        self.assertIn("belongs to rebotarm_teach", "\n".join(self.check()["errors"]))

    def test_frontend_cannot_publish_execution_feedback(self):
        self.rules["publisher_owners"] = {"/gripper/state": ["rebotarm_simulation", "rebotarmcontroller"]}
        self.source("rebotarm_motion", "node.create_publisher(JointMotorState, f'/{namespace}/gripper/state', 10)")
        self.assertIn("create_publisher /gripper/state belongs to", "\n".join(self.check()["errors"]))


if __name__ == "__main__":
    unittest.main()
