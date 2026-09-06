from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

import numpy as np

geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")


class _Pose:
    pass


geometry_msgs_msg.Pose = _Pose
geometry_msgs.msg = geometry_msgs_msg
sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

tf_transformations = types.ModuleType("tf_transformations")
tf_transformations.euler_from_quaternion = lambda _q: (0.0, 0.0, 0.0)
tf_transformations.quaternion_from_matrix = lambda _m: (0.0, 0.0, 0.0, 1.0)
sys.modules.setdefault("tf_transformations", tf_transformations)

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "src" / "rebotarmcontroller"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rebotarmcontroller.hardware_manager import apply_gravity_compensation_tau_scale  # type: ignore[import-not-found]
from rebotarmcontroller.hardware_manager import HardwareManager
from rebotarmcontroller.mode_transition import ModeTransitionResult


class _FakeTransitionCoordinator:
    def __init__(self, *, enter_success=True, exit_success=True):
        self.calls = []
        self.enter_success = enter_success
        self.exit_success = exit_success

    def enter_gravity_compensation(self):
        self.calls.append("enter")
        return ModeTransitionResult(
            success=self.enter_success,
            source_mode="pos_vel",
            target_mode="mit",
            stage="GRAVITY_COMP" if self.enter_success else "ENTERING_GRAVITY_COMP",
            duration_sec=0.1,
            failure_reason="" if self.enter_success else "enter failed",
        )

    def exit_gravity_compensation(self):
        self.calls.append("exit")
        return ModeTransitionResult(
            success=self.exit_success,
            source_mode="mit",
            target_mode="pos_vel",
            stage="POS_VEL_HOLD" if self.exit_success else "EXIT_BLENDING",
            duration_sec=0.1,
            failure_reason="" if self.exit_success else "exit failed",
        )


class GravityCompensationCoreTests(unittest.TestCase):
    def test_tau_scale_leaves_default_gravity_torque_unchanged(self) -> None:
        tau = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

        scaled = apply_gravity_compensation_tau_scale(tau)

        np.testing.assert_allclose(scaled, tau)
        np.testing.assert_allclose(tau, np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))

    def test_start_gravity_compensation_delegates_to_transition_coordinator(self) -> None:
        manager = HardwareManager.__new__(HardwareManager)
        manager._connected = True
        manager._enabled = True
        manager._gravity_comp_active = False
        manager._mode_transition = _FakeTransitionCoordinator()

        manager.start_gravity_compensation()

        self.assertEqual(manager._mode_transition.calls, ["enter"])

    def test_stop_gravity_compensation_delegates_to_transition_coordinator(self) -> None:
        manager = HardwareManager.__new__(HardwareManager)
        manager._gravity_comp_active = True
        manager._mode_transition = _FakeTransitionCoordinator()

        manager.stop_gravity_compensation()

        self.assertEqual(manager._mode_transition.calls, ["exit"])

    def test_transition_failure_is_returned_as_runtime_error(self) -> None:
        manager = HardwareManager.__new__(HardwareManager)
        manager._connected = True
        manager._enabled = True
        manager._gravity_comp_active = False
        manager._mode_transition = _FakeTransitionCoordinator(enter_success=False)

        with self.assertRaisesRegex(RuntimeError, "ENTERING_GRAVITY_COMP: enter failed"):
            manager.start_gravity_compensation()


if __name__ == "__main__":
    unittest.main()
