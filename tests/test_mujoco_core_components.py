from __future__ import annotations

from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"


def test_model_index_owns_resolved_ids_and_stable_identity() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.model_contract import JOINT_NAMES, TEST_CUBE_BODY_NAME
    from rebotarm_simulation.mujoco_model_index import MujocoModelIndex
    from rebotarm_simulation.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(SCENE) as simulation:
        model, _ = simulation.render_adapter.handles()
        index = MujocoModelIndex(simulation._mj, model)

        assert len(index.joint_ids) == len(index.actuator_ids) == len(JOINT_NAMES)
        assert TEST_CUBE_BODY_NAME in index.free_bodies
        assert index.model_dimensions[-1] > 0
        assert index.model_fingerprint == simulation._model_fingerprint


def test_contact_reader_preserves_public_contact_conversion() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.mujoco_contact_reader import MujocoContactReader
    from rebotarm_simulation.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(SCENE) as simulation:
        simulation.step(5)
        model, data = simulation.render_adapter.handles()
        reader = MujocoContactReader(simulation._mj, model, data)

        assert reader.read() == simulation.get_contacts()


def test_scene_runtime_seed_is_deterministic_and_updates_public_state() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.model_contract import TEST_CUBE_BODY_NAME
    from rebotarm_simulation.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(SCENE) as simulation:
        first = simulation.randomize_scene(seed=413)
        second = simulation.randomize_scene(seed=413)

        assert first == second
        assert simulation.get_state().object_poses[TEST_CUBE_BODY_NAME] == pytest.approx(
            second.cube_pose
        )
