from pathlib import Path

import numpy as np
import pytest

from rebotarm_simulation.pointcloud_proxy import estimate_aabb_proxy, write_box_proxy_scene


ROOT = Path(__file__).resolve().parents[1]


def test_estimator_trims_outlier_and_adds_margin():
    points = np.asarray([[0.2, -0.1, 0.0], [0.3, 0.1, 0.2]] * 20, dtype=float)
    points = np.vstack([points, [[10.0, 10.0, 10.0], [np.nan, 0, 0]]])
    proxy = estimate_aabb_proxy(points, trim_quantile=0.02, margin_m=0.003)
    assert proxy.point_count == 41
    assert proxy.center_xyz == pytest.approx((0.25, 0.0, 0.1))
    assert proxy.half_extents_xyz == pytest.approx((0.053, 0.103, 0.103), abs=1e-3)


def test_proxy_scene_preserves_source_and_loads_in_mujoco(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    points = np.asarray([[0.25, -0.03, 0.0], [0.31, 0.03, 0.18]] * 10, dtype=float)
    proxy = estimate_aabb_proxy(points, trim_quantile=0.0, margin_m=0.002)
    source = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"
    target = tmp_path / "proxy_scene.xml"
    write_box_proxy_scene(source, target, proxy)
    source_text = source.read_text(encoding="utf-8")
    assert "bottle_body" in source_text
    assert "pointcloud_proxy" in target.read_text(encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(target))
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    assert model.nbody > 0
    assert np.isfinite(data.qpos).all()
