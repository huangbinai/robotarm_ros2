"""Check image geometry, not just CameraInfo metadata, for off-center lenses."""
from pathlib import Path
import os
import numpy as np
import pytest
import yaml
from rebotarm_simulation.virtual_camera import VirtualCameraConfig, VirtualCameraRenderer, model_camera_intrinsics

ROOT = Path(__file__).resolve().parents[1]


def test_default_sim_camera_matches_repository_reference_intrinsics():
    mj = pytest.importorskip('mujoco')
    m = mj.MjModel.from_xml_path(str(ROOT/'src/rebotarm_simulation/models/rebotarm/scene.xml'))
    k = model_camera_intrinsics(m, m.camera('fixed_camera').id, 640, 480)
    ref = yaml.safe_load((ROOT/'src/rebotarm_vision/config/camera_ubuntu.yaml').read_text())
    ref = ref['rebotarm_ordinary_grasp_node']['ros__parameters']
    for key in ['fx', 'fy', 'cx', 'cy']:
        assert getattr(k, key) == pytest.approx(ref['ordinary_grasp.'+key], abs=1e-4)
    assert (k.width,k.height)==(640,480)


@pytest.mark.parametrize('width,height', [(640,480),(320,240)])
def test_rendered_pixels_and_depth_match_reported_offcenter_intrinsics(width,height):
    os.environ.setdefault('MUJOCO_GL','egl')
    mj = pytest.importorskip('mujoco')
    # Deliberately unequal focal lengths and large principal offsets expose
    # fovy-only, sign, half-pixel and resolution-scaling mistakes.
    pts=[(-.15,-.08,.8),(.12,.07,.9),(0.,0.,.7)]
    bodies=''.join(f'<body name="target{i}" pos="{x} {-y} {-z}"><geom type="sphere" size="0.008" rgba="1 .1 .1 1"/></body>' for i,(x,y,z) in enumerate(pts))
    m=mj.MjModel.from_xml_string(f'''<mujoco><visual><global offwidth="640" offheight="480"/></visual><worldbody>
      <body name="base"><camera name="test" resolution="640 480" sensorsize=".0064 .0048" focalpixel="500 540" principalpixel="-20 15"/></body>
      {bodies}</worldbody></mujoco>''')
    d=mj.MjData(m);mj.mj_forward(m,d)
    config=VirtualCameraConfig(camera_name='test',parent_body_name='base',width=width,height=height,annotation_bodies=tuple(f'target{i}' for i in range(3)))
    r=VirtualCameraRenderer(mj,m,d,config)
    try:
        frame=r.render();k=r.intrinsics
        assert frame.rgb.shape==(height,width,3)
        assert frame.depth_mm.shape==(height,width)
        boxes={a.class_name:a for a in frame.annotations}
        for i,(x,y,z) in enumerate(pts):
            a=boxes[f'target{i}'];u=k.fx*x/z+k.cx;v=k.fy*y/z+k.cy
            assert a.center_u == pytest.approx(u,abs=1.)
            assert a.center_v == pytest.approx(v,abs=1.)
            depth=float(frame.depth_mm[round(v),round(u)])*.001
            assert z-.010 < depth < z+.002
    finally:r.close()
