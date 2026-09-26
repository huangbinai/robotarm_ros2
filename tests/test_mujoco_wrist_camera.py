from pathlib import Path
import os
import numpy as np
import pytest
import yaml
from rebotarm_simulation.virtual_camera import VirtualCameraConfig, VirtualCameraRenderer, camera_optical_transform
ROOT=Path(__file__).resolve().parents[1]

def test_wrist_camera_tracks_arm_with_constant_calibrated_optical_transform():
    os.environ.setdefault('MUJOCO_GL','egl')
    mj=pytest.importorskip('mujoco')
    m=mj.MjModel.from_xml_path(str(ROOT/'src/rebotarm_simulation/models/rebotarm/scene.xml'))
    d=mj.MjData(m)
    h=yaml.safe_load((ROOT/'src/rebotarm_vision/config/handeye.yaml').read_text())['handeye']
    expected_t=np.array([h['translation'][k] for k in 'xyz'])
    expected_q=np.array([h['rotation'][k] for k in 'xyzw']);expected_q/=np.linalg.norm(expected_q)
    parent=m.body(h['parent_frame']).id;camera=m.camera('wrist_camera').id
    images=[];poses=[]
    for joints in [[0,-.1,-.2,.2,0,0],[.5,-.5,-.7,.4,.2,-.3]]:
        mj.mj_resetDataKeyframe(m,d,0);d.qpos[:6]=joints;mj.mj_forward(m,d)
        poses.append(d.cam_xpos[camera].copy())
        transform=camera_optical_transform(camera_position_world=d.cam_xpos[camera],
            camera_rotation_world_mujoco=d.cam_xmat[camera].reshape(3,3),
            parent_position_world=d.xpos[parent],parent_rotation_world=d.xmat[parent].reshape(3,3),
            parent_frame_id='end_link',child_frame_id='mujoco_wrist_camera_optical_frame')
        assert transform.translation_xyz==pytest.approx(expected_t,abs=1e-9)
        assert abs(np.dot(transform.rotation_xyzw,expected_q))==pytest.approx(1.,abs=1e-9)
        renderer=VirtualCameraRenderer(mj,m,d,VirtualCameraConfig(camera_name='wrist_camera',
            parent_body_name='end_link',parent_frame_id='end_link',frame_id='mujoco_wrist_camera_optical_frame'))
        try:
            frame=renderer.render();images.append(frame.rgb.copy())
            assert frame.depth_mm.shape==(480,640)
            assert renderer.intrinsics.fx==pytest.approx(519.422,abs=1e-4)
        finally:renderer.close()
    assert np.linalg.norm(poses[0]-poses[1])>.01
    assert np.any(images[0]!=images[1])


def test_wrist_sensor_filters_debug_and_own_shell_without_hiding_gripper():
    os.environ.setdefault('MUJOCO_GL','egl')
    mj=pytest.importorskip('mujoco')
    m=mj.MjModel.from_xml_path(str(ROOT/'src/rebotarm_simulation/models/rebotarm/scene.xml'))
    d=mj.MjData(m);mj.mj_resetDataKeyframe(m,d,0);d.qpos[:6]=[0,-.1,-.2,.2,0,0];mj.mj_forward(m,d)
    before_groups=m.geom_group.copy();before_camera=m.cam_pos.copy();before_mass=m.body_mass.copy()
    config=VirtualCameraConfig(camera_name='wrist_camera',parent_body_name='end_link',parent_frame_id='end_link',frame_id='mujoco_wrist_camera_optical_frame')
    r=VirtualCameraRenderer(mj,m,d,config)
    try:
        frame=r.render()
        # All three rendering passes share the same scene filtering. Inspect IDs
        # after rendering; do not merely assert the option was configured.
        r._renderer.enable_segmentation_rendering()
        ids=r._renderer.render()
        assert not np.any(ids[:,:,1]==int(mj.mjtObj.mjOBJ_SITE))
        visible=set(ids[:,:,0][ids[:,:,1]==int(mj.mjtObj.mjOBJ_GEOM)].tolist())
        assert m.geom('gemini2_camera_visual').id not in visible
        assert all(m.geom_group[i]!=3 for i in visible)
        for side in ['left','right']:
            assert m.geom(f'{side}_finger_link_{side}_finger_visual').id in visible
        # A removed site must reveal real scene depth, not a sphere-depth ghost.
        assert frame.depth_mm[180,448]==0 or frame.depth_mm[180,448]>100
        assert np.array_equal(m.geom_group,before_groups)
        assert np.array_equal(m.cam_pos,before_camera)
        assert np.array_equal(m.body_mass,before_mass)
        # Overall viewer/fixed sensor retains the real shell and physical model.
        assert m.geom('gemini2_camera_visual').group==2
        assert m.geom('gemini2_camera_collision').contype!=0
    finally:r.close()
