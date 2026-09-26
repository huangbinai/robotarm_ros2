# RGB-D / CameraInfo and MuJoCo intrinsics audit

2026-09-26. Software and simulated output audit; Gemini 2 was absent from USB
inventory and no real vision node was running. No actual device calibration or
real image/CameraInfo consistency acceptance is claimed.

## Production data path

`visual_grasp_system.launch.py` → vision launch → `camera_ubuntu.yaml`:
color 640x480 MJPG @30 fps; native depth 640x400 @30 fps; hardware D2C enabled.
The ROS loop is capped at 15 Hz, distinct from sensor stream rate. Production
YOLO receives the color BGR image; its internal inference preprocessing is not
a change to the published camera stream calibration. GraspNet subscribes to
`/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth/camera_info`.
The depth stream presented to it is aligned-to-color (expected 640x480), not
native depth coordinates. Profile selection and actual image shape still require
runtime verification on the connected camera.

Gemini2Driver reads fx/fy/cx/cy and distortion from the selected SDK profiles;
D2C depth CameraInfo copies color intrinsics. Native-depth intrinsics are stored
separately in driver metadata. ROS distortion is rational_polynomial with
[k1,k2,p1,p2,k3,k4,k5,k6]. The message converter preserves these fields.

Important gaps identified, not silently changed in this audit:
- VisionNode currently accepts SDK info without matching dimensions against the
  actual image; missing metadata has an approximate fallback, not calibration.
- GraspNet's CameraInfo callback retains only fx/fy/cx/cy, not D, dimensions or
  CameraInfo timestamps. Point-cloud construction is a pinhole projection.
- D2C alignment alone does not establish color lens rectification. For nonzero
  real D, raw-image distortion handling must be evaluated before claiming
  geometrically accurate pinhole backprojection.

## Simulation change

Canonical `scene.xml` fixed_camera now explicitly uses physical camera intrinsics:
640x480, fx=519.422, fy=519.17, cx=320.681, cy=241.362 pixels. These numbers come
from the repository's legacy ordinary_grasp section in camera_ubuntu.yaml; they
are a repository reference, NOT a fresh device reading and not the parameters
used as a constant by the active GraspNet pipeline. A regression compares them
against that source to detect drift.

The renderer's CameraInfo now derives from MuJoCo sensorsize/cam_intrinsic,
including unequal focal lengths and noncentral principal points. Legacy cameras
without sensorsize retain fovy fallback. Pixel-center and resizing conventions
are tested against rendered segmentation locations and depth at two resolutions.
This changes real projection as well as metadata, not just a CameraInfo label.

Simulation RGB and depth are ideal aligned pinhole images; D=[0,0,0,0,0],
plumb_bob is intentional. We do not claim to reproduce real lens distortion,
native stereo depth generation, holes/noise or sensor timing. Camera rate remains
15 Hz matching the ROS loop target, not the sensor's 30 fps acquisition.
F2 update: the output camera now defaults to wrist_camera, generated under
end_link from handeye.yaml. ROS optical axes are converted to MuJoCo camera axes
with a local 180-degree X rotation. No second factory color/depth transform is
applied. Static TF end_link -> mujoco_wrist_camera_optical_frame matches the
calibration; robot_state_publisher provides the moving world-to-end chain.
The fixed_camera remains available in the model. Camera/holder payload geometry
and production handeye.yaml remain unchanged. The wrist camera uses the same
repository-reference K, still awaiting live SDK validation.

Enable output with `ros2 launch rebotarm_simulation mujoco_sim.launch.py
enable_virtual_camera:=true`. This launches only simulation. TF consumers that
need world/base coordinates additionally need the existing robot TF publisher.
Camera rendering remains disabled by default for motion-only launches.
Regenerate the model with urdf_to_mjcf and rebuild simulation whenever handeye.yaml
changes: generation consumes source calibration; runtime loads packaged MJCF.

## Verification

- Compiled model K matches reference within float precision.
- EGL projection tests cover fx != fy, off-center principal point, 640x480 and
  320x240 output, RGB/depth shape and depth values.
- Actual installed ROS MuJoCo launch on isolated domain218 published at least
  five same-stamp Image/CameraInfo pairs per stream; dimensions, frame IDs,
  finite K/D/P and distortion model passed. Evidence JSON is in
  Agent/evidence/maintenance/2026-09-26-mujoco-camera-metadata.json.
- Temporary runtime stopped; SIGINT interrupted renderer join (KeyboardInterrupt),
  so clean graceful renderer shutdown was not established; process exited.
- No real camera, motor command or grasp execution was performed.

## Repeat read-only metadata check

With an already running camera publisher in the same ROS domain:

```bash
cd /home/huangbin/robotarm_ros2
source tools/source_local_environment.bash
python3 tools/check_rgbd_camera_info.py --timeout 15 --frames 5
```

The tool only subscribes. Passing proves message metadata consistency, not optical
calibration accuracy or that GraspNet compensates distortion. Save the real SDK
CameraInfo snapshot before replacing the simulation reference values.

## F2 verification (eye-in-hand)

Two different joint poses preserve the calibrated parent-relative optical transform
and change world camera position and rendered RGB. ROS runtime captured five paired
frames per stream plus transient-local TF: end_link -> mujoco_wrist_camera_optical_frame,
translation [-0.085,0.010,0.045], quaternion matching handeye.yaml.
The sample view includes substantial nearby model geometry: the independently fitted
camera shell/holder may occlude the calibrated optical view. This implementation
does not relocate the calibrated camera or hide geometry to conceal that discrepancy.
Runtime shutdown exited cleanly but emitted a late-publish invalid-context warning;
this is not a hardware error or proof of a graceful camera worker shutdown.
Final system suite: 755 passed/20 skipped; actual MuJoCo camera tests: 4 passed;
layering18, build, compileall and generated-model consistency passed.

## Sensor-only visibility correction

The previous gray sphere was the wrist_camera_mount diagnostic site, not a gripper
part. VirtualCameraRenderer now disables sites and duplicate collision geoms in
all RGB/depth/segmentation passes. For wrist_camera only, a private rendering model
excludes gemini2_camera_visual (the coarse shell has no optical aperture model).
The physics model, shell collision/mass, handeye pose and external viewer geometry
are unchanged; gripper and mount visuals remain eligible for sensor rendering.
This is an explicit sensor-rendering approximation, NOT mechanical re-registration
or proof that the calibrated optical frame matches the actual housing. No real
handeye calibration or real-camera image was modified.
Regression checks rendered segmentation IDs (no site/shell/collision proxy, both
fingers remain visible), depth and unchanged shared model arrays.
