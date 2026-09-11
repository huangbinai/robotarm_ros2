"""Read-only Open3D geometry for registered RGB-D and ROS grasp candidates."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import numpy as np

from .graspnet_inference import build_scene_cloud


@dataclass
class GraspScene:
    points: np.ndarray
    colors: np.ndarray
    grasps: list[tuple[np.ndarray, float, float]]


def candidate_transform(candidate, frame_id: str) -> np.ndarray:
    if not candidate.valid or candidate.header.frame_id != frame_id:
        raise ValueError("candidate is invalid or belongs to a different optical frame")
    pose = candidate.pose
    quaternion = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z], dtype=float)
    translation = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    if not np.isfinite(quaternion).all() or not np.isfinite(translation).all():
        raise ValueError("candidate pose contains non-finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError("candidate quaternion is zero")
    w, x, y, z = quaternion / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def build_grasp_scene(color_bgr, depth_mm, camera_info, candidates, *, top_n=5, max_points=30000) -> GraspScene:
    points, colors = build_scene_cloud(color_bgr=color_bgr, depth_mm=depth_mm, camera_info=camera_info)
    if max_points < 1 or top_n < 0:
        raise ValueError("max_points must be positive and top_n non-negative")
    if len(points) > max_points:
        indices = np.linspace(0, len(points)-1, max_points, dtype=int)
        points, colors = points[indices], colors[indices]
    ranked = []
    for candidate in candidates.candidates:
        try:
            if candidate.header.stamp != candidates.header.stamp:
                continue
            transform = candidate_transform(candidate, candidates.header.frame_id)
            width, score = float(candidate.jaw_width), float(candidate.confidence)
            if not math.isfinite(width) or not 0 < width <= 0.2 or not math.isfinite(score):
                continue
            ranked.append((transform, width, score))
        except ValueError:
            continue
    ranked.sort(key=lambda item: item[2], reverse=True)
    return GraspScene(points, colors, ranked[:top_n])


def scene_geometries(scene: GraspScene, *, finger_length=0.06):
    import open3d as o3d

    if not math.isfinite(finger_length) or finger_length <= 0:
        raise ValueError("display finger_length must be positive")
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(scene.points)
    cloud.colors = o3d.utility.Vector3dVector(scene.colors)
    geometries = [cloud]
    for index, (transform, width, _score) in enumerate(scene.grasps):
        # Candidate pose and opening are measured outputs. Finger dimensions are
        # display-only: this is not the robot's collision or execution model.
        thickness = 0.004
        color = [0.15, 1.0, 0.3] if index == 0 else [1.0, 0.3, 0.12]
        bars = [
            ((finger_length, thickness, thickness), (-finger_length, -width/2-thickness, -thickness/2)),
            ((finger_length, thickness, thickness), (-finger_length, width/2, -thickness/2)),
            ((thickness, width+2*thickness, thickness), (-finger_length-thickness, -width/2-thickness, -thickness/2)),
        ]
        for size, offset in bars:
            bar = o3d.geometry.TriangleMesh.create_box(*size)
            bar.translate(offset)
            bar.transform(transform)
            bar.paint_uniform_color(color)
            bar.compute_vertex_normals()
            geometries.append(bar)
        if index == 0:
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.035)
            axes.transform(transform)
            geometries.append(axes)
    return geometries


class Open3DGraspWindow:
    """GUI stays on the main thread; inference runs in a different ROS node."""

    def __init__(self, *, save_directory: Path, visible=True, point_size=2.0):
        import open3d as o3d

        self._o3d = o3d
        self._vis = o3d.visualization.VisualizerWithKeyCallback()
        if not self._vis.create_window("GraspNet | cloud + candidate poses | preview only", 1280, 800, visible=visible):
            raise RuntimeError("Open3D window creation failed; run in the Ubuntu desktop session")
        option = self._vis.get_render_option()
        option.point_size = float(point_size)
        option.background_color = np.array([0.035, 0.045, 0.055])
        self._save_directory = Path(save_directory)
        self._scene = None
        self._fitted = False
        self._save_requested = False
        self._reset_requested = False
        self._vis.register_key_callback(ord("S"), self._request_save)
        self._vis.register_key_callback(ord("R"), self._request_reset)

    def _request_save(self, _vis):
        self._save_requested = True
        return False

    def _request_reset(self, _vis):
        self._reset_requested = True
        return False

    def update(self, scene: GraspScene):
        self._scene = scene
        self._vis.clear_geometries()
        for geometry in scene_geometries(scene):
            self._vis.add_geometry(geometry, reset_bounding_box=False)
        if not self._fitted or self._reset_requested:
            self._vis.reset_view_point(True)
            view = self._vis.get_view_control()
            focus = scene.grasps[0][0][:3, 3] if scene.grasps else (np.median(scene.points, axis=0) if len(scene.points) else [0, 0, 0.5])
            view.set_lookat(focus)
            view.set_front([0.0, 0.0, -1.0])
            view.set_up([0.0, -1.0, 0.0])
            view.set_zoom(0.55)
            self._fitted = True
            self._reset_requested = False
        self._vis.update_renderer()

    def clear(self):
        self._scene = None
        self._vis.clear_geometries()
        self._vis.update_renderer()

    def poll(self):
        alive = self._vis.poll_events()
        self._vis.update_renderer()
        if self._save_requested and self._scene is not None:
            from datetime import datetime
            name = datetime.now().strftime("graspnet-%Y%m%d-%H%M%S-%f")
            self.save(self._save_directory / name)
        self._save_requested = False
        return alive

    def save(self, prefix: Path):
        if self._scene is None:
            raise ValueError("no matched scene to save")
        prefix = Path(prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        self._vis.capture_screen_image(str(prefix.with_suffix(".png")), do_render=True)
        cloud = self._o3d.geometry.PointCloud()
        cloud.points = self._o3d.utility.Vector3dVector(self._scene.points)
        cloud.colors = self._o3d.utility.Vector3dVector(self._scene.colors)
        if not self._o3d.io.write_point_cloud(str(prefix.with_suffix(".ply")), cloud):
            raise OSError("Open3D point cloud save failed")
        print(f"Saved Open3D preview: {prefix.with_suffix('.png')}", flush=True)

    def close(self):
        self._vis.destroy_window()
