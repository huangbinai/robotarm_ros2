"""Build a conservative MuJoCo collision proxy from an object point cloud.

The input points must already be expressed in the scene/world frame. This module
does not infer hidden surfaces, mass or friction; it only creates a bounded
geometric proxy for planning and simulation experiments.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class PointCloudProxy:
    center_xyz: tuple[float, float, float]
    half_extents_xyz: tuple[float, float, float]
    point_count: int
    trim_quantile: float
    margin_m: float

    @property
    def size_xyz(self) -> tuple[float, float, float]:
        return tuple(2.0 * value for value in self.half_extents_xyz)

    def to_dict(self) -> dict[str, object]:
        return {
            "center_xyz": list(self.center_xyz),
            "half_extents_xyz": list(self.half_extents_xyz),
            "size_xyz": list(self.size_xyz),
            "point_count": self.point_count,
            "trim_quantile": self.trim_quantile,
            "margin_m": self.margin_m,
        }


def estimate_aabb_proxy(
    points_xyz: np.ndarray,
    *,
    trim_quantile: float = 0.02,
    margin_m: float = 0.003,
    min_extent_m: float = 0.005,
) -> PointCloudProxy:
    """Estimate a robust axis-aligned proxy from finite Nx3 points.

    Trimming removes a small number of depth outliers. The result is a planning
    proxy, not a fitted mesh or a statement about hidden geometry.
    """
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 8:
        raise ValueError("at least 8 finite points are required")
    quantile = float(trim_quantile)
    margin = float(margin_m)
    minimum = float(min_extent_m)
    if not 0.0 <= quantile < 0.5:
        raise ValueError("trim_quantile must be in [0, 0.5)")
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("margin_m must be finite and non-negative")
    if not np.isfinite(minimum) or minimum <= 0.0:
        raise ValueError("min_extent_m must be finite and positive")
    # Select observed order statistics. Linear interpolation can still include
    # a distant lone outlier in a small cloud, producing a fictitiously huge box.
    lower = np.quantile(points, quantile, axis=0, method="nearest")
    upper = np.quantile(points, 1.0 - quantile, axis=0, method="nearest")
    center = (lower + upper) * 0.5
    half = np.maximum((upper - lower) * 0.5 + margin, minimum * 0.5)
    return PointCloudProxy(
        center_xyz=tuple(float(value) for value in center),
        half_extents_xyz=tuple(float(value) for value in half),
        point_count=int(len(points)),
        trim_quantile=quantile,
        margin_m=margin,
    )


def write_box_proxy_scene(
    base_scene: str | Path,
    output_scene: str | Path,
    proxy: PointCloudProxy,
    *,
    body_name: str = "bottle",
    geom_name: str = "pointcloud_proxy",
) -> Path:
    """Write a copy of a scene with the selected free-body geometry replaced by a box."""
    source = Path(base_scene)
    output = Path(output_scene)
    root = ET.parse(source).getroot()
    include = root.find("include[@file='robot.xml']")
    if include is None:
        raise ValueError("base scene must include robot.xml")
    include.set("file", str((source.resolve().parent / "robot.xml").resolve()))
    body = root.find(f".//body[@name='{body_name}']")
    if body is None or body.find("freejoint") is None:
        raise ValueError(f"scene must contain freejoint body {body_name!r}")
    for geom in list(body.findall("geom")):
        body.remove(geom)
    body.set("pos", "0 0 0")
    size = " ".join(f"{value:.9g}" for value in proxy.half_extents_xyz)
    ET.SubElement(
        body,
        "geom",
        {
            "name": geom_name,
            "type": "box",
            "size": size,
            "rgba": "0.12 0.42 0.82 1",
            "contype": "1",
            "conaffinity": "1",
            "friction": "0.8 0.02 0.001",
        },
    )
    key = root.find("keyframe/key[@name='home']")
    if key is not None:
        qpos = [float(value) for value in key.attrib["qpos"].split()]
        if len(qpos) < 15:
            raise ValueError("home keyframe does not contain the bottle free joint")
        qpos[8:11] = list(proxy.center_xyz)
        key.set("qpos", " ".join(f"{value:.9g}" for value in qpos))
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    output.write_text(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<!-- GENERATED point-cloud collision proxy; source scene is unchanged. -->\n"
        + ET.tostring(root, encoding="unicode")
        + "\n",
        encoding="utf-8",
    )
    return output


def load_points(path: str | Path) -> np.ndarray:
    source = Path(path)
    if source.suffix == ".npy":
        points = np.load(source)
    elif source.suffix == ".npz":
        archive = np.load(source)
        if "points" not in archive:
            raise ValueError("NPZ must contain an array named 'points'")
        points = archive["points"]
    elif source.suffix == ".json":
        payload = json.loads(source.read_text(encoding="utf-8"))
        points = payload.get("points") if isinstance(payload, dict) else payload
    else:
        raise ValueError("point cloud input must be .npy, .npz or .json")
    return np.asarray(points, dtype=np.float64)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build a MuJoCo box proxy from base-frame points")
    parser.add_argument("points", help="Nx3 .npy, .npz(points), or JSON point cloud")
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--output-scene", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--trim-quantile", type=float, default=0.02)
    parser.add_argument("--margin-m", type=float, default=0.003)
    args = parser.parse_args(argv)
    proxy = estimate_aabb_proxy(load_points(args.points), trim_quantile=args.trim_quantile, margin_m=args.margin_m)
    write_box_proxy_scene(args.base_scene, args.output_scene, proxy)
    Path(args.report).write_text(json.dumps(proxy.to_dict(), indent=2) + "\n", encoding="utf-8")
    print(json.dumps(proxy.to_dict(), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
