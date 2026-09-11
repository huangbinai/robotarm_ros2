"""Explicit local model paths; never downloads weights during node startup."""
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory


def default_workspace_path(relative: str) -> str:
    root = os.environ.get("REBOTARM_WORKSPACE", "").strip()
    if root:
        return str(Path(root).expanduser() / relative)
    for parent in Path(__file__).resolve().parents:
        if (parent / "rebotarm_dependencies.repos").is_file():
            return str(parent / relative)
    return ""


def yolo_model_path(explicit: str) -> Path:
    if explicit.strip():
        path = Path(explicit).expanduser()
    else:
        preferred = default_workspace_path("tools/yolo26s-seg.pt")
        path = Path(preferred) if preferred and Path(preferred).is_file() else (
            Path(get_package_share_directory("rebotarm_vision")) / "models/yolo11n-seg.pt"
        )
    if not path.is_file():
        raise FileNotFoundError(f"YOLO weights missing: {path}; set yolo_model_path explicitly")
    return path
