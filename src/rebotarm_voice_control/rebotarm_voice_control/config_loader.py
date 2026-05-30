from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .models import VoiceControlConfig


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return loaded


def load_voice_control_config(config_root: str | Path) -> VoiceControlConfig:
    root = Path(config_root)
    return VoiceControlConfig(
        intents=_load_yaml(root / "intents.yaml"),
        named_poses=_load_yaml(root / "named_poses.yaml"),
        task_templates=_load_yaml(root / "task_templates.yaml"),
        safety_limits=_load_yaml(root / "safety_limits.yaml"),
    )
