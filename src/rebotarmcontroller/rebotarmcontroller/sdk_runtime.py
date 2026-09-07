from __future__ import annotations

import sys
from pathlib import Path
from typing import MutableSequence

import yaml


class RebotSdkLocator:
    """Locate the external SDK and create runtime-only config overrides."""

    def __init__(
        self,
        *,
        workspace_root: Path,
        cwd: Path | None = None,
        user_home: Path | None = None,
        import_path: MutableSequence[str] | None = None,
    ) -> None:
        self._workspace_root = Path(workspace_root)
        self._cwd = Path.cwd() if cwd is None else Path(cwd)
        self._user_home = Path.home() if user_home is None else Path(user_home)
        self._import_path = sys.path if import_path is None else import_path

    @classmethod
    def for_module(cls, module_path: str | Path) -> "RebotSdkLocator":
        return cls(workspace_root=Path(module_path).resolve().parents[3])

    def candidates(self) -> list[Path]:
        return [
            self._workspace_root / "third_party" / "reBotArm_control_py",
            self._workspace_root / "third_party" / "reBotArm_control_py-main",
            self._workspace_root / "sdk" / "reBotArm_control_py",
            self._cwd / "third_party" / "reBotArm_control_py",
            self._cwd / "sdk" / "reBotArm_control_py",
            self._user_home / "robotarm_ros2" / "third_party" / "reBotArm_control_py",
            self._user_home / "robotarm_ros2" / "sdk" / "reBotArm_control_py",
            self._user_home / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
        ]

    def ensure_importable(self) -> Path:
        candidates = self.candidates()
        for root in candidates:
            if (root / "reBotArm_control_py").is_dir():
                root_str = str(root)
                if root_str not in self._import_path:
                    self._import_path.insert(0, root_str)
                return root
        rendered = "\n".join(f"  - {path}" for path in candidates)
        raise FileNotFoundError(
            "Cannot find reBotArm_control_py. Clone it into one of:\n"
            f"{rendered}"
        )

    @staticmethod
    def arm_config(root: Path) -> Path:
        return Path(root) / "config" / "arm.yaml"

    @staticmethod
    def gripper_config(root: Path) -> Path:
        return Path(root) / "config" / "gripper.yaml"

    @staticmethod
    def with_channel_override(
        config_path: Path,
        channel: str,
        *,
        temporary_directory: Path = Path("/tmp") / "rebotarm_ros2",
    ) -> Path:
        normalized_channel = str(channel or "").strip()
        if not normalized_channel or normalized_channel.lower() == "auto":
            return config_path
        with Path(config_path).open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        data["channel"] = normalized_channel
        temporary_directory.mkdir(parents=True, exist_ok=True)
        override_path = temporary_directory / "arm_channel_override.yaml"
        with override_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, sort_keys=False)
        return override_path
