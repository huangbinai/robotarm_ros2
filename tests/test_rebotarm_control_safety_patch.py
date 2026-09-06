from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "apply_rebotarm_control_safety_patch.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("sdk_patch_tool", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sdk_patch_artifact_has_reviewed_digest() -> None:
    module = _load_module()
    module.validate_patch_artifact()
