#!/usr/bin/env python3
"""Validate or apply the reviewed reBotArm_control_py safety patch.

This tool never imports the SDK, opens a bus, or constructs hardware objects.
It only inspects and optionally patches a pinned Git checkout.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
PATCH_PATH = ROOT / "patches" / "rebotarm_control_py" / "0001-feedback-and-safe-home-safety.patch"
PATCH_SHA256 = "b24ed757f63d436d206323e4c47b024fda44d0266f8e0f589fb0543ec67eadc2"
PINNED_COMMIT = "6a49302804f25e624995e771acb6d61896d1856d"
PATCHED_PATHS = (
    "config/arm.yaml",
    "reBotArm_control_py/actuator/arm.py",
    "reBotArm_control_py/controllers/arm_endpos_controller.py",
)


def _run(source: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(source), *args],
        check=check,
        text=True,
        encoding="utf-8",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _normalise(text: str) -> str:
    return text.replace("\r\n", "\n").rstrip("\n") + "\n"


def validate_patch_artifact() -> None:
    digest = hashlib.sha256(PATCH_PATH.read_bytes()).hexdigest()
    if digest != PATCH_SHA256:
        raise RuntimeError(
            f"SDK patch digest mismatch: expected {PATCH_SHA256}, found {digest}"
        )


def inspect_checkout(source: Path) -> str:
    validate_patch_artifact()
    if not (source / ".git").exists():
        raise RuntimeError(f"SDK source is not a Git checkout: {source}")
    head = _run(source, "rev-parse", "HEAD").stdout.strip()
    if head != PINNED_COMMIT:
        raise RuntimeError(f"SDK checkout must be {PINNED_COMMIT}, found {head}")
    untracked = _run(source, "ls-files", "--others", "--exclude-standard").stdout.strip()
    if untracked:
        raise RuntimeError("SDK checkout contains untracked files: " + untracked)

    actual = _run(
        source,
        "diff",
        "--binary",
        "--no-ext-diff",
        "HEAD",
        "--",
        *PATCHED_PATHS,
    ).stdout
    expected = PATCH_PATH.read_text(encoding="utf-8")
    other = _run(
        source,
        "diff",
        "--name-only",
        "HEAD",
        "--",
        *(":(exclude)" + path for path in PATCHED_PATHS),
    ).stdout.strip()
    if other:
        raise RuntimeError("SDK checkout contains unrelated tracked changes: " + other)
    if _normalise(actual) == _normalise(expected):
        return "already_applied"
    if actual.strip():
        raise RuntimeError("SDK checkout contains changes other than the reviewed patch")
    forward = _run(source, "apply", "--check", str(PATCH_PATH), check=False)
    if forward.returncode != 0:
        raise RuntimeError("reviewed SDK patch does not apply cleanly: " + forward.stderr.strip())
    return "ready"


def apply_patch(source: Path) -> str:
    state = inspect_checkout(source)
    if state == "already_applied":
        return state
    _run(source, "apply", str(PATCH_PATH))
    if inspect_checkout(source) != "already_applied":
        raise RuntimeError("SDK patch post-apply verification failed")
    return "applied"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=ROOT / "third_party" / "reBotArm_control_py",
        help="Pinned reBotArm_control_py Git checkout",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Apply the patch after all read-only checks pass",
    )
    args = parser.parse_args()
    source = args.source.expanduser().resolve()
    state = apply_patch(source) if args.apply else inspect_checkout(source)
    print(f"reBotArm_control_py safety patch: {state}; source={source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
