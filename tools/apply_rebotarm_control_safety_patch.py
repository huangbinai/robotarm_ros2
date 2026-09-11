#!/usr/bin/env python3
"""Validate or apply the reviewed reBotArm_control_py safety patch.

This tool never imports the SDK, opens a bus, or constructs hardware objects.
It only inspects and optionally patches a pinned Git checkout.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
PATCH_PATH = ROOT / "patches" / "rebotarm_control_py" / "0001-feedback-and-safe-home-safety.patch"
PATCH_SHA256 = "d01176ebea31ba2c0481c8908568b584630604f6aeed7b956a05d5a5a3891a8e"
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

    with tempfile.TemporaryDirectory(prefix="rebotarm-sdk-index-") as temp_name:
        actual_env = os.environ.copy()
        actual_env["GIT_INDEX_FILE"] = str(Path(temp_name) / "actual-index")
        subprocess.run(
            ["git", "-C", str(source), "read-tree", "HEAD"],
            check=True,
            env=actual_env,
        )
        subprocess.run(
            ["git", "-C", str(source), "add", "-A", "--", "."],
            check=True,
            env=actual_env,
        )
        actual_tree = subprocess.run(
            ["git", "-C", str(source), "write-tree"],
            check=True,
            env=actual_env,
            text=True,
            stdout=subprocess.PIPE,
        ).stdout.strip()

        expected_env = os.environ.copy()
        expected_env["GIT_INDEX_FILE"] = str(Path(temp_name) / "expected-index")
        subprocess.run(
            ["git", "-C", str(source), "read-tree", "HEAD"],
            check=True,
            env=expected_env,
        )
        subprocess.run(
            ["git", "-C", str(source), "apply", "--cached", str(PATCH_PATH)],
            check=True,
            env=expected_env,
        )
        expected_tree = subprocess.run(
            ["git", "-C", str(source), "write-tree"],
            check=True,
            env=expected_env,
            text=True,
            stdout=subprocess.PIPE,
        ).stdout.strip()

    if actual_tree == expected_tree:
        return "already_applied"
    actual = _run(source, "diff", "--name-only", "HEAD", "--").stdout.strip()
    if actual:
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
