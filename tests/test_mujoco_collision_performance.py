from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import statistics
from time import perf_counter

import pytest


ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = ROOT / "src/rebotarm_simulation/config/mujoco_collision_baseline.json"
HASH_PATTERN = re.compile(r"[0-9a-f]{64}")


def benchmark_scene(scene: Path, steps: int = 10_000) -> dict[str, int | float]:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    peak_contacts = 0
    started = perf_counter()
    for step in range(steps):
        mujoco.mj_step(model, data)
        peak_contacts = max(peak_contacts, int(data.ncon))
        for name in ("qpos", "qvel", "actuator_force"):
            values = getattr(data, name)
            if not all(math.isfinite(float(value)) for value in values):
                raise AssertionError(f"non-finite {name} after MuJoCo step {step + 1}")
    elapsed_seconds = perf_counter() - started
    return {
        "steps": steps,
        "elapsed_seconds": elapsed_seconds,
        "realtime_factor": steps * float(model.opt.timestep) / elapsed_seconds,
        "peak_contacts": peak_contacts,
    }


def test_primitive_collision_baseline_schema() -> None:
    assert BASELINE_PATH.is_file(), f"missing collision baseline: {BASELINE_PATH}"
    baseline = json.loads(BASELINE_PATH.read_text(encoding="utf-8"))

    assert set(baseline) == {
        "schema_version",
        "model_kind",
        "steps",
        "samples",
        "mujoco_version",
        "python_version",
        "platform",
        "cpu_count",
        "scene_sha256",
        "robot_sha256",
        "measurements",
        "median_elapsed_seconds",
    }
    assert baseline["schema_version"] == 1
    assert baseline["model_kind"] == "primitive_collision"
    assert baseline["steps"] == 10_000
    assert baseline["samples"] == 3
    assert baseline["mujoco_version"] == "3.10.0"
    assert isinstance(baseline["python_version"], str) and baseline["python_version"]
    assert isinstance(baseline["platform"], str) and baseline["platform"]
    assert isinstance(baseline["cpu_count"], int) and baseline["cpu_count"] > 0
    assert HASH_PATTERN.fullmatch(baseline["scene_sha256"])
    assert HASH_PATTERN.fullmatch(baseline["robot_sha256"])

    measurements = baseline["measurements"]
    assert isinstance(measurements, list) and len(measurements) == 3
    assert all(set(item) == {"steps", "elapsed_seconds", "realtime_factor", "peak_contacts"} for item in measurements)
    assert all(item["steps"] == 10_000 for item in measurements)
    assert all(isinstance(item["elapsed_seconds"], (int, float)) and item["elapsed_seconds"] > 0 for item in measurements)
    assert all(isinstance(item["realtime_factor"], (int, float)) and item["realtime_factor"] > 0 for item in measurements)
    assert all(isinstance(item["peak_contacts"], int) and item["peak_contacts"] >= 0 for item in measurements)

    expected_median = statistics.median(item["elapsed_seconds"] for item in measurements)
    assert baseline["median_elapsed_seconds"] == expected_median
    assert baseline["median_elapsed_seconds"] > 0


def test_benchmark_scene_reports_real_mujoco_steps_and_finite_state() -> None:
    scene = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"
    measurement = benchmark_scene(scene, steps=25)

    assert set(measurement) == {"steps", "elapsed_seconds", "realtime_factor", "peak_contacts"}
    assert measurement["steps"] == 25
    assert measurement["elapsed_seconds"] > 0
    assert measurement["realtime_factor"] > 0
    assert isinstance(measurement["peak_contacts"], int)
    assert measurement["peak_contacts"] >= 0


@pytest.mark.skipif(
    os.environ.get("REBOTARM_RUN_COLLISION_BENCHMARK") != "1",
    reason="set REBOTARM_RUN_COLLISION_BENCHMARK=1 to run the 10k collision benchmark",
)
def test_primitive_collision_10k_benchmark() -> None:
    baseline = json.loads(BASELINE_PATH.read_text(encoding="utf-8"))
    scene = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"
    measurements = [benchmark_scene(scene, steps=baseline["steps"]) for _ in range(3)]
    result = {
        "measurements": measurements,
        "median_elapsed_seconds": statistics.median(
            item["elapsed_seconds"] for item in measurements
        ),
    }

    print(json.dumps(result, indent=2, sort_keys=True))
