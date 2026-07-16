# Sim2Real / Real2Sim Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a simulation-only Sim2Real/Real2Sim foundation with reproducible parameter randomization, validated JSONL trajectory logs, deterministic replay, and trajectory comparison.

**Architecture:** Keep `RebotArmMujoco` and `RebotArmReachEnv` responsible for physics and task execution. Add a dependency-light `sim2real` package responsible for schemas, randomization samples, logging, replay, and comparison; optional hooks on the environment will expose observations and state without moving file I/O into the simulator. Real hardware remains out of scope.

**Tech Stack:** Python 3.12, dataclasses, NumPy, PyYAML, MuJoCo, pytest, JSONL.

---

## File Map

- Create: `src/rebotarm_simulation/rebotarm_simulation/sim2real/__init__.py` — public exports.
- Create: `src/rebotarm_simulation/rebotarm_simulation/sim2real/schemas.py` — immutable sample and metric records.
- Create: `src/rebotarm_simulation/rebotarm_simulation/sim2real/randomization.py` — YAML-backed ranges and deterministic samples.
- Create: `src/rebotarm_simulation/rebotarm_simulation/sim2real/trajectory_log.py` — sample validation and JSONL persistence.
- Create: `src/rebotarm_simulation/rebotarm_simulation/sim2real/replay_compare.py` — action replay and metric comparison.
- Create: `src/rebotarm_simulation/config/sim2real_randomization.yaml` — zero-noise compatible defaults and bounded ranges.
- Modify: `src/rebotarm_simulation/rebotarm_simulation/mujoco_env.py` — optional randomization session and recorder-facing sample extraction.
- Modify: `src/rebotarm_simulation/setup.py` — install the YAML config if required by package resources.
- Create: `tests/test_sim2real_schemas.py` — dimensions, finite values, immutability.
- Create: `tests/test_sim2real_randomization.py` — YAML parsing, seed reproducibility, bounds, default compatibility.
- Create: `tests/test_sim2real_trajectory_log.py` — append, JSONL round-trip, monotonic time, invalid data.
- Create: `tests/test_sim2real_replay_compare.py` — replay determinism and comparison metrics.
- Modify: `tests/test_mujoco_env.py` — verify optional randomization and recorder integration does not alter default behavior.
- Modify: `src/rebotarm_simulation/README_mujoco.md` — document first-stage Sim2Real/Real2Sim APIs and commands.

### Task 1: Define immutable trajectory and metric schemas

**Files:** Create `schemas.py`; create `tests/test_sim2real_schemas.py`.

- [ ] Write tests for a valid `TrajectorySample` with six-joint arrays, optional seven-dimensional action, XYZW orientation, source `sim`, and finite values.
- [ ] Write tests that reject wrong dimensions, NaN/Inf, negative step index, non-finite time, and unknown source values; verify frozen records cannot be mutated.
- [ ] Write tests for `TrajectoryMetrics` and `ComparisonReport` serialization to plain dictionaries.
- [ ] Run `python -m pytest tests/test_sim2real_schemas.py -q`; expected initial failure because the module does not exist.
- [ ] Implement frozen dataclasses with tuple normalization and explicit dimension/finite validation. Use `to_dict()` methods that return JSON-compatible primitives.
- [ ] Run the focused test; expected result: all schema tests pass.
- [ ] Commit: `git add src/rebotarm_simulation/rebotarm_simulation/sim2real/schemas.py tests/test_sim2real_schemas.py && git commit -m "feat: add sim2real trajectory schemas"`.

### Task 2: Implement deterministic domain randomization

**Files:** Create `randomization.py`, create `config/sim2real_randomization.yaml`, create `tests/test_sim2real_randomization.py`.

- [ ] Write tests for `RandomizationConfig.from_yaml()` loading scalar ranges, default zero-noise behavior, invalid reversed ranges, non-positive physical scales, and unknown keys.
- [ ] Write tests that `config.sample(seed=7) == config.sample(seed=7)` and that every sampled value lies within its configured range.
- [ ] Write tests that two different seeds differ for at least one non-degenerate range and that all default ranges produce scale `1.0`, latency `0`, and noise `0`.
- [ ] Run `python -m pytest tests/test_sim2real_randomization.py -q`; expected initial failure.
- [ ] Implement `Range` validation, frozen `RandomizationSample`, YAML parsing with `yaml.safe_load`, and `numpy.random.default_rng(seed)` sampling. Keep ranges for mass, damping, friction, torque scale, latency, and three noise standard deviations.
- [ ] Add YAML defaults with all scales fixed at `1.0`, latency `0`, and noise `0`; include bounded non-zero ranges in a separate `training_profile` section for later use, without applying it by default.
- [ ] Run the focused test; expected result: all randomization tests pass.
- [ ] Commit: `git add src/rebotarm_simulation/rebotarm_simulation/sim2real/randomization.py src/rebotarm_simulation/config/sim2real_randomization.yaml tests/test_sim2real_randomization.py && git commit -m "feat: add deterministic sim2real randomization config"`.

### Task 3: Add trajectory recorder and JSONL persistence

**Files:** Create `trajectory_log.py`; create `tests/test_sim2real_trajectory_log.py`.

- [ ] Write tests that `TrajectoryRecorder.append()` accepts increasing time and correct dimensions, rejects duplicate/decreasing time and invalid samples, and reports count/start/end/max velocity/max torque/max contact force.
- [ ] Write a JSONL round-trip test using `tmp_path`: write two samples, load them, and assert exact schema/version/source/values.
- [ ] Run `python -m pytest tests/test_sim2real_trajectory_log.py -q`; expected initial failure.
- [ ] Implement `TrajectoryRecorder(episode_id, source, schema_version=1)`, `append`, `to_jsonl`, `from_jsonl`, and `summary`. Use UTF-8 text, one JSON object per line, and no NumPy-specific serialization in the file.
- [ ] Run the focused test; expected result: all recorder tests pass.
- [ ] Commit: `git add src/rebotarm_simulation/rebotarm_simulation/sim2real/trajectory_log.py tests/test_sim2real_trajectory_log.py && git commit -m "feat: add sim2real trajectory recorder"`.

### Task 4: Connect recorder and randomization to the Reach environment

**Files:** Modify `mujoco_env.py`; modify `tests/test_mujoco_env.py`.

- [ ] Add tests that `RebotArmReachEnv.reset(seed=7, randomization=sample)` stores the sample, default reset remains behavior-compatible, and `record_sample(action)` returns a valid `TrajectorySample` containing observations, control targets, actuator torques, contacts, and source `sim`.
- [ ] Add a test that the environment can create a `TrajectoryRecorder` and record one complete step without requiring ROS 2 or Gymnasium.
- [ ] Run `python -m pytest tests/test_mujoco_env.py -q`; expected initial failure for the new methods.
- [ ] Implement optional `randomization_sample` state, a non-invasive `set_randomization_sample()` hook, and `sample_from_last_step(action, episode_id, step_index)` that converts the current state into a schema record. Do not modify default control or physics when no sample is supplied.
- [ ] Run the focused tests and existing environment tests; expected result: all pass.
- [ ] Commit: `git add src/rebotarm_simulation/rebotarm_simulation/mujoco_env.py tests/test_mujoco_env.py && git commit -m "feat: expose sim2real samples from reach env"`.

### Task 5: Implement replay and comparison

**Files:** Create `replay_compare.py`; create `tests/test_sim2real_replay_compare.py`.

- [ ] Write tests for `compare_trajectories()` using two small records: verify position/velocity/EE/gripper/torque/contact RMSE and max error, shape/time validation, and threshold-based `ok`.
- [ ] Write a fake-environment replay test that applies a recorded action sequence twice with the same seed and produces equal samples.
- [ ] Run `python -m pytest tests/test_sim2real_replay_compare.py -q`; expected initial failure.
- [ ] Implement `compare_trajectories(reference, candidate, thresholds)` using NumPy arrays and an explicit report. Implement `replay_actions(env_factory, actions, seed, recorder)` using `reset(seed)` and `step(action)` without assuming ROS 2 or real hardware.
- [ ] Run focused replay/comparison tests; expected result: all pass.
- [ ] Commit: `git add src/rebotarm_simulation/rebotarm_simulation/sim2real/replay_compare.py tests/test_sim2real_replay_compare.py && git commit -m "feat: add sim2real replay and trajectory comparison"`.

### Task 6: Public exports, documentation, and regression verification

**Files:** Modify `sim2real/__init__.py`; modify `README_mujoco.md`; optionally modify `setup.py`; create or extend package contract tests if needed.

- [ ] Add public imports for schemas, randomization, recorder, replay, and comparison functions without importing ROS 2.
- [ ] Document a complete no-hardware example: load config, sample seed, run Reach, record JSONL, replay actions, compare reports. State clearly that real logs are not collected yet.
- [ ] Run `python -m pytest -q`; expected result: existing suite plus all new tests pass.
- [ ] Run `git diff --check` and `python -m rebotarm_simulation.mujoco_acceptance --skip-renderer` with `PYTHONPATH=src/rebotarm_simulation`; expected `ok=true` and no regression.
- [ ] Sync the touched files to Ubuntu VM and run the focused Sim2Real tests plus the total headless/ROS acceptance command.
- [ ] Commit: `git add src/rebotarm_simulation/rebotarm_simulation/sim2real src/rebotarm_simulation/rebotarm_simulation/mujoco_env.py src/rebotarm_simulation/config/sim2real_randomization.yaml src/rebotarm_simulation/README_mujoco.md tests && git commit -m "feat: add sim2real real2sim foundation"`.

## Self-Review

- Schema, randomization, logging, environment integration, replay/comparison, and documentation each have a dedicated task.
- Defaults preserve current MuJoCo behavior; hardware and ROS 2 are not required by the new modules.
- Every code task has a focused test command and a concrete commit boundary.
- Real hardware calibration, system identification, and policy deployment remain explicitly deferred to a later phase.
