# reBotArm VHACD STL Collision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace reBotArm's primitive MuJoCo collision geoms with reproducible VHACD-decomposed STL hulls while preserving dynamics, ROS 2 interfaces, and no-hardware safety.

**Architecture:** Original STL files remain non-colliding visual assets. A pinned Python generator uses `vhacdx` synchronously, normalizes and deterministically sorts hulls, writes versioned collision STL files plus a SHA-256 manifest, and supports a read-only `--check` mode. MJCF references every generated hull as a collision-only mesh geom; tests validate asset provenance, geometry, contact behavior, performance, integration, and Windows/Ubuntu equality.

**Tech Stack:** Python 3.12, NumPy, trimesh 4.12.2, vhacdx 0.0.10, MuJoCo 3.10.0, pytest, MJCF, ROS 2 Jazzy, PowerShell/SSH for explicit VM sync.

---

## File map

- Create `src/rebotarm_simulation/tools/generate_vhacd_collision.py`: deterministic generation and `--check` entry point.
- Create `src/rebotarm_simulation/config/vhacd_collision.json`: per-part source names and fixed decomposition budgets.
- Create `src/rebotarm_simulation/requirements-vhacd.txt`: generation-only pinned dependencies.
- Create `src/rebotarm_simulation/models/rebotarm/collision_vhacd_manifest.json`: generated provenance and hash manifest.
- Create `src/rebotarm_simulation/models/rebotarm/collision_vhacd/<part>/hull_NNN.stl`: generated convex assets.
- Modify `src/rebotarm_simulation/models/rebotarm/robot.xml`: visual-only original meshes and collision-only hull meshes/geoms.
- Modify `src/rebotarm_simulation/setup.py`: install manifest and nested collision assets.
- Modify `src/rebotarm_simulation/README_mujoco.md`: generation, verification, precision, and benchmark instructions.
- Modify `tests/test_mujoco_model_contract.py`: STL collision contract and physical contact regressions.
- Create `tests/test_vhacd_collision_assets.py`: generator/config/manifest determinism and validation.
- Create `tests/test_mujoco_collision_performance.py`: fixed 10,000-step headless benchmark.

### Task 1: Lock the generator contract and dependencies

**Files:**
- Create: `src/rebotarm_simulation/config/vhacd_collision.json`
- Create: `src/rebotarm_simulation/requirements-vhacd.txt`
- Create: `tests/test_vhacd_collision_assets.py`

- [ ] **Step 1: Write the failing configuration contract test**

```python
def test_vhacd_config_covers_all_visual_meshes_with_precision_budgets():
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    assert config["schema_version"] == 1
    assert config["generator"] == {"package": "vhacdx", "version": "0.0.10"}
    assert set(config["parts"]) == {
        "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
        "gripper_base", "left_finger", "right_finger",
    }
    for name, part in config["parts"].items():
        assert part["source"].lower().endswith(".stl")
        assert 4 <= part["max_convex_hulls"] <= (20 if "finger" in name else 16)
```

- [ ] **Step 2: Run the test and verify RED**

Run: `python -m pytest tests/test_vhacd_collision_assets.py::test_vhacd_config_covers_all_visual_meshes_with_precision_budgets -q`  
Expected: FAIL because `vhacd_collision.json` does not exist.

- [ ] **Step 3: Add pinned generation dependencies**

```text
numpy==2.4.2
trimesh==4.12.2
vhacdx==0.0.10
```

- [ ] **Step 4: Add the fixed configuration**

Use common parameters `resolution=400000`, `minimum_volume_percent_error_allowed=1.0`, `max_recursion_depth=10`, `shrink_wrap=true`, `fill_mode="flood"`, `max_num_vertices_per_hull=64`, `async_acd=false`, `min_edge_length=2`, and `find_best_plane=false`. Set hull budgets to 8 for base/link parts, 12 for `gripper_base`, and 20 for each finger.

- [ ] **Step 5: Run the contract test and verify GREEN**

Run: `python -m pytest tests/test_vhacd_collision_assets.py -q`  
Expected: PASS for the configuration tests; generator-dependent tests are not added yet.

- [ ] **Step 6: Commit**

```bash
git add src/rebotarm_simulation/config/vhacd_collision.json \
  src/rebotarm_simulation/requirements-vhacd.txt tests/test_vhacd_collision_assets.py
git commit -m "build: define VHACD collision generation contract"
```

### Task 2: Implement deterministic generation and checking

**Files:**
- Create: `src/rebotarm_simulation/tools/generate_vhacd_collision.py`
- Modify: `tests/test_vhacd_collision_assets.py`

- [ ] **Step 1: Write failing unit tests for canonical hull ordering and manifest checks**

```python
def test_hull_sort_key_is_geometry_based_and_stable(generator_module):
    small = trimesh.creation.box((0.01, 0.01, 0.01))
    large = trimesh.creation.box((0.02, 0.02, 0.02))
    ordered = sorted([small, large], key=generator_module.hull_sort_key)
    assert [round(mesh.volume, 9) for mesh in ordered] == [0.000008, 0.000001]


def test_check_manifest_rejects_changed_output_hash(tmp_path, generator_module):
    manifest = {"outputs": [{"path": "hull_000.stl", "sha256": "0" * 64}]}
    (tmp_path / "hull_000.stl").write_bytes(b"changed")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        generator_module.check_outputs(tmp_path, manifest)
```

- [ ] **Step 2: Run the new tests and verify RED**

Run: `python -m pytest tests/test_vhacd_collision_assets.py -q`  
Expected: FAIL because the generator module and functions do not exist.

- [ ] **Step 3: Implement the generator's stable primitives**

Implement these public helpers with the following core behavior:

```python
def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hull_sort_key(mesh: trimesh.Trimesh) -> tuple[float, ...]:
    values = (-float(mesh.volume), *mesh.centroid.tolist(), *mesh.bounds.reshape(-1).tolist())
    return tuple(round(value, 12) for value in values)


def load_config(path: Path) -> dict[str, object]:
    config = json.loads(path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("unsupported VHACD config schema")
    return config


def decompose_part(source: Path, settings: dict[str, object]) -> list[trimesh.Trimesh]:
    source_mesh = trimesh.load_mesh(source, process=False)
    vertices = np.asarray(source_mesh.vertices, dtype=np.float64)
    triangles = np.asarray(source_mesh.faces, dtype=np.uint32)
    vhacd_faces = np.column_stack(
        (np.full(len(triangles), 3, dtype=np.uint32), triangles)
    ).reshape(-1)
    raw_hulls = vhacdx.compute_vhacd(
        vertices,
        vhacd_faces,
        maxConvexHulls=settings["max_convex_hulls"],
        resolution=settings["resolution"],
        minimumVolumePercentErrorAllowed=settings["minimum_volume_percent_error_allowed"],
        maxRecursionDepth=settings["max_recursion_depth"],
        shrinkWrap=settings["shrink_wrap"],
        fillMode=settings["fill_mode"],
        maxNumVerticesPerCH=settings["max_num_vertices_per_hull"],
        asyncACD=False,
        minEdgeLength=settings["min_edge_length"],
        findBestPlane=settings["find_best_plane"],
    )
    hulls = [trimesh.Trimesh(vertices=v, faces=f, process=True) for v, f in raw_hulls]
    for hull in hulls:
        hull.remove_unreferenced_vertices()
        if not hull.is_convex or not np.isfinite(hull.vertices).all():
            raise ValueError(f"invalid VHACD hull generated from {source}")
    return sorted(hulls, key=hull_sort_key)
```

Add `build_manifest(model_dir, config)`, `check_outputs(model_dir, manifest)`, and `main(argv=None)` around these helpers. The manifest builder must export `hull_NNN.stl` in sorted order and record all hashes and mesh metadata. `main --check` must perform no writes and return nonzero on any config, input, output, count, or hash mismatch.

- [ ] **Step 4: Run generator unit tests and verify GREEN**

Run: `python -m pytest tests/test_vhacd_collision_assets.py -q`  
Expected: all unit tests PASS without generating repository assets.

- [ ] **Step 5: Commit**

```bash
git add src/rebotarm_simulation/tools/generate_vhacd_collision.py \
  tests/test_vhacd_collision_assets.py
git commit -m "feat: add deterministic VHACD collision generator"
```

### Task 3: Capture the primitive-collision performance baseline

**Files:**
- Create: `tests/test_mujoco_collision_performance.py`
- Create: `src/rebotarm_simulation/config/mujoco_collision_baseline.json`

- [ ] **Step 1: Add a benchmark function with a machine-readable result**

```python
def benchmark_scene(scene: Path, steps: int = 10_000) -> dict[str, float | int]:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    start = time.perf_counter()
    peak_contacts = 0
    for _ in range(steps):
        mujoco.mj_step(model, data)
        peak_contacts = max(peak_contacts, int(data.ncon))
    elapsed = time.perf_counter() - start
    return {"steps": steps, "elapsed_seconds": elapsed,
            "realtime_factor": steps * model.opt.timestep / elapsed,
            "peak_contacts": peak_contacts}
```

- [ ] **Step 2: Run it on Ubuntu before changing MJCF**

Run with `.venv-mujoco-ros/bin/python` and write the observed environment, MuJoCo version, elapsed seconds, and realtime factor into `mujoco_collision_baseline.json`. Run three times and store the median elapsed time.

- [ ] **Step 3: Add a schema test for the captured baseline**

Assert `steps == 10000`, `samples == 3`, `median_elapsed_seconds > 0`, `mujoco_version == "3.10.0"`, and `model_kind == "primitive_collision"`.

- [ ] **Step 4: Commit**

```bash
git add tests/test_mujoco_collision_performance.py \
  src/rebotarm_simulation/config/mujoco_collision_baseline.json
git commit -m "test: record primitive MuJoCo collision baseline"
```

### Task 4: Generate and validate all collision hull assets

**Files:**
- Create: `src/rebotarm_simulation/models/rebotarm/collision_vhacd_manifest.json`
- Create: `src/rebotarm_simulation/models/rebotarm/collision_vhacd/**/*.stl`
- Modify: `tests/test_vhacd_collision_assets.py`

- [ ] **Step 1: Write failing repository-asset tests**

Tests must assert that the manifest covers all 10 inputs, every input hash matches `assets/`, every output path is below `collision_vhacd/`, every output hash matches, names are consecutively numbered from `hull_000.stl`, hull counts do not exceed configured budgets, and each output loads as a finite convex `trimesh.Trimesh`.

- [ ] **Step 2: Verify RED**

Run: `python -m pytest tests/test_vhacd_collision_assets.py -q`  
Expected: FAIL because the manifest and collision assets do not exist.

- [ ] **Step 3: Install generation-only dependencies and generate**

```powershell
python -m pip install -r src/rebotarm_simulation/requirements-vhacd.txt
python src/rebotarm_simulation/tools/generate_vhacd_collision.py
python src/rebotarm_simulation/tools/generate_vhacd_collision.py --check
```

Expected: 10 parts generated, zero check mismatches, no primitive fallback.

- [ ] **Step 4: Inspect hull budgets and finger openings**

Render or load the decomposed hulls over the visual meshes. Confirm both finger inner faces and the open gap remain represented; if not, adjust only that part's budget/resolution in the config, regenerate all outputs, and rerun `--check`.

- [ ] **Step 5: Run repository-asset tests and verify GREEN**

Run: `python -m pytest tests/test_vhacd_collision_assets.py -q`  
Expected: all tests PASS.

- [ ] **Step 6: Commit generated assets and manifest**

```bash
git add src/rebotarm_simulation/models/rebotarm/collision_vhacd \
  src/rebotarm_simulation/models/rebotarm/collision_vhacd_manifest.json \
  src/rebotarm_simulation/config/vhacd_collision.json tests/test_vhacd_collision_assets.py
git commit -m "feat: add reBotArm VHACD collision assets"
```

### Task 5: Replace primitive MJCF collisions with hull meshes

**Files:**
- Modify: `src/rebotarm_simulation/models/rebotarm/robot.xml`
- Modify: `tests/test_mujoco_model_contract.py`

- [ ] **Step 1: Replace the old primitive-collision test with a failing VHACD contract**

```python
def test_robot_visuals_are_noncolliding_and_collisions_use_vhacd_meshes():
    robot = _robot_root()
    visuals = robot.findall('.//geom[@class="visual"]')
    collisions = robot.findall('.//geom[@class="collision"]')
    assert visuals and collisions
    assert all(g.attrib.get("contype") == "0" and g.attrib.get("conaffinity") == "0"
               for g in visuals)
    assert all(g.attrib.get("type") == "mesh" and "mesh" in g.attrib for g in collisions)
    assert all(g.attrib["mesh"].startswith("collision_") for g in collisions)
    assert not any(g.attrib.get("type") in {"box", "cylinder", "capsule", "sphere"}
                   for g in collisions)
```

- [ ] **Step 2: Verify RED against the primitive MJCF**

Run the single test; expected FAIL because collisions are primitives.

- [ ] **Step 3: Add every hull to `<asset>` and every corresponding body**

Use names `collision_<part>_<NNN>` and files `../collision_vhacd/<part>/hull_NNN.stl`. Keep original mesh assets for visuals. Set visual defaults to `contype="0" conaffinity="0" group="2"`; collision defaults to `type="mesh" contype="1" conaffinity="1" group="3" rgba="0 0 0 0"`. Remove every prior robot primitive collision geom.

- [ ] **Step 4: Verify model contract and compilation**

Run: `python -m pytest tests/test_mujoco_model_contract.py -q`  
Expected: all model, inertia, joint, zero-pose, folded-collision, cube, finger, and table tests PASS. If a previous known collision pose no longer contacts, locate a new deterministic legal pose and document why the convex geometry changed; do not weaken the assertion to “some contact”.

- [ ] **Step 5: Commit**

```bash
git add src/rebotarm_simulation/models/rebotarm/robot.xml \
  tests/test_mujoco_model_contract.py
git commit -m "feat: use VHACD meshes for MuJoCo collisions"
```

### Task 6: Package and document the collision assets

**Files:**
- Modify: `src/rebotarm_simulation/setup.py`
- Modify: `src/rebotarm_simulation/README_mujoco.md`
- Modify: `tests/test_mujoco_package_contract.py`
- Modify: `tests/test_mujoco_documentation.py`

- [ ] **Step 1: Add failing install/resource and documentation tests**

Assert `setup.py` installs the manifest and recursively grouped hull files, and README contains `VHACD`, `requirements-vhacd.txt`, generator command, `--check`, convex-hull limitation, performance command, and the statement that no primitive fallback exists.

- [ ] **Step 2: Verify RED**

Run the two focused test files; expected FAIL for missing package data/docs.

- [ ] **Step 3: Implement package-data grouping and documentation**

Add a helper that produces `(destination, files)` entries per `collision_vhacd/<part>` directory instead of flattening STL names. Document generation dependencies as development-only; normal runtime still installs only `requirements-mujoco.txt`.

- [ ] **Step 4: Verify GREEN**

Run: `python -m pytest tests/test_mujoco_package_contract.py tests/test_mujoco_documentation.py -q`  
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/rebotarm_simulation/setup.py src/rebotarm_simulation/README_mujoco.md \
  tests/test_mujoco_package_contract.py tests/test_mujoco_documentation.py
git commit -m "docs: package and explain VHACD collision assets"
```

### Task 7: Enforce stability and performance gates

**Files:**
- Modify: `tests/test_mujoco_collision_performance.py`
- Modify: `tests/test_mujoco_model_contract.py`

- [ ] **Step 1: Add the post-migration performance assertion**

Run three 10,000-step samples, take the median, assert all state arrays are finite, and assert `vhacd_median_elapsed <= primitive_median_elapsed * 5.0`. Record hull count and peak contacts in failure output.

- [ ] **Step 2: Run and diagnose the gate**

Expected: PASS. If over 5x, use the manifest to identify high-count parts and reduce only hulls that do not define gripper inner surfaces or fingertips. Regenerate, rerun asset checks, then rerun all contact tests.

- [ ] **Step 3: Run all Windows focused tests**

```powershell
python -m pytest tests/test_mujoco_*.py tests/test_package_layering.py -q
```

Expected: zero failures and no MuJoCo runtime skips on the configured Windows environment.

- [ ] **Step 4: Commit any final validated budget adjustment**

Stage only config, regenerated collision assets, manifest, and the performance/contact tests; commit as `perf: bound VHACD collision overhead`.

### Task 8: Explicitly sync and verify Ubuntu VM

**Files:** all changed files from this plan only.

- [ ] **Step 1: Build an explicit changed-file list**

Use `git diff --name-only <pre-vhacd-commit>..HEAD`. Exclude `.venv`, `build`, `install`, `log`, caches, and all unrelated dirty visual-grasp files.

- [ ] **Step 2: Back up every destination file on the VM**

Create `/home/u24/rebotarm_sync_backups/<timestamp>-vhacd` and preserve relative paths. Do not use mirror or delete synchronization.

- [ ] **Step 3: Copy the explicit file list and verify SHA-256**

Compare every Windows source/manifest/STL hash with `/home/u24/robotarm_ros2/<path>`. Expected: zero missing files and zero mismatches.

- [ ] **Step 4: Install generation dependencies only for check mode**

Install `requirements-vhacd.txt` into a separate `.venv-vhacd-tools` or the approved development environment, not the ROS runtime venv. Run generator `--check`; do not regenerate VM assets.

- [ ] **Step 5: Run Ubuntu tests and build**

```bash
MUJOCO_GL=egl .venv-mujoco-ros/bin/python -m pytest \
  tests/test_mujoco_*.py tests/test_package_layering.py -q
.venv-mujoco-ros/bin/python -m colcon build --symlink-install \
  --packages-select rebotarm_simulation rebotarm_moveit_config rebotarm_bringup
```

Expected: zero failures; three packages finish successfully.

### Task 9: Run no-hardware runtime acceptance

**Files:** no repository modifications unless a failing acceptance exposes a defect, in which case return to TDD before fixing.

- [ ] **Step 1: Verify no hardware process exists**

Search exact processes for `rebotarmcontroller`, hardware nodes, CANopen, and SocketCAN. If found, stop acceptance and report; do not terminate unrelated processes without confirmation.

- [ ] **Step 2: Run health, headless, and Viewer**

Run EGL health with a real 64×64 render, headless CLI for 5 simulated seconds, and Viewer with the current desktop session's `DISPLAY` and `XAUTHORITY`. Expected: model loads, physics is finite, renderer is available, Viewer exits normally.

- [ ] **Step 3: Run ROS 2 adapter acceptance**

Launch only `rebotarm_mujoco_node`/`mujoco_sim.launch.py`. Send a six-joint trajectory, set gripper width, stop a long active trajectory, read `/joint_states`, `/gripper/state`, and monotonic `/clock`. Expected: success, reached width, ABORTED stop result, and actual simulated states.

- [ ] **Step 4: Run MoveIt to MuJoCo acceptance**

Launch with `use_hardware:=false`, `use_moveit_fake_joint_states:=false`, `start_passive_joint_state_publisher:=false`, and `use_sim_time:=true`. Send a collision-free MoveGroup target. Expected: MoveIt SUCCESS and changed MuJoCo joint state with no hardware node.

- [ ] **Step 5: Confirm cleanup and final hashes**

Terminate only confirmed test PIDs, verify no MuJoCo or hardware process remains, and repeat all changed-file hashes.

### Task 10: Final review, commit, and push

- [ ] **Step 1: Run requirement-by-requirement audit against the design's nine completion criteria**

Record authoritative evidence for asset count, default MJCF collision type, manifest check, dynamics invariants, contact tests, Ubuntu runtime/build/ROS/MoveIt, 5x performance gate, hashes, and no-hardware/no-training boundary.

- [ ] **Step 2: Run an independent code review**

Review generator safety, path containment, deterministic output, manifest validation, MJCF mapping, test false positives, and unrelated worktree preservation. Fix findings with a failing test first.

- [ ] **Step 3: Commit remaining plan-scoped changes only**

Do not stage existing visual-grasp changes or deleted user documents. Verify staged names before committing.

- [ ] **Step 4: Push the current branch**

```bash
git push origin codex/graspnet-only-migration
git rev-list --left-right --count origin/codex/graspnet-only-migration...HEAD
```

Expected: push succeeds and comparison is `0 0`.
