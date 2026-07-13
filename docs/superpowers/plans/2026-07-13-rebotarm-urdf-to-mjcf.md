# reBotArm URDF to MJCF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the abandoned VHACD path and hand-maintained robot structure with a reproducible MuJoCo-official URDF-to-MJCF generator whose checked-in output retains the existing Python and ROS 2 simulation interfaces.

**Architecture:** `src/rebotarm_moveit_config/config/rebotarm.urdf` remains authoritative. A focused generator stages a temporary URDF with local mesh paths, loads it through `mujoco.MjSpec`, normalizes the exported MJCF, and deterministically adds MuJoCo-only actuators, coupling, sites, sensors, contact exclusions, and collision properties. Runtime code loads the checked-in `robot.xml`; `--check` regenerates in memory and detects stale output without modifying the tree.

**Tech Stack:** Python 3.12, MuJoCo 3.10.x (`MjSpec`), `xml.etree.ElementTree`, SHA-256, pytest, MJCF, ROS 2 Jazzy, Git, PowerShell/SSH.

---

## File map

- Create `src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py`: URI staging, official conversion, deterministic MuJoCo augmentation, canonical serialization, generate/check CLI.
- Create `tests/test_urdf_to_mjcf.py`: generator unit and integration contracts.
- Modify `src/rebotarm_simulation/models/rebotarm/robot.xml`: checked-in generated artifact.
- Modify `tests/test_mujoco_model_contract.py`: require original STL mesh collision instead of primitives while retaining dynamics/API contracts.
- Modify `src/rebotarm_simulation/setup.py`: install the generation/check console entry point.
- Modify `src/rebotarm_simulation/README_mujoco.md`: document local generation, checking, source-of-truth, and no-hardware workflow.
- Delete `src/rebotarm_simulation/config/vhacd_collision.json`, `src/rebotarm_simulation/requirements-vhacd.txt`, `src/rebotarm_simulation/tools/generate_vhacd_collision.py`, `src/rebotarm_simulation/models/rebotarm/collision_sources_repaired/`, `src/rebotarm_simulation/models/rebotarm/collision_vhacd/`, `src/rebotarm_simulation/models/rebotarm/collision_vhacd_manifest.json`, `tests/test_vhacd_collision_assets.py`, `docs/superpowers/specs/2026-07-12-rebotarm-vhacd-stl-collision-design.md`, and `docs/superpowers/plans/2026-07-12-rebotarm-vhacd-stl-collision.md`.

### Task 1: Remove the abandoned VHACD surface

**Files:**
- Delete the VHACD files and directories listed in the file map.
- Modify: `tests/test_mujoco_package_contract.py`

- [ ] **Step 1: Add a failing absence contract**

Add to `tests/test_mujoco_package_contract.py`:

```python
def test_abandoned_vhacd_pipeline_is_not_shipped() -> None:
    forbidden = (
        PACKAGE / "config/vhacd_collision.json",
        PACKAGE / "requirements-vhacd.txt",
        PACKAGE / "tools/generate_vhacd_collision.py",
        PACKAGE / "models/rebotarm/collision_sources_repaired",
        PACKAGE / "models/rebotarm/collision_vhacd",
        PACKAGE / "models/rebotarm/collision_vhacd_manifest.json",
    )
    assert not [path for path in forbidden if path.exists()]
```

- [ ] **Step 2: Run the absence contract and verify failure**

Run: `python -m pytest tests/test_mujoco_package_contract.py::test_abandoned_vhacd_pipeline_is_not_shipped -q`

Expected: FAIL and list at least the tracked VHACD config, requirements, generator, and repaired source directory.

- [ ] **Step 3: Discard only failed uncommitted VHACD output, then remove tracked VHACD files**

Before deletion, run `git status --short` and verify every dirty path is one of the agreed VHACD paths. Restore the three modified failed-experiment files to `HEAD`, remove untracked `collision_vhacd/` and its manifest, then use `git rm` for all tracked paths in the file-map deletion list. Do not touch unrelated dirty files.

- [ ] **Step 4: Run the absence contract**

Run: `python -m pytest tests/test_mujoco_package_contract.py::test_abandoned_vhacd_pipeline_is_not_shipped -q`

Expected: `1 passed`.

- [ ] **Step 5: Commit**

```powershell
git add tests/test_mujoco_package_contract.py
git commit -m "refactor: remove abandoned VHACD pipeline"
```

### Task 2: Build the official deterministic URDF converter core

**Files:**
- Create: `src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py`
- Create: `tests/test_urdf_to_mjcf.py`

- [ ] **Step 1: Write failing tests for source discovery, URI staging, and deterministic output**

Create tests that import `authoritative_urdf_path`, `generate_mjcf_bytes`, and `stage_urdf`, then assert:

```python
def test_authoritative_urdf_is_moveit_model() -> None:
    assert authoritative_urdf_path(ROOT) == (
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    )


def test_stage_urdf_rewrites_package_meshes_without_modifying_source(tmp_path) -> None:
    source = authoritative_urdf_path(ROOT)
    before = source.read_bytes()
    staged = stage_urdf(source, ROOT, tmp_path)
    text = staged.read_text(encoding="utf-8")
    assert "package://" not in text
    assert "assets/base_link.STL" in text
    assert source.read_bytes() == before


def test_generation_is_byte_deterministic() -> None:
    assert generate_mjcf_bytes(ROOT) == generate_mjcf_bytes(ROOT)
```

- [ ] **Step 2: Run the new tests and verify import failure**

Run: `python -m pytest tests/test_urdf_to_mjcf.py -q`

Expected: FAIL because `rebotarm_simulation.urdf_to_mjcf` does not exist.

- [ ] **Step 3: Implement the minimal official conversion core**

Implement these public functions with the exact signatures:

```python
def authoritative_urdf_path(repo_root: Path) -> Path: ...
def stage_urdf(source: Path, repo_root: Path, temporary_dir: Path) -> Path: ...
def generate_mjcf_bytes(repo_root: Path) -> bytes: ...
```

`stage_urdf` must parse a copy of the URDF, map each `package://rebotarm_bringup/description/meshes/<name>` to the checked-in `models/rebotarm/assets/<name>`, write an XML declaration, and leave the source untouched. `generate_mjcf_bytes` must create a `TemporaryDirectory`, call `mujoco.MjSpec.from_file(staged_path)`, call `compile()` to fail early on invalid input, and serialize with `to_xml()`. Normalize line endings to LF and ensure exactly one final newline.

- [ ] **Step 4: Run converter tests**

Run: `PYTHONPATH=src/rebotarm_simulation python -m pytest tests/test_urdf_to_mjcf.py -q` (PowerShell equivalent: `$env:PYTHONPATH='src/rebotarm_simulation'; python -m pytest tests/test_urdf_to_mjcf.py -q`).

Expected: PASS for discovery, staging, source immutability, compilation, and byte determinism.

- [ ] **Step 5: Commit**

```powershell
git add src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py tests/test_urdf_to_mjcf.py
git commit -m "feat: add official URDF to MJCF converter"
```

### Task 3: Add deterministic MuJoCo-only augmentation

**Files:**
- Modify: `src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py`
- Modify: `tests/test_urdf_to_mjcf.py`

- [ ] **Step 1: Write failing augmentation contracts**

Parse `generate_mjcf_bytes(ROOT)` and assert all eight joint names exist; every URDF collision mesh becomes a `class="collision"` mesh geom with `contype="1"`, `conaffinity="1"`; visual geoms are non-colliding; adjacent body exclusions match the existing nine pairs; finger equality is `polycoef="0 -1 0 0 0"`; eight position actuators inherit URDF ranges/efforts; joint position/velocity, actuator force, and end-effector pose sensors exist; `ee_site` and `wrist_camera_mount` remain on `end_link`.

- [ ] **Step 2: Run the augmentation contracts and verify failure**

Run: `$env:PYTHONPATH='src/rebotarm_simulation'; python -m pytest tests/test_urdf_to_mjcf.py -q`

Expected: FAIL on missing MuJoCo-only defaults, equality, actuators, sites, exclusions, or sensors.

- [ ] **Step 3: Implement focused augmentation helpers**

Add private helpers with one responsibility each:

```python
def _configure_compiler(root: ET.Element) -> None: ...
def _classify_geoms(root: ET.Element) -> None: ...
def _add_contact_exclusions(root: ET.Element) -> None: ...
def _add_finger_coupling(root: ET.Element) -> None: ...
def _add_sites(root: ET.Element) -> None: ...
def _add_actuators(root: ET.Element, urdf_root: ET.Element) -> None: ...
def _add_sensors(root: ET.Element) -> None: ...
def _canonicalize(root: ET.Element) -> bytes: ...
```

Use the currently validated actuator gains (`480, 480, 400, 240, 200, 160, 250, 250`) and positive `kv` values from the existing model. Derive `ctrlrange` and symmetric `forcerange` from each URDF joint limit. Preserve the current site transforms and exact adjacent-contact exclusion list. Do not encode robot transforms, mass, inertia, joint axes, or mesh filenames by hand; those must remain converter output derived from URDF.

- [ ] **Step 4: Run converter and existing API contract tests**

Run: `$env:PYTHONPATH='src/rebotarm_simulation'; python -m pytest tests/test_urdf_to_mjcf.py tests/test_mujoco_sim_core.py tests/test_mujoco_ros_adapter.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py tests/test_urdf_to_mjcf.py
git commit -m "feat: augment generated reBotArm MJCF"
```

### Task 4: Add generate/check CLI and replace the checked-in model

**Files:**
- Modify: `src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py`
- Modify: `src/rebotarm_simulation/models/rebotarm/robot.xml`
- Modify: `tests/test_urdf_to_mjcf.py`
- Modify: `tests/test_mujoco_model_contract.py`

- [ ] **Step 1: Write failing CLI and stale-output tests**

Test `main(["--repo-root", str(ROOT), "--check"])`, generation to a temporary output, and stale output behavior. `--check` returns `0` only for byte-identical output and returns `1` with a concise regenerate command when the target differs. Generation must write transactionally via a sibling temporary file followed by `Path.replace()`.

- [ ] **Step 2: Update model contracts to require original STL collisions**

Replace `test_robot_model_separates_mesh_visuals_from_primitive_collisions` with a contract that requires the ten original mesh assets, visual geoms with no contacts, collision geoms with mesh references, and no robot collision geom of type `box`, `capsule`, `cylinder`, or `sphere`. Keep all transform, inertia, actuator, FK, scene, and API contracts.

- [ ] **Step 3: Run focused tests and verify failure against the hand-written model**

Run: `$env:PYTHONPATH='src/rebotarm_simulation'; python -m pytest tests/test_urdf_to_mjcf.py tests/test_mujoco_model_contract.py -q`

Expected: FAIL because the checked-in model is stale and still uses primitive collision geoms.

- [ ] **Step 4: Implement CLI and generate `robot.xml`**

Implement:

```python
def check_generated_model(repo_root: Path, output: Path) -> bool: ...
def write_generated_model(repo_root: Path, output: Path) -> None: ...
def main(argv: Sequence[str] | None = None) -> int: ...
```

Default repository root must be discovered from the installed/source module layout when `--repo-root` is omitted; default output is `src/rebotarm_simulation/models/rebotarm/robot.xml`. Run the generator once to replace `robot.xml`.

- [ ] **Step 5: Verify generated artifact and repeatability**

Run the generation command, hash `robot.xml`, run it again, and verify the hash is unchanged. Then run `--check` and expect exit code `0`.

- [ ] **Step 6: Run model contracts**

Run: `$env:PYTHONPATH='src/rebotarm_simulation'; python -m pytest tests/test_urdf_to_mjcf.py tests/test_mujoco_model_contract.py -q`

Expected: all tests pass, including MuJoCo compile, FK, cube/table contact, finger/cube contact, and settled joint control. If original mesh collisions change a pose-specific contact expectation, adjust only the fixture pose based on observed geometry; do not weaken the presence/absence assertion.

- [ ] **Step 7: Commit**

```powershell
git add src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py src/rebotarm_simulation/models/rebotarm/robot.xml tests/test_urdf_to_mjcf.py tests/test_mujoco_model_contract.py
git commit -m "feat: generate checked-in MJCF from URDF"
```

### Task 5: Package and document the workflow

**Files:**
- Modify: `src/rebotarm_simulation/setup.py`
- Modify: `tests/test_mujoco_package_contract.py`
- Modify: `src/rebotarm_simulation/README_mujoco.md`
- Modify: `tests/test_mujoco_documentation.py`

- [ ] **Step 1: Write failing packaging and documentation contracts**

Require the console script:

```text
rebotarm_urdf_to_mjcf = rebotarm_simulation.urdf_to_mjcf:main
```

Require the README to contain the authoritative URDF path, local `robot.xml` path, generation command, `--check` command, the fixed MuJoCo compatibility range, original-STL collision policy, Windows/Ubuntu hash check, and an explicit no-hardware warning.

- [ ] **Step 2: Run focused tests and verify failure**

Run: `python -m pytest tests/test_mujoco_package_contract.py tests/test_mujoco_documentation.py -q`

Expected: FAIL on missing entry point and workflow text.

- [ ] **Step 3: Add entry point and concise operator documentation**

Add the console script in `setup.py`. Document source-tree and installed commands, explain that runtime is local/offline, explain when regeneration is required, and include PowerShell plus Ubuntu shell SHA-256 commands. State that neither conversion nor validation connects to hardware.

- [ ] **Step 4: Run focused tests**

Run: `python -m pytest tests/test_mujoco_package_contract.py tests/test_mujoco_documentation.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add src/rebotarm_simulation/setup.py src/rebotarm_simulation/README_mujoco.md tests/test_mujoco_package_contract.py tests/test_mujoco_documentation.py
git commit -m "docs: add local URDF to MJCF workflow"
```

### Task 6: Full Windows and Ubuntu VM acceptance

**Files:**
- Modify only if a test exposes a scoped defect in files already listed above.

- [ ] **Step 1: Run the complete Windows test suite**

Run: `$env:PYTHONPATH='src/rebotarm_simulation'; python -m pytest tests -q`

Expected: all tests pass; hardware tests remain absent/skipped and no hardware process is started.

- [ ] **Step 2: Run Windows headless smoke tests**

Run the generator `--check`, `rebotarm_mujoco_health`, and a five-second headless CLI simulation. Expected: model load succeeds, renderer/runtime health succeeds where supported, time advances, and all state values remain finite.

- [ ] **Step 3: Inspect repository state before synchronization**

Run `git status --short`, `git diff --check`, and `git log --oneline -8`. Expected: only intentional implementation changes are present and no `collision_vhacd` or manifest assets remain.

- [ ] **Step 4: Synchronize committed source to Ubuntu VM without deletion mirroring**

Use the established repository sync method from `README_mujoco.md`. Do not use a `--delete` mirror, do not copy virtual environments/build/install/log directories, and do not start hardware launch files.

- [ ] **Step 5: Build and test on Ubuntu VM**

Activate `.venv-mujoco-ros`, run `colcon build --symlink-install --packages-select rebotarm_simulation`, source `install/setup.bash`, run the converter `--check`, focused MuJoCo tests, health check, five-second headless simulation, and `ros2 launch rebotarm_simulation mujoco_sim.launch.py` with the existing software-only configuration.

Expected: build succeeds, `--check` passes, model loads, simulation time advances, ROS 2 publishes simulation state, and no hardware endpoint is opened.

- [ ] **Step 6: Compare cross-platform controlled-file hashes**

Compare SHA-256 for the authoritative URDF, all ten STL assets, converter source, and generated `robot.xml`. Expected: Windows and Ubuntu hashes match byte-for-byte.

- [ ] **Step 7: Final verification commit if acceptance required scoped fixes**

If and only if acceptance caused code/document changes, rerun the affected focused tests and full suite, then commit those specific files with `fix: complete URDF to MJCF acceptance`. Otherwise leave the verified commits unchanged.
