from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import io
import json
import math

import pytest

from rebotarm_simulation.mujoco_commands import dispatch_sim_command
from rebotarm_simulation import mujoco_cli
from rebotarm_simulation.mujoco_session import MujocoSession


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"


class FakeSimulation:
    def __init__(self):
        self.time = 0.0
        self.targets = [0.0] * 6
        self.positions = [0.0] * 6
        self.gripper_target = 0.08
        self.gripper = 0.08
        self.mode = "hold"
        self.events = []
        self.closed = False

    def get_state(self):
        return SimpleNamespace(
            simulation_time=self.time,
            joint_positions=tuple(self.positions) + (self.gripper / 2, -self.gripper / 2),
            gripper_width=self.gripper,
        )

    def get_control_status(self):
        return SimpleNamespace(
            joint_targets=tuple(self.targets),
            gripper_target_width_m=self.gripper_target,
        )

    def command_joint_positions(self, values):
        self.targets[:] = values
        self.events.append(("joints", tuple(values)))
        return tuple(values)

    def command_gripper_width(self, value):
        self.gripper_target = value
        self.events.append(("gripper", value))
        return value

    def set_mode(self, mode):
        self.mode = mode
        self.events.append(("mode", mode))
        return mode

    def step(self, count=1):
        self.time += 0.1 * count
        self.positions[:] = self.targets
        self.gripper = self.gripper_target
        return self.get_state()

    def reset_home(self):
        self.time = 0.0
        return self.get_state()

    def close(self):
        self.closed = True


def test_session_records_targets_and_actuals_across_steps():
    sim = FakeSimulation()
    session = MujocoSession(sim)
    session.record_start()
    sim.command_joint_positions([1] * 6)
    session.step(2)
    status = session.record_stop()
    assert status["frame_count"] == 3
    assert session.trajectory.frames[0].simulation_time_s == 0.0
    assert session.trajectory.frames[-1].joint_targets_rad == (1.0,) * 6
    assert session.trajectory.frames[-1].joint_positions_rad == (1.0,) * 6


def test_session_save_clear_load_and_quoted_command_path(tmp_path):
    sim = FakeSimulation()
    session = MujocoSession(sim)
    dispatch_sim_command(sim, "record start", session=session)
    session.step()
    dispatch_sim_command(sim, "record stop", session=session)
    path = tmp_path / "missing parent" / "trajectory with spaces.json"
    dispatch_sim_command(sim, f'trajectory save "{path}"', session=session)
    dispatch_sim_command(sim, "record clear", session=session)
    assert session.state()["frame_count"] == 0
    dispatch_sim_command(sim, f'trajectory load "{path}"', session=session)
    assert session.state()["trajectory_loaded"]
    assert session.state()["frame_count"] == 2


def test_session_replay_interpolates_then_enters_hold():
    sim = FakeSimulation()
    session = MujocoSession(sim)
    session.record_start()
    sim.command_joint_positions([0.5] * 6)
    sim.command_gripper_width(0.05)
    session.step()
    sim.command_joint_positions([1] * 6)
    sim.command_gripper_width(0.02)
    session.step()
    session.record_stop()
    sim.time = 5.0
    sim.targets[:] = [0.0] * 6
    session.replay_start()
    session.step()
    assert sim.targets == pytest.approx([0.5] * 6)
    assert sim.gripper_target == pytest.approx(0.05)
    session.step()
    assert session.state()["replay_state"] == "finished"
    assert session.state()["replay_progress"] == 1.0
    assert sim.mode == "hold"


def test_replay_pause_resume_and_manual_stop_hold():
    sim = FakeSimulation()
    session = MujocoSession(sim)
    session.record_start()
    sim.command_joint_positions([1] * 6)
    session.step(3)
    session.record_stop()
    session.replay_start()
    session.step()
    session.replay_pause()
    paused_progress = session.state()["replay_progress"]
    sim.step(10)
    session.update()
    assert session.state()["replay_progress"] == paused_progress
    session.replay_resume()
    session.step()
    assert session.state()["replay_progress"] > paused_progress
    session.replay_stop()
    assert sim.mode == "hold"
    assert session.state()["replay_state"] == "stopped"


def test_session_rejects_conflicting_lifecycles_and_commands_require_session():
    sim = FakeSimulation()
    session = MujocoSession(sim)
    with pytest.raises(RuntimeError, match="no trajectory"):
        session.replay_start()
    session.record_start()
    with pytest.raises(RuntimeError, match="recording"):
        session.replay_start()
    with pytest.raises(RuntimeError, match="session"):
        dispatch_sim_command(sim, "trajectory state")
    with pytest.raises(ValueError, match="usage"):
        dispatch_sim_command(sim, "record maybe", session=session)


def test_trajectory_state_command_is_read_only():
    sim = FakeSimulation()
    session = MujocoSession(sim)
    result = dispatch_sim_command(sim, "trajectory state", session=session)
    assert not result.mutated
    assert result.value["replay_state"] == "idle"


def test_cli_shell_exposes_record_save_load_and_replay_commands(tmp_path):
    sim = FakeSimulation()
    path = tmp_path / "shell.json"
    stdin = io.StringIO(
        f'record start\nstep 2\nrecord stop\ntrajectory save "{path}"\n'
        f'record clear\ntrajectory load "{path}"\nreplay start\nstep 2\ntrajectory state\nquit\n'
    )
    stdout = io.StringIO()
    assert mujoco_cli.main(
        ["shell"], sim_factory=lambda _: sim, stdin=stdin, stdout=stdout
    ) == 0
    payloads = [json.loads(line) for line in stdout.getvalue().splitlines()]
    assert any(payload.get("trajectory_loaded") for payload in payloads if isinstance(payload, dict))
    assert any(payload.get("replay_state") == "finished" for payload in payloads if isinstance(payload, dict))
    assert sim.mode == "hold"
    assert sim.closed


def test_cli_reports_invalid_session_transition_and_continues():
    stdout = io.StringIO()
    assert mujoco_cli.main(
        ["shell"],
        sim_factory=lambda _: FakeSimulation(),
        stdin=io.StringIO("replay start\ntrajectory state\nquit\n"),
        stdout=stdout,
    ) == 0
    assert "error: no trajectory is available" in stdout.getvalue()
    assert '"replay_state": "idle"' in stdout.getvalue()


def test_completed_replay_produces_and_saves_error_comparison(tmp_path):
    sim = FakeSimulation()
    session = MujocoSession(sim)
    session.record_start()
    session.step()
    session.record_stop()
    sim.time = 2.0
    session.replay_start()
    session.step()

    report = dispatch_sim_command(sim, "trajectory compare", session=session).value
    assert report["completed"] is True
    assert report["passed"] is True
    assert report["sample_count"] == 2
    assert report["overall_tracking_rmse_rad"] == pytest.approx(0.0)
    assert report["overall_repeatability_rmse_rad"] == pytest.approx(0.0)

    path = tmp_path / "replay report.json"
    result = dispatch_sim_command(
        sim, f'trajectory report "{path}"', session=session
    ).value
    assert result["path"] == str(path)
    assert json.loads(path.read_text(encoding="utf-8")) == report


def test_comparison_is_unavailable_before_replay():
    session = MujocoSession(FakeSimulation())
    with pytest.raises(RuntimeError, match="no replay comparison"):
        session.comparison()


def test_real_mujoco_record_replay_generates_finite_repeatability_report():
    pytest.importorskip("mujoco")
    from rebotarm_simulation.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(SCENE) as sim:
        sim.reset_home()
        session = MujocoSession(sim)
        session.record_start()
        home = tuple(sim.control_targets[:6])
        for step_index in range(1, 101):
            target = list(home)
            target[0] += 0.05 * step_index / 100.0
            sim.command_joint_positions(target)
            session.step()
        session.record_stop()

        sim.reset_home()
        session.replay_start()
        while session.playback.state == "playing":
            session.step()
        report = session.comparison()

    assert report["completed"] is True
    assert report["sample_count"] >= 100
    assert math.isfinite(report["overall_tracking_rmse_rad"])
    assert math.isfinite(report["overall_repeatability_rmse_rad"])
    assert report["overall_tracking_max_abs_rad"] < 0.08
