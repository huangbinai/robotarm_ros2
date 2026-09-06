"""Simulation-thread coordinator for MuJoCo recording and target playback."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .mujoco_trajectory import (
    MujocoTrajectory,
    MujocoTrajectoryPlayback,
    MujocoTrajectoryRecorder,
    PlaybackErrorAccumulator,
    PlaybackErrorThresholds,
    PlaybackOutput,
)


class MujocoSession:
    """Own trajectory lifecycle without exposing mutable MuJoCo internals."""

    def __init__(
        self,
        simulation: Any,
        *,
        error_thresholds: PlaybackErrorThresholds | None = None,
    ) -> None:
        self.simulation = simulation
        self.recorder = MujocoTrajectoryRecorder()
        self.trajectory: MujocoTrajectory | None = None
        self.playback: MujocoTrajectoryPlayback | None = None
        self.errors = PlaybackErrorAccumulator(error_thresholds)

    def record_start(self) -> dict[str, Any]:
        if self.playback is not None and self.playback.state in ("playing", "paused"):
            raise RuntimeError("cannot record while replay is active")
        self.recorder.start(clear=True)
        self._capture()
        return self.state()

    def record_stop(self) -> dict[str, Any]:
        self.trajectory = self.recorder.stop()
        return self.state()

    def record_clear(self) -> dict[str, Any]:
        if self.playback is not None and self.playback.state in ("playing", "paused"):
            raise RuntimeError("cannot clear trajectory while replay is active")
        self.recorder.clear()
        self.trajectory = None
        self.playback = None
        self.errors.clear()
        return self.state()

    def save(self, path: str | Path) -> dict[str, Any]:
        trajectory = self._available_trajectory()
        trajectory.save_json(path)
        result = self.state()
        result["path"] = str(Path(path))
        return result

    def load(self, path: str | Path) -> dict[str, Any]:
        if self.recorder.is_recording:
            raise RuntimeError("stop recording before loading a trajectory")
        if self.playback is not None and self.playback.state in ("playing", "paused"):
            raise RuntimeError("stop replay before loading a trajectory")
        self.trajectory = MujocoTrajectory.load_json(path)
        self.playback = None
        self.errors.clear()
        result = self.state()
        result["path"] = str(Path(path))
        return result

    def replay_start(self) -> dict[str, Any]:
        if self.recorder.is_recording:
            raise RuntimeError("stop recording before replay")
        self.playback = MujocoTrajectoryPlayback(self._available_trajectory())
        self.errors.clear()
        output = self.playback.start(self._simulation_time())
        self._apply(output)
        self._measure(output)
        return self.state()

    def replay_pause(self) -> dict[str, Any]:
        self._require_playback().pause(self._simulation_time())
        return self.state()

    def replay_resume(self) -> dict[str, Any]:
        self._require_playback().resume(self._simulation_time())
        return self.state()

    def replay_stop(self) -> dict[str, Any]:
        output = self._require_playback().stop()
        self._measure(output)
        self._apply(output)
        return self.state()

    def comparison(self) -> dict[str, Any]:
        report = self._comparison_report()
        if report is None:
            raise RuntimeError("no replay comparison is available")
        return report

    def save_comparison(self, path: str | Path) -> dict[str, Any]:
        report = self.comparison()
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(
                report,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n",
            encoding="utf-8",
            newline="\n",
        )
        return {"path": str(destination), "comparison": report}

    def step(self, count: int = 1):
        """Advance physics while servicing playback and recorder each step."""
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("step count must be a positive integer")
        replay_active = self.playback is not None and self.playback.state == "playing"
        if not self.recorder.is_recording and not replay_active:
            return self.simulation.step() if count == 1 else self.simulation.step(count)
        state = None
        for _ in range(count):
            state = self.simulation.step()
            self.update()
        return state

    def update(self) -> PlaybackOutput | None:
        """Service one owner-thread tick after simulation time advances."""
        output = None
        if self.playback is not None and self.playback.state == "playing":
            output = self.playback.update(self._simulation_time())
            self._measure(output)
            self._apply(output)
        if self.recorder.is_recording:
            self._capture()
        return output

    def state(self) -> dict[str, Any]:
        trajectory = self.trajectory
        if trajectory is None and self.recorder.frames:
            frame_count = len(self.recorder.frames)
            duration = self.recorder.frames[-1].simulation_time_s - self.recorder.frames[0].simulation_time_s
        elif trajectory is not None:
            frame_count = len(trajectory.frames)
            duration = trajectory.duration_s
        else:
            frame_count = 0
            duration = 0.0
        return {
            "recording": self.recorder.is_recording,
            "frame_count": frame_count,
            "duration_s": duration,
            "trajectory_loaded": trajectory is not None,
            "replay_state": "idle" if self.playback is None else self.playback.state,
            "replay_progress": 0.0 if self.playback is None else self.playback.progress,
            "comparison": self._comparison_report(),
        }

    def _comparison_report(self) -> dict[str, Any] | None:
        playback_state = "idle" if self.playback is None else self.playback.state
        report = self.errors.report(completed=playback_state == "finished")
        return None if report.sample_count == 0 else report.to_dict()

    def _available_trajectory(self) -> MujocoTrajectory:
        if self.trajectory is not None:
            return self.trajectory
        if self.recorder.is_recording:
            raise RuntimeError("stop recording before using the trajectory")
        if self.recorder.frames:
            return self.recorder.trajectory()
        raise RuntimeError("no trajectory is available")

    def _require_playback(self) -> MujocoTrajectoryPlayback:
        if self.playback is None:
            raise RuntimeError("replay has not been started")
        return self.playback

    def _simulation_time(self) -> float:
        return float(self.simulation.get_state().simulation_time)

    def _capture(self) -> None:
        state = self.simulation.get_state()
        status = self.simulation.get_control_status()
        time = float(state.simulation_time)
        if self.recorder.frames and time == self.recorder.frames[-1].simulation_time_s:
            return
        self.recorder.record(
            time,
            status.joint_targets,
            state.joint_positions[:6],
            status.gripper_target_width_m,
            state.gripper_width,
        )

    def _apply(self, output: PlaybackOutput) -> None:
        self.simulation.command_joint_positions(output.joint_targets_rad)
        self.simulation.command_gripper_width(output.gripper_target_width_m)
        if output.hold_requested:
            self.simulation.set_mode("hold")

    def _measure(self, output: PlaybackOutput) -> None:
        state = self.simulation.get_state()
        self.errors.append(output, state.joint_positions[:6], state.gripper_width)
