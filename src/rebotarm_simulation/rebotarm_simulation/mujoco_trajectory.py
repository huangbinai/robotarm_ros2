"""ROS-independent MuJoCo target recording and deterministic playback.

This module records commands and observations, but playback intentionally emits
only commands.  The simulation owner remains responsible for applying a final
Hold transition when ``PlaybackOutput.hold_requested`` is true.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Sequence

from .trajectory_sampler import ARM_JOINT_NAMES


SCHEMA = "rebotarm.mujoco.trajectory"
SCHEMA_VERSION = 1
UNITS = {
    "time": "s",
    "joint_position": "rad",
    "gripper_width": "m",
}


def _finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _six(values: Sequence[float], label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{label} must contain six finite values")
    try:
        result = tuple(_finite(value, label) for value in values)
    except TypeError as exc:
        raise ValueError(f"{label} must contain six finite values") from exc
    if len(result) != 6:
        raise ValueError(f"{label} must contain six finite values")
    return result


def _joint_names(values: Sequence[str]) -> tuple[str, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError("joint_names must match the canonical six arm joints")
    names = tuple(values)
    if names != ARM_JOINT_NAMES:
        raise ValueError("joint_names must match the canonical six arm joints")
    return names


@dataclass(frozen=True)
class TrajectoryFrame:
    simulation_time_s: float
    joint_targets_rad: tuple[float, ...]
    joint_positions_rad: tuple[float, ...]
    gripper_target_width_m: float
    gripper_width_m: float

    def __post_init__(self) -> None:
        time = _finite(self.simulation_time_s, "simulation_time_s")
        if time < 0.0:
            raise ValueError("simulation_time_s must be non-negative")
        object.__setattr__(self, "simulation_time_s", time)
        object.__setattr__(
            self, "joint_targets_rad", _six(self.joint_targets_rad, "joint_targets_rad")
        )
        object.__setattr__(
            self, "joint_positions_rad", _six(self.joint_positions_rad, "joint_positions_rad")
        )
        for name in ("gripper_target_width_m", "gripper_width_m"):
            value = _finite(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")
            object.__setattr__(self, name, value)

    def to_dict(self) -> dict[str, Any]:
        return {
            "simulation_time_s": self.simulation_time_s,
            "joint_targets_rad": list(self.joint_targets_rad),
            "joint_positions_rad": list(self.joint_positions_rad),
            "gripper_target_width_m": self.gripper_target_width_m,
            "gripper_width_m": self.gripper_width_m,
        }


@dataclass(frozen=True)
class MujocoTrajectory:
    joint_names: tuple[str, ...]
    frames: tuple[TrajectoryFrame, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "joint_names", _joint_names(self.joint_names))
        frames = tuple(self.frames)
        if not frames:
            raise ValueError("trajectory must contain at least one frame")
        if any(not isinstance(frame, TrajectoryFrame) for frame in frames):
            raise TypeError("frames must be TrajectoryFrame records")
        for previous, current in zip(frames, frames[1:]):
            if current.simulation_time_s <= previous.simulation_time_s:
                raise ValueError("simulation times must be strictly increasing")
        object.__setattr__(self, "frames", frames)

    @property
    def duration_s(self) -> float:
        return self.frames[-1].simulation_time_s - self.frames[0].simulation_time_s

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "version": SCHEMA_VERSION,
            "units": dict(UNITS),
            "joint_names": list(self.joint_names),
            "frames": [frame.to_dict() for frame in self.frames],
        }

    def save_json(self, path: str | Path) -> None:
        text = json.dumps(
            self.to_dict(), ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")
        ) + "\n"
        Path(path).write_text(text, encoding="utf-8", newline="\n")

    @classmethod
    def from_dict(cls, payload: Any) -> "MujocoTrajectory":
        if not isinstance(payload, dict):
            raise ValueError("trajectory JSON root must be an object")
        if payload.get("schema") != SCHEMA or payload.get("version") != SCHEMA_VERSION:
            raise ValueError("unsupported trajectory schema or version")
        if payload.get("units") != UNITS:
            raise ValueError("trajectory units do not match the supported SI contract")
        names = _joint_names(payload.get("joint_names", ()))
        raw_frames = payload.get("frames")
        if not isinstance(raw_frames, list):
            raise ValueError("frames must be an array")
        try:
            frames = tuple(
                TrajectoryFrame(
                    simulation_time_s=frame["simulation_time_s"],
                    joint_targets_rad=frame["joint_targets_rad"],
                    joint_positions_rad=frame["joint_positions_rad"],
                    gripper_target_width_m=frame["gripper_target_width_m"],
                    gripper_width_m=frame["gripper_width_m"],
                )
                for frame in raw_frames
            )
        except (KeyError, TypeError) as exc:
            raise ValueError("trajectory frame structure is invalid") from exc
        return cls(joint_names=names, frames=frames)

    @classmethod
    def load_json(cls, path: str | Path) -> "MujocoTrajectory":
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError("could not load trajectory JSON") from exc
        return cls.from_dict(payload)


class MujocoTrajectoryRecorder:
    """In-memory recorder with explicit start/stop/clear lifecycle."""

    def __init__(self) -> None:
        self._frames: list[TrajectoryFrame] = []
        self._recording = False

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def frames(self) -> tuple[TrajectoryFrame, ...]:
        return tuple(self._frames)

    def start(self, *, clear: bool = True) -> None:
        if self._recording:
            raise RuntimeError("recorder is already running")
        if clear:
            self._frames.clear()
        self._recording = True

    def record(
        self,
        simulation_time_s: float,
        joint_targets_rad: Sequence[float],
        joint_positions_rad: Sequence[float],
        gripper_target_width_m: float,
        gripper_width_m: float,
    ) -> TrajectoryFrame:
        if not self._recording:
            raise RuntimeError("recorder is not running")
        frame = TrajectoryFrame(
            simulation_time_s,
            tuple(joint_targets_rad),
            tuple(joint_positions_rad),
            gripper_target_width_m,
            gripper_width_m,
        )
        if self._frames and frame.simulation_time_s <= self._frames[-1].simulation_time_s:
            raise ValueError("simulation times must be strictly increasing")
        self._frames.append(frame)
        return frame

    def stop(self) -> MujocoTrajectory:
        if not self._recording:
            raise RuntimeError("recorder is not running")
        self._recording = False
        return self.trajectory()

    def clear(self) -> None:
        self._frames.clear()
        self._recording = False

    def trajectory(self) -> MujocoTrajectory:
        return MujocoTrajectory(ARM_JOINT_NAMES, tuple(self._frames))


@dataclass(frozen=True)
class PlaybackOutput:
    joint_targets_rad: tuple[float, ...]
    gripper_target_width_m: float
    state: str
    progress: float
    hold_requested: bool = False
    reference_joint_positions_rad: tuple[float, ...] = (0.0,) * 6
    reference_gripper_width_m: float = 0.0


@dataclass(frozen=True)
class PlaybackErrorThresholds:
    joint_rmse_rad: float = 0.03
    joint_max_abs_rad: float = 0.08
    gripper_rmse_m: float = 0.005
    gripper_max_abs_m: float = 0.012

    def __post_init__(self) -> None:
        for name in (
            "joint_rmse_rad",
            "joint_max_abs_rad",
            "gripper_rmse_m",
            "gripper_max_abs_m",
        ):
            value = _finite(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class PlaybackErrorReport:
    sample_count: int
    completed: bool
    passed: bool
    joint_tracking_rmse_rad: tuple[float, ...]
    joint_tracking_max_abs_rad: tuple[float, ...]
    joint_repeatability_rmse_rad: tuple[float, ...]
    joint_repeatability_max_abs_rad: tuple[float, ...]
    overall_tracking_rmse_rad: float
    overall_tracking_max_abs_rad: float
    overall_repeatability_rmse_rad: float
    overall_repeatability_max_abs_rad: float
    gripper_tracking_rmse_m: float
    gripper_tracking_max_abs_m: float
    gripper_repeatability_rmse_m: float
    gripper_repeatability_max_abs_m: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "sample_count": self.sample_count,
            "completed": self.completed,
            "passed": self.passed,
            "joint_tracking_rmse_rad": list(self.joint_tracking_rmse_rad),
            "joint_tracking_max_abs_rad": list(self.joint_tracking_max_abs_rad),
            "joint_repeatability_rmse_rad": list(self.joint_repeatability_rmse_rad),
            "joint_repeatability_max_abs_rad": list(self.joint_repeatability_max_abs_rad),
            "overall_tracking_rmse_rad": self.overall_tracking_rmse_rad,
            "overall_tracking_max_abs_rad": self.overall_tracking_max_abs_rad,
            "overall_repeatability_rmse_rad": self.overall_repeatability_rmse_rad,
            "overall_repeatability_max_abs_rad": self.overall_repeatability_max_abs_rad,
            "gripper_tracking_rmse_m": self.gripper_tracking_rmse_m,
            "gripper_tracking_max_abs_m": self.gripper_tracking_max_abs_m,
            "gripper_repeatability_rmse_m": self.gripper_repeatability_rmse_m,
            "gripper_repeatability_max_abs_m": self.gripper_repeatability_max_abs_m,
        }


class PlaybackErrorAccumulator:
    """Compare replay observations with commands and the original recording."""

    def __init__(self, thresholds: PlaybackErrorThresholds | None = None) -> None:
        self.thresholds = thresholds or PlaybackErrorThresholds()
        self.clear()

    def clear(self) -> None:
        self._tracking: list[tuple[float, ...]] = []
        self._repeatability: list[tuple[float, ...]] = []
        self._gripper_tracking: list[float] = []
        self._gripper_repeatability: list[float] = []

    def append(
        self,
        output: PlaybackOutput,
        joint_positions_rad: Sequence[float],
        gripper_width_m: float,
    ) -> None:
        actual = _six(joint_positions_rad, "joint_positions_rad")
        gripper_actual = _finite(gripper_width_m, "gripper_width_m")
        self._tracking.append(
            tuple(actual_value - target for actual_value, target in zip(actual, output.joint_targets_rad))
        )
        self._repeatability.append(
            tuple(
                actual_value - reference
                for actual_value, reference in zip(
                    actual, output.reference_joint_positions_rad
                )
            )
        )
        self._gripper_tracking.append(gripper_actual - output.gripper_target_width_m)
        self._gripper_repeatability.append(
            gripper_actual - output.reference_gripper_width_m
        )

    @staticmethod
    def _vector_metrics(samples: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], tuple[float, ...]]:
        if not samples:
            return (0.0,) * 6, (0.0,) * 6
        rmse = tuple(
            math.sqrt(sum(row[index] ** 2 for row in samples) / len(samples))
            for index in range(6)
        )
        maximum = tuple(max(abs(row[index]) for row in samples) for index in range(6))
        return rmse, maximum

    @staticmethod
    def _scalar_metrics(samples: Sequence[float]) -> tuple[float, float]:
        if not samples:
            return 0.0, 0.0
        return math.sqrt(sum(value * value for value in samples) / len(samples)), max(
            abs(value) for value in samples
        )

    def report(self, *, completed: bool) -> PlaybackErrorReport:
        tracking_rmse, tracking_max = self._vector_metrics(self._tracking)
        repeat_rmse, repeat_max = self._vector_metrics(self._repeatability)
        all_tracking = tuple(value for row in self._tracking for value in row)
        all_repeatability = tuple(value for row in self._repeatability for value in row)
        overall_tracking = self._scalar_metrics(all_tracking)
        overall_repeatability = self._scalar_metrics(all_repeatability)
        gripper_tracking = self._scalar_metrics(self._gripper_tracking)
        gripper_repeatability = self._scalar_metrics(self._gripper_repeatability)
        limits = self.thresholds
        tolerance = 1e-12
        passed = bool(
            completed
            and self._tracking
            and overall_tracking[0] <= limits.joint_rmse_rad + tolerance
            and overall_tracking[1] <= limits.joint_max_abs_rad + tolerance
            and gripper_tracking[0] <= limits.gripper_rmse_m + tolerance
            and gripper_tracking[1] <= limits.gripper_max_abs_m + tolerance
        )
        return PlaybackErrorReport(
            sample_count=len(self._tracking),
            completed=completed,
            passed=passed,
            joint_tracking_rmse_rad=tracking_rmse,
            joint_tracking_max_abs_rad=tracking_max,
            joint_repeatability_rmse_rad=repeat_rmse,
            joint_repeatability_max_abs_rad=repeat_max,
            overall_tracking_rmse_rad=overall_tracking[0],
            overall_tracking_max_abs_rad=overall_tracking[1],
            overall_repeatability_rmse_rad=overall_repeatability[0],
            overall_repeatability_max_abs_rad=overall_repeatability[1],
            gripper_tracking_rmse_m=gripper_tracking[0],
            gripper_tracking_max_abs_m=gripper_tracking[1],
            gripper_repeatability_rmse_m=gripper_repeatability[0],
            gripper_repeatability_max_abs_m=gripper_repeatability[1],
        )


class MujocoTrajectoryPlayback:
    """Single-pass, simulation-time-driven linear target playback."""

    def __init__(self, trajectory: MujocoTrajectory) -> None:
        if not isinstance(trajectory, MujocoTrajectory):
            raise TypeError("trajectory must be a MujocoTrajectory")
        self.trajectory = trajectory
        self._state = "idle"
        self._start_time_s: float | None = None
        self._pause_time_s: float | None = None
        self._paused_total_s = 0.0
        self._last_output = self._output_for_elapsed(0.0, state="idle")

    @property
    def state(self) -> str:
        return self._state

    @property
    def progress(self) -> float:
        return self._last_output.progress

    def start(self, simulation_time_s: float) -> PlaybackOutput:
        now = self._time(simulation_time_s)
        self._state = "playing"
        self._start_time_s = now
        self._pause_time_s = None
        self._paused_total_s = 0.0
        self._last_output = self._output_for_elapsed(0.0, state="playing")
        return self._last_output

    def update(self, simulation_time_s: float) -> PlaybackOutput:
        now = self._time(simulation_time_s)
        if self._state == "paused":
            return self._last_output
        if self._state != "playing" or self._start_time_s is None:
            raise RuntimeError("playback is not running")
        elapsed = now - self._start_time_s - self._paused_total_s
        if elapsed < 0.0:
            raise ValueError("playback simulation time cannot move backwards")
        # Fixed-step simulation clocks still accumulate binary float error;
        # accept a tiny absolute endpoint tolerance instead of requiring an
        # unnecessary extra physics step.
        finished = elapsed + 1e-12 >= self.trajectory.duration_s
        self._state = "finished" if finished else "playing"
        self._last_output = self._output_for_elapsed(
            self.trajectory.duration_s if finished else elapsed,
            state=self._state,
            hold_requested=finished,
        )
        return self._last_output

    def pause(self, simulation_time_s: float) -> PlaybackOutput:
        if self._state != "playing":
            raise RuntimeError("only playing playback can be paused")
        output = self.update(simulation_time_s)
        if output.state == "finished":
            return output
        self._state = "paused"
        self._pause_time_s = self._time(simulation_time_s)
        self._last_output = PlaybackOutput(
            output.joint_targets_rad,
            output.gripper_target_width_m,
            "paused",
            output.progress,
            reference_joint_positions_rad=output.reference_joint_positions_rad,
            reference_gripper_width_m=output.reference_gripper_width_m,
        )
        return self._last_output

    def resume(self, simulation_time_s: float) -> PlaybackOutput:
        if self._state != "paused" or self._pause_time_s is None:
            raise RuntimeError("only paused playback can be resumed")
        now = self._time(simulation_time_s)
        if now < self._pause_time_s:
            raise ValueError("playback simulation time cannot move backwards")
        self._paused_total_s += now - self._pause_time_s
        self._pause_time_s = None
        self._state = "playing"
        self._last_output = PlaybackOutput(
            self._last_output.joint_targets_rad,
            self._last_output.gripper_target_width_m,
            "playing",
            self._last_output.progress,
            reference_joint_positions_rad=self._last_output.reference_joint_positions_rad,
            reference_gripper_width_m=self._last_output.reference_gripper_width_m,
        )
        return self._last_output

    def stop(self) -> PlaybackOutput:
        if self._state not in ("playing", "paused"):
            raise RuntimeError("playback is not running")
        self._state = "stopped"
        self._pause_time_s = None
        self._last_output = PlaybackOutput(
            self._last_output.joint_targets_rad,
            self._last_output.gripper_target_width_m,
            "stopped",
            self._last_output.progress,
            hold_requested=True,
            reference_joint_positions_rad=self._last_output.reference_joint_positions_rad,
            reference_gripper_width_m=self._last_output.reference_gripper_width_m,
        )
        return self._last_output

    @staticmethod
    def _time(value: Any) -> float:
        result = _finite(value, "playback simulation time")
        if result < 0.0:
            raise ValueError("playback simulation time must be non-negative")
        return result

    def _output_for_elapsed(
        self, elapsed_s: float, *, state: str, hold_requested: bool = False
    ) -> PlaybackOutput:
        frames = self.trajectory.frames
        relative_times = tuple(
            frame.simulation_time_s - frames[0].simulation_time_s for frame in frames
        )
        if len(frames) == 1 or elapsed_s >= relative_times[-1]:
            target = frames[-1]
            progress = 1.0
            joints = target.joint_targets_rad
            gripper = target.gripper_target_width_m
            reference_joints = target.joint_positions_rad
            reference_gripper = target.gripper_width_m
        else:
            upper = bisect_right(relative_times, elapsed_s)
            if upper == 0:
                lower = upper = 0
                fraction = 0.0
            else:
                lower = upper - 1
                span = relative_times[upper] - relative_times[lower]
                fraction = (elapsed_s - relative_times[lower]) / span
            start, end = frames[lower], frames[upper]
            joints = tuple(
                a + fraction * (b - a)
                for a, b in zip(start.joint_targets_rad, end.joint_targets_rad)
            )
            gripper = start.gripper_target_width_m + fraction * (
                end.gripper_target_width_m - start.gripper_target_width_m
            )
            reference_joints = tuple(
                a + fraction * (b - a)
                for a, b in zip(start.joint_positions_rad, end.joint_positions_rad)
            )
            reference_gripper = start.gripper_width_m + fraction * (
                end.gripper_width_m - start.gripper_width_m
            )
            progress = 1.0 if self.trajectory.duration_s == 0.0 else elapsed_s / self.trajectory.duration_s
        return PlaybackOutput(
            joints,
            gripper,
            state,
            min(1.0, max(0.0, progress)),
            hold_requested,
            reference_joints,
            reference_gripper,
        )
