from __future__ import annotations

import json
import math

import pytest

from rebotarm_simulation.mujoco_trajectory import (
    ARM_JOINT_NAMES,
    MujocoTrajectory,
    MujocoTrajectoryPlayback,
    MujocoTrajectoryRecorder,
    SCHEMA,
    SCHEMA_VERSION,
    TrajectoryFrame,
    UNITS,
)


def _frame(time, target=0.0, actual=0.0, gripper_target=0.08, gripper=0.07):
    return TrajectoryFrame(
        time,
        (target,) * 6,
        (actual,) * 6,
        gripper_target,
        gripper,
    )


def _trajectory():
    return MujocoTrajectory(
        ARM_JOINT_NAMES,
        (_frame(10.0, 0.0, gripper_target=0.08), _frame(12.0, 1.0, gripper_target=0.02)),
    )


def test_recorder_lifecycle_and_memory_snapshot():
    recorder = MujocoTrajectoryRecorder()
    with pytest.raises(RuntimeError, match="not running"):
        recorder.record(0, [0] * 6, [0] * 6, 0.08, 0.08)
    recorder.start()
    recorder.record(3.0, [0] * 6, [0.1] * 6, 0.08, 0.07)
    recorder.record(3.1, [0.2] * 6, [0.15] * 6, 0.04, 0.05)
    trajectory = recorder.stop()
    assert not recorder.is_recording
    assert len(recorder.frames) == 2
    assert trajectory.duration_s == pytest.approx(0.1)
    recorder.clear()
    assert recorder.frames == ()
    with pytest.raises(ValueError, match="at least one"):
        recorder.trajectory()


def test_recorder_can_append_across_sessions_only_when_requested():
    recorder = MujocoTrajectoryRecorder()
    recorder.start()
    recorder.record(1, [0] * 6, [0] * 6, 0.08, 0.08)
    recorder.stop()
    recorder.start(clear=False)
    recorder.record(2, [1] * 6, [1] * 6, 0.02, 0.02)
    assert len(recorder.stop().frames) == 2
    recorder.start()
    assert recorder.frames == ()


def test_json_is_deterministic_versioned_and_round_trips(tmp_path):
    trajectory = _trajectory()
    first = tmp_path / "first.json"
    second = tmp_path / "second.json"
    trajectory.save_json(first)
    trajectory.save_json(second)
    assert first.read_bytes() == second.read_bytes()
    payload = json.loads(first.read_text())
    assert payload["schema"] == SCHEMA
    assert payload["version"] == SCHEMA_VERSION
    assert payload["units"] == UNITS
    assert payload["joint_names"] == list(ARM_JOINT_NAMES)
    assert MujocoTrajectory.load_json(first) == trajectory


@pytest.mark.parametrize(
    "change, match",
    [
        (lambda p: p.update(version=99), "schema or version"),
        (lambda p: p.update(units={"time": "ms"}), "units"),
        (lambda p: p.update(joint_names=list(reversed(ARM_JOINT_NAMES))), "joint_names"),
        (lambda p: p["frames"][1].update(simulation_time_s=10.0), "strictly"),
        (lambda p: p["frames"][1].update(gripper_width_m=math.nan), "finite"),
    ],
)
def test_json_strictly_validates_contract(change, match):
    payload = _trajectory().to_dict()
    change(payload)
    with pytest.raises(ValueError, match=match):
        MujocoTrajectory.from_dict(payload)


def test_frames_reject_nonfinite_wrong_width_negative_gripper_and_time_order():
    with pytest.raises(ValueError, match="six"):
        _frame(0, target=0).__class__(0, (0,) * 5, (0,) * 6, 0.1, 0.1)
    with pytest.raises(ValueError, match="finite"):
        _frame(math.inf)
    with pytest.raises(ValueError, match="non-negative"):
        TrajectoryFrame(0, (0,) * 6, (0,) * 6, -0.1, 0.0)
    with pytest.raises(ValueError, match="strictly"):
        MujocoTrajectory(ARM_JOINT_NAMES, (_frame(2), _frame(1)))


def test_playback_interpolates_targets_not_observations_and_finishes_with_hold():
    playback = MujocoTrajectoryPlayback(_trajectory())
    initial = playback.start(100.0)
    assert initial.state == "playing"
    assert initial.joint_targets_rad == pytest.approx((0.0,) * 6)
    middle = playback.update(101.0)
    assert middle.joint_targets_rad == pytest.approx((0.5,) * 6)
    assert middle.gripper_target_width_m == pytest.approx(0.05)
    assert middle.progress == pytest.approx(0.5)
    assert not middle.hold_requested
    final = playback.update(102.0)
    assert final.state == "finished"
    assert final.progress == 1.0
    assert final.hold_requested
    with pytest.raises(RuntimeError, match="not running"):
        playback.update(103.0)


def test_pause_resume_excludes_paused_simulation_time():
    playback = MujocoTrajectoryPlayback(_trajectory())
    playback.start(20.0)
    paused = playback.pause(20.5)
    assert paused.state == "paused"
    assert paused.progress == pytest.approx(0.25)
    assert playback.update(50.0) == paused
    resumed = playback.resume(50.0)
    assert resumed.state == "playing"
    continued = playback.update(50.5)
    assert continued.progress == pytest.approx(0.5)


def test_manual_stop_requests_hold_and_preserves_progress():
    playback = MujocoTrajectoryPlayback(_trajectory())
    playback.start(0.0)
    playback.update(0.4)
    stopped = playback.stop()
    assert stopped.state == "stopped"
    assert stopped.progress == pytest.approx(0.2)
    assert stopped.hold_requested


def test_single_frame_playback_finishes_deterministically_and_requests_hold():
    playback = MujocoTrajectoryPlayback(MujocoTrajectory(ARM_JOINT_NAMES, (_frame(5),)))
    playback.start(1.0)
    output = playback.update(1.0)
    assert output.state == "finished"
    assert output.progress == 1.0
    assert output.hold_requested


def test_playback_rejects_time_reversal_and_invalid_transitions():
    playback = MujocoTrajectoryPlayback(_trajectory())
    with pytest.raises(RuntimeError, match="not running"):
        playback.update(0)
    playback.start(5)
    with pytest.raises(ValueError, match="backwards"):
        playback.update(4)
    with pytest.raises(RuntimeError, match="paused"):
        playback.resume(5)
