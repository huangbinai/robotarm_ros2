from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from rebotarm_dashboard.teach_replay_config import TeachReplayParameterAdapter
from rebotarm_motion.collision_precheck import CollisionPrecheckConfig
from rebotarm_motion.teach_replay_start_alignment import MoveItStartAlignmentConfig
from rebotarm_teach.teach_recording import TeachSample, encode_teach_sample
from rebotarm_teach.teach_replay_trajectory_builder import TeachReplayTrajectoryConfig
from rebotarm_teach.teach_replay_workflow import (
    TeachReplayPreparationConfig,
    TeachReplayWorkflow,
)


class _CollisionPrechecker:
    def __init__(self) -> None:
        self.calls = []

    def check_positions(self, **kwargs):
        self.calls.append(kwargs)
        return {"state": "pass", "checked_samples": len(kwargs["positions_list"])}


class _AlignmentPrechecker:
    def __init__(self) -> None:
        self.calls = []

    def summary(self, info_payload, **kwargs):
        self.calls.append((info_payload, kwargs))
        return {"state": "ready"}


class _StartAligner:
    def __init__(self) -> None:
        self.calls = []

    def append(self, trajectory, **kwargs):
        self.calls.append((trajectory, kwargs))
        return 0.5


@dataclass
class _TrajectoryResult:
    trajectory: object


class _TrajectoryBuilder:
    def __init__(self) -> None:
        self.calls = []

    def build(self, **kwargs):
        self.calls.append(kwargs)
        return _TrajectoryResult(trajectory={"source": "prepared"})


def _preparation_config() -> TeachReplayPreparationConfig:
    return TeachReplayPreparationConfig(
        smoothing_enabled=False,
        smoothing_window=3,
        filter_enabled=False,
        filter_cutoff_hz=5.0,
        filter_sample_rate_hz=50.0,
        resample_enabled=False,
        resample_rate_hz=50.0,
        replay_speed=1.0,
        max_velocity_rad_s=3.0,
        max_acceleration_rad_s2=5.0,
        max_jerk_rad_s3=20.0,
        time_parameterization_method="current_jerk_retime",
        large_motion_span_rad=0.8,
        large_motion_total_rad=2.5,
        large_motion_max_speed=1.0,
    )


def _trajectory_config(*, use_moveit: bool) -> TeachReplayTrajectoryConfig:
    return TeachReplayTrajectoryConfig(
        use_moveit_start_align=use_moveit,
        start_hold_sec=0.1,
        soft_start_duration=0.2,
        soft_start_steps=2,
        first_hold_sec=0.1,
        yellow_max_speed=0.6,
        initial_replay_delay_sec=0.05,
        max_velocity_rad_s=3.0,
        max_acceleration_rad_s2=5.0,
        max_jerk_rad_s3=20.0,
    )


def _alignment_config() -> MoveItStartAlignmentConfig:
    return MoveItStartAlignmentConfig(
        start_hold_sec=0.1,
        first_hold_sec=0.1,
        skip_threshold=0.05,
        joint_goal_tolerance=0.02,
        velocity_scaling=0.4,
        acceleration_scaling=0.3,
    )


def _workflow():
    trajectory_builder = _TrajectoryBuilder()
    start_aligner = _StartAligner()
    alignment_prechecker = _AlignmentPrechecker()
    collision_prechecker = _CollisionPrechecker()
    workflow = TeachReplayWorkflow(
        trajectory_builder=trajectory_builder,
        moveit_start_aligner=start_aligner,
        moveit_start_align_prechecker=alignment_prechecker,
        collision_prechecker=collision_prechecker,
    )
    return workflow, trajectory_builder, start_aligner, alignment_prechecker, collision_prechecker


def test_prepared_record_is_reused_for_trajectory_build(tmp_path) -> None:
    sample = TeachSample(
        stamp=0.0,
        joint_names=("joint1", "joint2"),
        positions=(0.1, -0.1),
        velocities=(0.0, 0.0),
        efforts=(),
        motor_status={},
        arm_state="RECORDING",
    )
    record_path = tmp_path / "record.jsonl"
    record_path.write_text(encode_teach_sample(sample) + "\n", encoding="utf-8")
    workflow, builder, aligner, _, _ = _workflow()

    record = workflow.prepare_record(record_path, config=_preparation_config())
    trajectory = workflow.build_trajectory(
        record,
        current_positions={"joint1": 0.0, "joint2": 0.0},
        start_band="direct",
        settings={"align_duration": 1.0, "align_steps": 2, "final_hold_sec": 0.2},
        trajectory_config=_trajectory_config(use_moveit=False),
        alignment_config=_alignment_config(),
    )

    assert trajectory == {"source": "prepared"}
    assert builder.calls[0]["prepared"] is record.prepared
    assert aligner.calls == []
    assert record.prepared_path.endswith("record.prepared.jsonl")


def test_loaded_record_can_be_prepared_without_reloading_source_file(tmp_path) -> None:
    sample = TeachSample(
        stamp=0.0,
        joint_names=("joint1",),
        positions=(0.1,),
        velocities=(0.0,),
        efforts=(),
        motor_status={},
        arm_state="RECORDING",
    )
    source_path = tmp_path / "already_loaded.jsonl"
    workflow, _, _, _, _ = _workflow()

    record = workflow.prepare_loaded_record(
        source_path,
        [sample],
        config=_preparation_config(),
    )

    assert record.source_samples == [sample]
    assert record.source_path == str(source_path)
    assert Path(record.prepared_path).exists()


def test_workflow_owns_alignment_and_collision_collaborators() -> None:
    workflow, builder, aligner, alignment_prechecker, collision_prechecker = _workflow()
    config = CollisionPrecheckConfig(
        enabled=True,
        service="/check_state_validity",
        group_name="arm_with_gripper",
        max_samples=10,
        timeout_sec=0.5,
    )
    sample = TeachSample(
        stamp=0.0,
        joint_names=("joint1",),
        positions=(0.1,),
        velocities=(0.0,),
        efforts=(),
        motor_status={},
        arm_state="RECORDING",
    )

    collision = workflow.check_samples([sample], config=config)
    alignment = workflow.summarize_start_alignment(
        {"start_band": "align"},
        config=object(),
        samples=[sample],
        plan=True,
    )

    assert collision == {"state": "pass", "checked_samples": 1}
    assert collision_prechecker.calls[0]["joint_names"] == ("joint1",)
    assert alignment == {"state": "ready"}
    assert alignment_prechecker.calls[0][1]["plan"] is True
    assert builder.calls == []
    assert aligner.calls == []


def test_dashboard_parameter_adapter_builds_bounded_domain_configs() -> None:
    values = {
        "max_replay_velocity_rad_s": 1.5,
        "max_replay_velocity_rad_s_by_joint": [3.0, 1.8],
        "collision_check_enabled": True,
        "collision_check_service": "/check_state_validity",
        "collision_group_name": "arm_with_gripper",
        "collision_check_max_samples": 0,
        "collision_check_timeout_sec": 0.0,
    }

    def get_parameter(name):
        return type("Parameter", (), {"value": values[name]})()

    adapter = TeachReplayParameterAdapter(get_parameter)
    collision = adapter.collision(
        default_joint_positions=(("left_finger_joint", 0.03),),
    )

    assert adapter.velocity_limits(("joint1", "joint2")) == (3.0, 1.8)
    assert adapter.velocity_limits(("joint1",)) == 1.5
    assert collision.max_samples == 1
    assert collision.timeout_sec == 0.1
    assert collision.default_joint_positions == (("left_finger_joint", 0.03),)
