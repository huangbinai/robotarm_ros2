from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rebotarm_motion.collision_precheck import CollisionPrecheckConfig
from rebotarm_motion.teach_replay_start_align_precheck import (
    MoveItStartAlignPrecheckConfig,
)
from rebotarm_motion.teach_replay_start_alignment import MoveItStartAlignmentConfig

from .teach_recording import (
    PreparedTeachReplay,
    TeachSample,
    load_teach_samples,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    teach_trajectory_preview_to_dict,
    write_prepared_teach_record,
)
from .teach_replay_trajectory_builder import TeachReplayTrajectoryConfig


@dataclass(frozen=True)
class TeachReplayPreparationConfig:
    smoothing_enabled: bool
    smoothing_window: int
    filter_enabled: bool
    filter_cutoff_hz: float
    filter_sample_rate_hz: float
    resample_enabled: bool
    resample_rate_hz: float
    replay_speed: float
    max_velocity_rad_s: Any
    max_acceleration_rad_s2: float
    max_jerk_rad_s3: float
    time_parameterization_method: str
    large_motion_span_rad: float
    large_motion_total_rad: float
    large_motion_max_speed: float


PreparationConfigSource = TeachReplayPreparationConfig | Callable[
    [tuple[str, ...]], TeachReplayPreparationConfig
]
CollisionConfigSource = CollisionPrecheckConfig | Callable[
    [tuple[str, ...]], CollisionPrecheckConfig
]


@dataclass(frozen=True)
class PreparedReplayRecord:
    source_path: str
    source_samples: list[TeachSample]
    prepared: PreparedTeachReplay
    prepared_path: str
    prepared_samples: list[TeachSample]
    payload: dict[str, Any]


class TeachReplayWorkflow:
    """Own teach replay preparation and motion planning outside UI transports."""

    def __init__(
        self,
        *,
        trajectory_builder: Any,
        moveit_start_aligner: Any,
        moveit_start_align_prechecker: Any,
        collision_prechecker: Any,
    ) -> None:
        self._trajectory_builder = trajectory_builder
        self._moveit_start_aligner = moveit_start_aligner
        self._moveit_start_align_prechecker = moveit_start_align_prechecker
        self._collision_prechecker = collision_prechecker

    def prepare_record(
        self,
        record_path: str | Path,
        *,
        config: PreparationConfigSource,
    ) -> PreparedReplayRecord:
        source_path = str(record_path)
        source_samples = load_teach_samples(source_path)
        if not source_samples:
            raise ValueError("record contains no samples")
        joint_names = tuple(source_samples[0].joint_names)
        resolved_config = config(joint_names) if callable(config) else config
        prepared = self.prepare_samples(source_samples, config=resolved_config)
        prepared_path = str(write_prepared_teach_record(source_path, prepared))
        return PreparedReplayRecord(
            source_path=source_path,
            source_samples=source_samples,
            prepared=prepared,
            prepared_path=prepared_path,
            prepared_samples=load_teach_samples(prepared_path),
            payload=prepared_teach_replay_to_dict(prepared),
        )

    @staticmethod
    def prepare_samples(
        samples: list[TeachSample],
        *,
        config: TeachReplayPreparationConfig,
    ) -> PreparedTeachReplay:
        return prepare_teach_replay_samples(
            samples,
            smoothing_enabled=config.smoothing_enabled,
            smoothing_window=config.smoothing_window,
            filter_enabled=config.filter_enabled,
            filter_cutoff_hz=config.filter_cutoff_hz,
            filter_sample_rate_hz=config.filter_sample_rate_hz,
            resample_enabled=config.resample_enabled,
            resample_rate_hz=config.resample_rate_hz,
            retime_enabled=True,
            replay_speed=config.replay_speed,
            max_velocity_rad_s=config.max_velocity_rad_s,
            max_acceleration_rad_s2=config.max_acceleration_rad_s2,
            max_jerk_rad_s3=config.max_jerk_rad_s3,
            time_parameterization_method=config.time_parameterization_method,
            large_motion_span_rad=config.large_motion_span_rad,
            large_motion_total_rad=config.large_motion_total_rad,
            large_motion_max_speed=config.large_motion_max_speed,
        )

    def build_preview(
        self,
        record_path: str | Path,
        *,
        max_points: int,
        preparation_config: PreparationConfigSource,
        collision_config: CollisionConfigSource,
        info_payload: dict,
    ) -> dict:
        record = self.prepare_record(record_path, config=preparation_config)
        payload = teach_trajectory_preview_to_dict(record.prepared_samples, max_points=max_points)
        payload.update(
            {
                "accepted": True,
                "curve_source": "prepared",
                "path": record.prepared_path,
                "raw_record_path": record.source_path,
                "prepared_record_path": record.prepared_path,
                "prepared_replay": record.payload,
                "collision_precheck": self.check_samples(
                    record.prepared_samples,
                    config=collision_config,
                ),
                "info": info_payload,
            }
        )
        return payload

    def summarize_start_alignment(
        self,
        info_payload: dict,
        *,
        config: MoveItStartAlignPrecheckConfig,
        samples: list[TeachSample] | None = None,
        plan: bool = False,
    ) -> dict:
        return self._moveit_start_align_prechecker.summary(
            info_payload,
            config=config,
            samples=samples,
            plan=plan,
        )

    def build_trajectory(
        self,
        record: PreparedReplayRecord,
        *,
        current_positions: dict[str, float],
        start_band: str,
        settings: dict[str, float | int],
        trajectory_config: TeachReplayTrajectoryConfig,
        alignment_config: MoveItStartAlignmentConfig,
    ) -> Any:
        def append_start_alignment(
            trajectory: Any,
            *,
            current_positions: tuple[float, ...],
            first_positions: tuple[float, ...],
        ) -> float:
            return self._moveit_start_aligner.append(
                trajectory,
                current_positions=current_positions,
                first_positions=first_positions,
                config=alignment_config,
            )

        result = self._trajectory_builder.build(
            prepared=record.prepared,
            current_positions=current_positions,
            start_band=start_band,
            settings=settings,
            config=trajectory_config,
            moveit_start_alignment=(
                append_start_alignment if trajectory_config.use_moveit_start_align else None
            ),
        )
        return result.trajectory

    def check_samples(
        self,
        samples: list[TeachSample],
        *,
        config: CollisionConfigSource,
    ) -> dict:
        if not samples:
            return self.check_positions((), [], config=config)
        return self.check_positions(
            tuple(samples[0].joint_names),
            [tuple(sample.positions) for sample in samples],
            config=config,
        )

    def check_trajectory(
        self,
        trajectory: Any,
        *,
        config: CollisionConfigSource,
    ) -> dict:
        positions = [
            tuple(point.positions)
            for point in getattr(trajectory, "points", [])
            if getattr(point, "positions", None)
        ]
        return self.check_positions(
            tuple(trajectory.joint_names),
            positions,
            config=config,
        )

    def check_positions(
        self,
        joint_names: tuple[str, ...],
        positions: list[tuple[float, ...]],
        *,
        config: CollisionConfigSource,
    ) -> dict:
        resolved_config = config(joint_names) if callable(config) else config
        return self._collision_prechecker.check_positions(
            joint_names=joint_names,
            positions_list=positions,
            config=resolved_config,
        )
