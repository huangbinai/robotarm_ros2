from __future__ import annotations

from dataclasses import dataclass

from rebotarm_motion.replay_start_policy import ReplayStartBand, ReplayStartDecision
from rebotarm_motion.teach_sample_processing import RetimedTeachPoint


@dataclass(frozen=True)
class TeachSample:
    stamp: float
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    motor_status: dict[str, int]
    arm_state: str


@dataclass(frozen=True)
class TeachTrajectoryEvent:
    sample: int
    joint_name: str
    level: str
    message: str
    delta_rad: float
    velocity_rad_s: float | None
    acceleration_rad_s2: float | None = None
    jerk_rad_s3: float | None = None


@dataclass(frozen=True)
class TeachTrajectoryQuality:
    risk_level: str
    replay_policy: str
    allow_real_replay: bool
    requires_safe_retiming: bool
    max_jump_rad: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    worst_joint: str
    worst_sample: int
    anomalies: tuple[str, ...]
    events: tuple[TeachTrajectoryEvent, ...]
    green_jump_rad: float
    yellow_jump_rad: float
    velocity_limit_rad_s: float
    acceleration_limit_rad_s2: float
    max_jerk_rad_s3: float = 0.0
    jerk_limit_rad_s3: float = 999.0


@dataclass(frozen=True)
class PreparedTeachReplay:
    samples: list[TeachSample]
    raw_quality: TeachTrajectoryQuality
    filtered_quality: TeachTrajectoryQuality
    retimed_quality: TeachTrajectoryQuality
    smoothing_applied: bool
    filter_applied: bool
    resample_applied: bool
    retime_applied: bool
    smoothing_window: int
    filter_cutoff_hz: float
    filter_sample_rate_hz: float
    resample_rate_hz: float
    retimed_points: list[RetimedTeachPoint]
    large_motion: bool = False
    max_joint_span_rad: float = 0.0
    total_joint_motion_rad: float = 0.0
    requested_replay_speed: float = 1.0
    effective_replay_speed: float = 1.0
    large_motion_max_speed: float = 1.0
    time_parameterization_requested_method: str = "auto"
    time_parameterization_used_method: str = "current_jerk_retime"
    time_parameterization_message: str = ""

    @property
    def before_quality(self) -> TeachTrajectoryQuality:
        return self.raw_quality

    @property
    def after_quality(self) -> TeachTrajectoryQuality:
        return self.retimed_quality


@dataclass(frozen=True)
class TeachRecordInfo:
    path: str
    exists: bool
    samples: int
    duration_sec: float
    joint_names: tuple[str, ...]
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]
    start_band: str
    max_error: float | None
    worst_joint: str
    per_joint_error: dict[str, float]
    anomalies: tuple[str, ...]
    message: str
    quality: TeachTrajectoryQuality | None = None


@dataclass(frozen=True)
class TeachDryRunDecision:
    accepted: bool
    state: str
    message: str
