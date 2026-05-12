from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ControlMode(str, Enum):
    SIMULATION = "simulation"
    REAL = "real"


class ExecutionState(str, Enum):
    IDLE = "idle"
    PREVIEW_READY = "preview_ready"
    EXECUTING = "executing"
    ESTOPPED = "estopped"


@dataclass(frozen=True)
class PoseTarget:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class PreviewSolveResult:
    success: bool
    joint_positions: tuple[float, ...]
    message: str


@dataclass(frozen=True)
class PreviewCommand:
    command_type: str
    reachable: bool
    message: str
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    pose_target: PoseTarget | None = None


@dataclass(frozen=True)
class ExecutionRequest:
    mode: ControlMode
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    duration: float
    preview_only: bool
    source_command: str


@dataclass(frozen=True)
class ExecutionDecision:
    accepted: bool
    message: str
    request: ExecutionRequest | None
