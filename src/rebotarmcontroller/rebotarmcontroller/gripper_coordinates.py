from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GripperCoordinateModel:
    """Calibrated conversion and acceptance bounds for the real gripper."""

    maximum_distance_m: float
    maximum_command_opening_m: float
    open_angle_rad: float
    open_soft_limit_rad: float
    coordinate_tolerance_rad: float
    closed_feedback_tolerance_rad: float

    @property
    def feedback_lower_rad(self) -> float:
        return self.open_angle_rad - self.coordinate_tolerance_rad

    @property
    def feedback_upper_rad(self) -> float:
        return self.closed_feedback_tolerance_rad

    def accepts_feedback_angle(self, angle_rad: float) -> bool:
        angle = float(angle_rad)
        return bool(
            math.isfinite(angle)
            and self.feedback_lower_rad <= angle <= self.feedback_upper_rad
        )

    def validate_position_request(
        self,
        position_m: float,
        max_effort: float,
    ) -> tuple[float, float]:
        position = float(position_m)
        effort = float(max_effort)
        if not math.isfinite(position) or not math.isfinite(effort):
            raise ValueError("gripper position and effort must be finite")
        if not 0.0 <= position <= self.maximum_command_opening_m:
            raise ValueError(
                "gripper position must be within "
                f"[0.0, {self.maximum_command_opening_m:g}] m"
            )
        return position, effort

    def opening_to_angle(self, opening_m: float) -> float:
        angle = (float(opening_m) / self.maximum_distance_m) * self.open_angle_rad
        return max(angle, self.open_soft_limit_rad)

    def angle_to_opening(self, angle_rad: float) -> float:
        angle = float(angle_rad)
        if not self.accepts_feedback_angle(angle):
            return float("nan")
        distance = (angle / self.open_angle_rad) * self.maximum_distance_m
        return float(min(max(distance, 0.0), self.maximum_distance_m))


DEFAULT_GRIPPER_COORDINATES = GripperCoordinateModel(
    maximum_distance_m=0.09,
    maximum_command_opening_m=0.085,
    open_angle_rad=-5.0,
    open_soft_limit_rad=-4.9,
    coordinate_tolerance_rad=2.0 * 25.0 / 65535.0,
    # About one millimetre at the closed end is accepted only as feedback.
    # It does not move zero, enlarge commands, or change command clamping.
    closed_feedback_tolerance_rad=0.001 * 5.0 / 0.09,
)
