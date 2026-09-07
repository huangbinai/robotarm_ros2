from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class HardwareRuntimeConfig:
    hardware_feedback_rate_hz: float
    feedback_stale_timeout_sec: float
    gripper_position_torque_cap_nm: float
    gripper_position_max_speed_rad_s: float
    gripper_position_timeout_margin_sec: float
    grasp_hold_timeout_sec: float
    gripper_contact_torque_min_nm: float

    @classmethod
    def validate(
        cls,
        *,
        hardware_feedback_rate_hz: float,
        feedback_stale_timeout_sec: float,
        gripper_position_torque_cap_nm: float,
        gripper_position_max_speed_rad_s: float,
        gripper_position_timeout_margin_sec: float,
        grasp_hold_timeout_sec: float,
        gripper_contact_torque_min_nm: float,
    ) -> "HardwareRuntimeConfig":
        feedback_rate = _bounded(
            "hardware_feedback_rate_hz",
            hardware_feedback_rate_hz,
            20.0,
            100.0,
            unit="Hz",
        )
        stale_timeout = _bounded(
            "feedback_stale_timeout_sec",
            feedback_stale_timeout_sec,
            0.05,
            2.0,
            unit="s",
        )
        torque_cap = _bounded(
            "gripper_position_torque_cap_nm",
            gripper_position_torque_cap_nm,
            0.05,
            1.5,
        )
        position_speed = _bounded(
            "gripper_position_max_speed_rad_s",
            gripper_position_max_speed_rad_s,
            0.05,
            3.0,
        )
        timeout_margin = _bounded(
            "gripper_position_timeout_margin_sec",
            gripper_position_timeout_margin_sec,
            0.1,
            10.0,
        )
        hold_timeout = _bounded(
            "grasp_hold_timeout_sec",
            grasp_hold_timeout_sec,
            0.1,
            120.0,
        )
        contact_torque = _bounded(
            "gripper_contact_torque_min_nm",
            gripper_contact_torque_min_nm,
            0.0,
            1.5,
        )
        return cls(
            hardware_feedback_rate_hz=feedback_rate,
            feedback_stale_timeout_sec=stale_timeout,
            gripper_position_torque_cap_nm=torque_cap,
            gripper_position_max_speed_rad_s=position_speed,
            gripper_position_timeout_margin_sec=timeout_margin,
            grasp_hold_timeout_sec=hold_timeout,
            gripper_contact_torque_min_nm=contact_torque,
        )

    @property
    def hardware_feedback_period_sec(self) -> float:
        return 1.0 / self.hardware_feedback_rate_hz


def _bounded(name: str, value: float, minimum: float, maximum: float, *, unit: str = "") -> float:
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        suffix = f" {unit}" if unit else ""
        raise ValueError(
            f"{name} must be finite and within [{minimum:g}, {maximum:g}]{suffix}"
        )
    return number
