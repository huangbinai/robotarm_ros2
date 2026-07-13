"""Pure motor-control math shared by MuJoCo front ends.

Firmware gains remain visible as raw parameters.  Values that translate the
firmware velocity-loop output into physical torque live in the separate,
explicitly named simulation calibration file.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


@dataclass(frozen=True)
class ArmJointParameters:
    name: str
    pos_kp: float
    pos_ki: float
    vel_kp_raw: float
    vel_ki_raw: float
    velocity_limit_rad_s: float
    effort_limit_nm: float
    firmware_to_torque_scale: float


@dataclass(frozen=True)
class GripperParameters:
    firmware_default_kp: float
    firmware_default_kd: float
    move_kp: float
    move_kd: float
    closing_kp: float
    closing_kd: float
    hold_kp: float
    hold_kd: float
    motor_radians_per_opening_m: float
    transmission_efficiency: float
    motor_torque_limit_nm: float
    displacement_min_m: float
    displacement_max_m: float


@dataclass(frozen=True)
class MotorControlParameters:
    rate_hz: float
    arm: tuple[ArmJointParameters, ...]
    gripper: GripperParameters
    position_integral_limit: float
    velocity_integral_limit: float


@dataclass(frozen=True)
class GripperCommand:
    mode: str
    target_displacement_m: float
    motor_target_rad: float
    kp: float
    kd: float
    motor_torque_nm: float
    opening_force_n: float


def _read_yaml(path: Path) -> dict:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected YAML mapping in {path}")
    return payload


def load_motor_control_parameters(repo_root: str | Path) -> MotorControlParameters:
    root = Path(repo_root)
    arm_yaml = _read_yaml(root / "src/rebotarm_bringup/config/arm.yaml")
    gripper_yaml = _read_yaml(root / "src/rebotarm_bringup/config/gripper.yaml")
    calibration = _read_yaml(
        root / "src/rebotarm_simulation/config/motor_control_calibration.yaml"
    )
    urdf = ET.parse(root / "src/rebotarm_moveit_config/config/rebotarm.urdf").getroot()
    efforts = {
        node.attrib["name"]: float(node.find("limit").attrib["effort"])
        for node in urdf.findall("joint")
        if node.find("limit") is not None and "effort" in node.find("limit").attrib
    }

    arm_cal = calibration["arm"]
    joints = []
    for entry in arm_yaml["joints"]:
        pos_vel = entry["POS_VEL"]
        name = entry["name"]
        joints.append(
            ArmJointParameters(
                name=name,
                pos_kp=float(pos_vel["pos_kp"]),
                pos_ki=float(pos_vel["pos_ki"]),
                vel_kp_raw=float(pos_vel["vel_kp"]),
                vel_ki_raw=float(pos_vel["vel_ki"]),
                velocity_limit_rad_s=float(pos_vel["vlim"]),
                effort_limit_nm=efforts[name],
                firmware_to_torque_scale=float(arm_cal[name]["firmware_to_torque_scale"]),
            )
        )

    source_gripper = gripper_yaml["gripper"][0]["MIT"]
    gripper_cal = calibration["gripper"]
    modes = gripper_cal["modes"]
    displacement_range = gripper_cal["displacement_range_m"]
    gripper = GripperParameters(
        firmware_default_kp=float(source_gripper["kp"]),
        firmware_default_kd=float(source_gripper["kd"]),
        move_kp=float(modes["move"]["kp"]),
        move_kd=float(modes["move"]["kd"]),
        closing_kp=float(modes["closing"]["kp"]),
        closing_kd=float(modes["closing"]["kd"]),
        hold_kp=float(modes["hold"]["kp"]),
        hold_kd=float(modes["hold"]["kd"]),
        motor_radians_per_opening_m=float(gripper_cal["motor_radians_per_opening_m"]),
        transmission_efficiency=float(gripper_cal["transmission_efficiency"]),
        motor_torque_limit_nm=float(gripper_cal["motor_torque_limit_nm"]),
        displacement_min_m=float(displacement_range[0]),
        displacement_max_m=float(displacement_range[1]),
    )
    return MotorControlParameters(
        rate_hz=float(arm_yaml["rate"]),
        arm=tuple(joints),
        gripper=gripper,
        position_integral_limit=float(arm_cal["position_integral_limit_rad_s"]),
        velocity_integral_limit=float(arm_cal["velocity_integral_limit_rad"]),
    )


class PosVelController:
    """100 Hz cascaded position/velocity PI with conditional anti-windup."""

    def __init__(self, parameters: MotorControlParameters):
        self.parameters = parameters
        self.position_integral = np.zeros(6, dtype=float)
        self.velocity_integral = np.zeros(6, dtype=float)
        self.target_velocity = np.zeros(6, dtype=float)
        self.position_integral_limit = np.full(6, parameters.position_integral_limit)
        self.velocity_integral_limit = np.full(6, parameters.velocity_integral_limit)

    def reset(self) -> None:
        self.position_integral.fill(0.0)
        self.velocity_integral.fill(0.0)
        self.target_velocity.fill(0.0)

    def compute(self, target, position, velocity, dt: float) -> np.ndarray:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        target = _vector6(target, "target")
        position = _vector6(position, "position")
        velocity = _vector6(velocity, "velocity")
        pos_kp = np.array([p.pos_kp for p in self.parameters.arm])
        pos_ki = np.array([p.pos_ki for p in self.parameters.arm])
        velocity_limit = np.array([p.velocity_limit_rad_s for p in self.parameters.arm])
        position_error = target - position
        candidate = np.clip(
            self.position_integral + position_error * dt,
            -self.position_integral_limit,
            self.position_integral_limit,
        )
        raw_velocity = pos_kp * position_error + pos_ki * candidate
        self.target_velocity = np.clip(raw_velocity, -velocity_limit, velocity_limit)
        accept = (raw_velocity == self.target_velocity) | (
            np.sign(position_error) != np.sign(raw_velocity)
        )
        self.position_integral = np.where(accept, candidate, self.position_integral)

        velocity_error = self.target_velocity - velocity
        vel_kp = np.array([p.vel_kp_raw for p in self.parameters.arm])
        vel_ki = np.array([p.vel_ki_raw for p in self.parameters.arm])
        scale = np.array([p.firmware_to_torque_scale for p in self.parameters.arm])
        effort = np.array([p.effort_limit_nm for p in self.parameters.arm])
        vel_candidate = np.clip(
            self.velocity_integral + velocity_error * dt,
            -self.velocity_integral_limit,
            self.velocity_integral_limit,
        )
        raw_torque = (vel_kp * velocity_error + vel_ki * vel_candidate) * scale
        torque = np.clip(raw_torque, -effort, effort)
        accept = (raw_torque == torque) | (np.sign(velocity_error) != np.sign(raw_torque))
        self.velocity_integral = np.where(accept, vel_candidate, self.velocity_integral)
        return torque


class GripperMitController:
    def __init__(self, parameters: MotorControlParameters | GripperParameters):
        self.parameters = (
            parameters.gripper if isinstance(parameters, MotorControlParameters) else parameters
        )

    def compute(
        self,
        target: float,
        position: float,
        velocity: float,
        mode: str = "move",
        feedforward_torque: float = 0.0,
    ) -> GripperCommand:
        p = self.parameters
        gains = {
            "move": (p.move_kp, p.move_kd),
            "closing": (p.closing_kp, p.closing_kd),
            "hold": (p.hold_kp, p.hold_kd),
        }
        if mode not in gains:
            raise ValueError(f"unsupported gripper mode: {mode}")
        kp, kd = gains[mode]
        target_m = float(np.clip(target, p.displacement_min_m, p.displacement_max_m))
        position_m = float(np.clip(position, p.displacement_min_m, p.displacement_max_m))
        ratio = p.motor_radians_per_opening_m
        motor_target = target_m * ratio
        motor_position = position_m * ratio
        motor_velocity = float(velocity) * ratio
        raw_torque = kp * (motor_target - motor_position) - kd * motor_velocity
        raw_torque += float(feedforward_torque)
        motor_torque = float(
            np.clip(raw_torque, -p.motor_torque_limit_nm, p.motor_torque_limit_nm)
        )
        return GripperCommand(
            mode=mode,
            target_displacement_m=target_m,
            motor_target_rad=motor_target,
            kp=kp,
            kd=kd,
            motor_torque_nm=motor_torque,
            opening_force_n=motor_torque * ratio * p.transmission_efficiency,
        )


def _vector6(value, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (6,):
        raise ValueError(f"{name} must have shape (6,), got {result.shape}")
    return result
