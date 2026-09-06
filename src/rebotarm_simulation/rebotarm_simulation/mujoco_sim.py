from __future__ import annotations

import importlib
import math
import os
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np

from .model_contract import (
    ARM_JOINT_NAMES,
    FINGER_JOINT_NAMES,
    HOME_KEYFRAME_NAME,
    JOINT_NAMES,
)
from .motor_control import (
    GripperMitController,
    PosVelController,
    load_motor_control_parameters_from_files,
)
from .mujoco_adapters import MujocoKinematicsAdapter, MujocoRenderAdapter
from .mujoco_contact_reader import MujocoContactReader
from .mujoco_model_index import MujocoModelIndex
from .mujoco_scene_runtime import MujocoSceneRuntime
from .mujoco_types import (
    ContactInfo,
    ControlStatus,
    RandomizedScene,
    SavedSimulationState,
    SimulationState,
)
from .sim2real.randomization import RandomizationSample
from .sim_gripper import gripper_joint_positions_for_width
from .simulation_config import SimulationConfig


CONTROL_MODES = ("position", "hold", "gravity_comp", "raw_torque")
_CONTROL_MODE_ALIASES = {"pos_vel": "position"}


def _finite_vector(values: Sequence[float], length: int, label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a numeric sequence") from exc
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


class RebotArmMujoco:
    joint_names = JOINT_NAMES

    def __init__(
        self,
        model_path: str | os.PathLike[str] | None = None,
        *,
        config: SimulationConfig | None = None,
    ) -> None:
        if model_path is not None and config is not None:
            raise ValueError("model_path and config cannot both be supplied")
        self.config = config or SimulationConfig.default(model_path)
        try:
            self._mj = importlib.import_module("mujoco")
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "MuJoCo is required. Install src/rebotarm_simulation/requirements-mujoco.txt "
                "in the active Python environment."
            ) from exc
        self.model_path = str(self.config.model_path)
        self._model = self._mj.MjModel.from_xml_path(self.model_path)
        self._data = self._mj.MjData(self._model)
        self._closed = False
        self._randomization_baseline = {
            "body_mass": np.asarray(self._model.body_mass).copy(),
            "dof_damping": np.asarray(self._model.dof_damping).copy(),
            "geom_friction": np.asarray(self._model.geom_friction).copy(),
        }
        self._randomization_sample: RandomizationSample | None = None
        self._randomization_torque_scale = 1.0
        motor_parameters = load_motor_control_parameters_from_files(
            self.config.arm_config_path,
            self.config.gripper_config_path,
            self.config.motor_calibration_path,
            self.config.robot_urdf_path,
        )
        self._motor_parameters = motor_parameters
        self._arm_controller = PosVelController(motor_parameters.arm)
        self._gripper_controller = GripperMitController(motor_parameters.gripper)
        self._control_steps_per_update = max(
            1, int(round((1.0 / motor_parameters.control_rate_hz) / float(self._model.opt.timestep)))
        )
        self._control_phase = 0
        self._position_targets = np.zeros(len(JOINT_NAMES), dtype=float)
        self._control_mode = "hold"
        self._raw_torque_command = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._raw_torque_requested = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._raw_torque_deadline: float | None = None
        self._requested_arm_torque = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._applied_arm_torque = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._arm_torque_saturated = np.zeros(len(ARM_JOINT_NAMES), dtype=bool)
        self._gripper_max_force_n: float | None = None
        self._gripper_control_force = np.zeros(2, dtype=float)

        self._model_index = MujocoModelIndex(self._mj, self._model)
        self._joint_ids = self._model_index.joint_ids
        self._actuator_ids = self._model_index.actuator_ids
        self._ee_site_id = self._model_index.end_effector_site_id
        self._render_adapter = MujocoRenderAdapter(
            self._model, self._data, self._ensure_open
        )
        self._kinematics_adapter = MujocoKinematicsAdapter(
            self._model, self._data, self._ensure_open
        )
        self._free_bodies = self._model_index.free_bodies
        self._state_spec = self._model_index.state_spec
        self._model_dimensions = self._model_index.model_dimensions
        self._model_fingerprint = self._model_index.model_fingerprint
        self._contact_reader = MujocoContactReader(self._mj, self._model, self._data)
        self._scene_runtime = MujocoSceneRuntime(
            self._mj, self._model, self._data, self._free_bodies
        )
        self.reset()

    @property
    def timestep(self) -> float:
        self._ensure_open()
        return float(self._model.opt.timestep)

    @property
    def render_adapter(self) -> MujocoRenderAdapter:
        """Return the explicit MuJoCo rendering capability."""
        self._ensure_open()
        return self._render_adapter

    @property
    def kinematics_adapter(self) -> MujocoKinematicsAdapter:
        """Return the explicit MuJoCo FK/Jacobian capability."""
        self._ensure_open()
        return self._kinematics_adapter

    @property
    def control_targets(self) -> tuple[float, ...]:
        self._ensure_open()
        return tuple(float(value) for value in self._position_targets)

    @property
    def control_mode(self) -> str:
        self._ensure_open()
        return self._control_mode

    @property
    def randomization_sample(self) -> RandomizationSample | None:
        self._ensure_open()
        return self._randomization_sample

    @property
    def arm_joint_limits(self) -> tuple[tuple[float, float], ...]:
        """Return the six arm joint ranges declared by the loaded MJCF."""
        self._ensure_open()
        return tuple(
            tuple(float(value) for value in self._model.jnt_range[joint_id])
            for joint_id in self._joint_ids[:6]
        )

    @property
    def arm_actuator_force_limits(self) -> tuple[float, ...]:
        """Return symmetric arm actuator limits used for safety validation."""
        self._ensure_open()
        limits = []
        for actuator_id in self._actuator_ids[:6]:
            if int(self._model.actuator_ctrllimited[actuator_id]):
                lower, upper = self._model.actuator_ctrlrange[actuator_id]
                limits.append(max(abs(float(lower)), abs(float(upper))))
            else:
                limits.append(math.inf)
        return tuple(limits)

    def randomization_session(self, sample: RandomizationSample):
        from .sim2real.randomization import RandomizationSession

        return RandomizationSession(self, sample)

    def apply_randomization(self, sample: RandomizationSample) -> None:
        self._ensure_open()
        if not isinstance(sample, RandomizationSample):
            raise TypeError("sample must be a RandomizationSample")
        self.restore_randomization()
        self._model.body_mass[:] = self._randomization_baseline["body_mass"] * sample.mass_scale
        self._model.dof_damping[:] = self._randomization_baseline["dof_damping"] * sample.damping_scale
        self._model.geom_friction[:] = self._randomization_baseline["geom_friction"] * sample.friction_scale
        self._randomization_torque_scale = float(sample.torque_scale)
        self._randomization_sample = sample
        self._mj.mj_forward(self._model, self._data)

    def restore_randomization(self) -> None:
        self._ensure_open()
        self._model.body_mass[:] = self._randomization_baseline["body_mass"]
        self._model.dof_damping[:] = self._randomization_baseline["dof_damping"]
        self._model.geom_friction[:] = self._randomization_baseline["geom_friction"]
        self._randomization_torque_scale = 1.0
        self._randomization_sample = None
        self._mj.mj_forward(self._model, self._data)

    def set_mode(self, mode: str) -> str:
        self._ensure_open()
        mode = _CONTROL_MODE_ALIASES.get(str(mode), str(mode))
        if mode not in CONTROL_MODES:
            raise ValueError(f"control mode must be one of {CONTROL_MODES}")
        # Re-entering Hold must be a true no-op. Re-capturing the measured
        # position and resetting the simulated firmware controller on every
        # keyboard-repeat event creates a small target/torque discontinuity
        # that is visible as a shake even though the requested mode did not
        # change.
        if mode == "hold" and self._control_mode == "hold":
            return self._control_mode
        if mode == "hold":
            self._sync_arm_targets_to_current_position()
        if mode != "raw_torque":
            self._raw_torque_command.fill(0.0)
            self._raw_torque_requested.fill(0.0)
            self._raw_torque_deadline = None
        elif self._raw_torque_deadline is None:
            self._raw_torque_command.fill(0.0)
            self._raw_torque_requested.fill(0.0)
            self._raw_torque_deadline = float(self._data.time) + 0.1
        if mode in ("gravity_comp", "hold"):
            self._arm_controller.reset()
        self._control_mode = mode
        self._apply_motor_control()
        return self._control_mode

    def set_control_mode(self, mode: str) -> str:
        """Compatibility alias for :meth:`set_mode`.

        ``pos_vel`` is accepted as a legacy spelling and normalized to
        ``position``.
        """
        return self.set_mode(mode)

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("MuJoCo simulation is closed")

    def reset(self, seed: int | None = None) -> SimulationState:
        self._ensure_open()
        self._scene_runtime.reset_random_generator(seed)
        home_key = self._mj.mj_name2id(
            self._model, self._mj.mjtObj.mjOBJ_KEY, HOME_KEYFRAME_NAME
        )
        if home_key >= 0:
            self._mj.mj_resetDataKeyframe(self._model, self._data, home_key)
        else:
            self._mj.mj_resetData(self._model, self._data)
        return self._finish_reset()

    def reset_home(self, seed: int | None = None) -> SimulationState:
        return self.reset(seed=seed)

    def _finish_reset(self) -> SimulationState:
        for index, joint_id in enumerate(self._joint_ids):
            qpos_address = int(self._model.jnt_qposadr[joint_id])
            self._position_targets[index] = self._data.qpos[qpos_address]
        self._data.ctrl[:] = 0.0
        self._arm_controller.reset()
        self._control_mode = "hold"
        self._raw_torque_command.fill(0.0)
        self._raw_torque_requested.fill(0.0)
        self._raw_torque_deadline = None
        self._requested_arm_torque.fill(0.0)
        self._applied_arm_torque.fill(0.0)
        self._arm_torque_saturated.fill(False)
        self._gripper_max_force_n = None
        self._gripper_control_force.fill(0.0)
        self._control_phase = 0
        self._mj.mj_forward(self._model, self._data)
        self._seed_arm_torque_from_gravity()
        self._apply_motor_control()
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def command_joint_positions(
        self, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        self._ensure_open()
        current = list(self.control_targets[: len(ARM_JOINT_NAMES)])
        if isinstance(targets, Mapping):
            unknown = set(targets) - set(ARM_JOINT_NAMES)
            if unknown:
                raise ValueError(f"Unknown arm joint names: {sorted(unknown)}")
            updates = {name: float(value) for name, value in targets.items()}
            if not all(math.isfinite(value) for value in updates.values()):
                raise ValueError("Joint targets must be finite")
            for name, value in updates.items():
                current[ARM_JOINT_NAMES.index(name)] = value
        else:
            current = list(_finite_vector(targets, len(ARM_JOINT_NAMES), "joint targets"))

        reached = []
        for index, joint_id in enumerate(self._joint_ids[:6]):
            lower, upper = (float(value) for value in self._model.jnt_range[joint_id])
            value = min(max(current[index], lower), upper)
            self._position_targets[index] = value
            reached.append(value)
        self.set_mode("position")
        return tuple(reached)

    def set_joint_position_targets(
        self, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        return self.command_joint_positions(targets)

    def command_joint_torques(
        self, torques: Sequence[float], timeout_s: float = 0.1
    ) -> tuple[float, ...]:
        self._ensure_open()
        requested = np.asarray(
            _finite_vector(torques, len(ARM_JOINT_NAMES), "joint torques"), dtype=float
        )
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        effort = np.asarray(self._motor_parameters.arm.effort_limit, dtype=float)
        self._raw_torque_requested[:] = requested
        self._raw_torque_command[:] = np.clip(requested, -effort, effort)
        self._raw_torque_deadline = float(self._data.time) + timeout
        self._arm_controller.reset()
        self._control_mode = "raw_torque"
        self._apply_motor_control()
        return tuple(float(value) for value in self._raw_torque_command)

    def command_gripper_width(self, width_m: float, max_force_n: float | None = None) -> float:
        self._ensure_open()
        value = float(width_m)
        if not math.isfinite(value):
            raise ValueError("Gripper width must be finite")
        if max_force_n is not None:
            max_force_n = float(max_force_n)
            if not math.isfinite(max_force_n) or max_force_n <= 0.0:
                raise ValueError("max_force_n must be finite and positive")
            max_force_n = min(max_force_n, self._motor_parameters.gripper.finger_force_limit_n)
        self._gripper_max_force_n = max_force_n
        left, right, reached = gripper_joint_positions_for_width(value)
        self._position_targets[-2] = left
        self._position_targets[-1] = right
        return reached

    def set_gripper_width(self, width: float) -> float:
        return self.command_gripper_width(width)

    def mirror_joint_state(
        self,
        positions: Sequence[float],
        velocities: Sequence[float] | None = None,
        *,
        gripper_width: float | None = None,
    ) -> SimulationState:
        """Kinematically synchronize arm state while preserving free objects."""
        self._ensure_open()
        position_values = _finite_vector(positions, len(ARM_JOINT_NAMES), "joint positions")
        velocity_values = (
            (0.0,) * len(ARM_JOINT_NAMES)
            if velocities is None
            else _finite_vector(velocities, len(ARM_JOINT_NAMES), "joint velocities")
        )
        for index, (joint_id, position, velocity) in enumerate(
            zip(self._joint_ids[:6], position_values, velocity_values)
        ):
            lower, upper = (float(value) for value in self._model.jnt_range[joint_id])
            if position < lower - 1e-6 or position > upper + 1e-6:
                raise ValueError(
                    f"joint position {ARM_JOINT_NAMES[index]}={position} is outside [{lower}, {upper}]"
                )
            qpos_address = int(self._model.jnt_qposadr[joint_id])
            qvel_address = int(self._model.jnt_dofadr[joint_id])
            self._data.qpos[qpos_address] = min(max(position, lower), upper)
            self._data.qvel[qvel_address] = velocity
            self._position_targets[index] = self._data.qpos[qpos_address]
        if gripper_width is not None:
            left, right, _reached = gripper_joint_positions_for_width(gripper_width)
            for index, value in zip((-2, -1), (left, right)):
                joint_id = self._joint_ids[index]
                self._data.qpos[int(self._model.jnt_qposadr[joint_id])] = value
                self._data.qvel[int(self._model.jnt_dofadr[joint_id])] = 0.0
                self._position_targets[index] = value
        self._arm_controller.reset()
        self._control_phase = 0
        self._mj.mj_forward(self._model, self._data)
        self._seed_arm_torque_from_gravity()
        self._apply_motor_control()
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def step(self, n_steps: int = 1) -> SimulationState:
        self._ensure_open()
        if isinstance(n_steps, bool) or not isinstance(n_steps, int):
            raise TypeError("n_steps must be a positive integer")
        if n_steps <= 0:
            raise ValueError("n_steps must be a positive integer")
        for _ in range(n_steps):
            if self._raw_torque_watchdog_expired():
                self._expire_raw_torque()
                self._control_phase = 0
            if self._control_phase == 0:
                self._apply_motor_control()
            self._mj.mj_step(self._model, self._data)
            self._control_phase = (self._control_phase + 1) % self._control_steps_per_update
            if self._raw_torque_watchdog_expired():
                # Make the safe Hold command observable immediately when a
                # multi-step call lands exactly on the deadline.
                self._expire_raw_torque()
                self._control_phase = 0
                self._apply_motor_control()
        # mj_step integrates qpos after its position stage; refresh derived
        # kinematics so the returned pose describes the new qpos, not the
        # beginning of the final step.
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def _apply_motor_control(self) -> None:
        qpos_addresses = [int(self._model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids[:6]]
        qvel_addresses = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        position = np.asarray([self._data.qpos[address] for address in qpos_addresses], dtype=float)
        velocity = np.asarray([self._data.qvel[address] for address in qvel_addresses], dtype=float)
        gravity = self._gravity_compensation_torque(qvel_addresses)
        if self._control_mode == "raw_torque":
            requested_torque = self._raw_torque_requested.copy()
            arm_torque = self._raw_torque_command.copy()
            self._arm_controller.applied_torque[:] = arm_torque
        elif self._control_mode == "gravity_comp":
            requested_torque = gravity
            arm_torque = requested_torque.copy()
            self._arm_controller.applied_torque[:] = arm_torque
        elif self._control_mode == "hold":
            # Hold is position regulation around the pose captured on entry.
            # It deliberately shares the simulated firmware loop with
            # position mode; the semantic difference is who owns the target.
            arm_torque = self._arm_controller.compute(
                target=self._position_targets[:6],
                position=position,
                velocity=velocity,
                dt=1.0 / self._motor_parameters.control_rate_hz,
                feedforward=gravity,
            )
            requested_torque = arm_torque.copy()
        else:
            arm_torque = self._arm_controller.compute(
                target=self._position_targets[:6],
                position=position,
                velocity=velocity,
                dt=1.0 / self._motor_parameters.control_rate_hz,
                feedforward=gravity,
            )
            requested_torque = arm_torque.copy()
        requested_torque = np.asarray(requested_torque, dtype=float)
        scaled_torque = np.asarray(arm_torque, dtype=float) * self._randomization_torque_scale
        applied_torque = np.asarray(
            [
                self._clamp_actuator_control(actuator_id, torque)
                for actuator_id, torque in zip(self._actuator_ids[:6], scaled_torque)
            ],
            dtype=float,
        )
        self._requested_arm_torque[:] = requested_torque
        self._applied_arm_torque[:] = applied_torque
        self._arm_torque_saturated[:] = ~np.isclose(requested_torque, applied_torque, atol=1e-12, rtol=0.0)
        for actuator_id, torque in zip(self._actuator_ids[:6], applied_torque):
            self._data.ctrl[actuator_id] = torque

        left_qpos = float(self._data.qpos[int(self._model.jnt_qposadr[self._joint_ids[-2]])])
        right_qpos = float(self._data.qpos[int(self._model.jnt_qposadr[self._joint_ids[-1]])])
        left_qvel = float(self._data.qvel[int(self._model.jnt_dofadr[self._joint_ids[-2]])])
        right_qvel = float(self._data.qvel[int(self._model.jnt_dofadr[self._joint_ids[-1]])])
        command = self._gripper_controller.compute(
            target=float(self._position_targets[-2] - self._position_targets[-1]),
            position=left_qpos - right_qpos,
            velocity=left_qvel - right_qvel,
            mode="move",
        )
        finger_force = self._stable_gripper_force(
            target_width=command.target_displacement_m,
            current_width=left_qpos - right_qpos,
            current_velocity=left_qvel - right_qvel,
        )
        if self._gripper_max_force_n is not None:
            finger_force = float(np.clip(
                finger_force, -self._gripper_max_force_n, self._gripper_max_force_n
            ))
        left_force = self._clamp_actuator_control(self._actuator_ids[-2], finger_force)
        right_force = self._clamp_actuator_control(self._actuator_ids[-1], -finger_force)
        self._gripper_control_force[:] = (left_force, right_force)
        self._data.ctrl[self._actuator_ids[-2]] = left_force
        self._data.ctrl[self._actuator_ids[-1]] = right_force

    def _expire_raw_torque(self) -> None:
        self._raw_torque_command.fill(0.0)
        self._raw_torque_requested.fill(0.0)
        self._raw_torque_deadline = None
        self._sync_arm_targets_to_current_position()
        self._arm_controller.reset()
        self._control_mode = "hold"

    def _raw_torque_watchdog_expired(self) -> bool:
        return (
            self._control_mode == "raw_torque"
            and self._raw_torque_deadline is not None
            and float(self._data.time) >= self._raw_torque_deadline
        )

    def _stable_gripper_force(
        self,
        *,
        target_width: float,
        current_width: float,
        current_velocity: float,
    ) -> float:
        gripper = self._motor_parameters.gripper
        error = float(target_width) - float(current_width)
        velocity = float(current_velocity)
        if (
            abs(error) < gripper.sim_force_deadband_m
            and abs(velocity) < gripper.sim_velocity_deadband_m_s
        ):
            return 0.0
        # MuJoCo force actuators act directly on the sliding finger joints in N.
        # The real MIT command still defines the motor-side torque limit, while
        # this linear-space PD keeps the simulated prismatic joints stable.
        return (
            gripper.sim_force_kp_n_per_m * error
            - gripper.sim_force_kd_n_s_per_m * velocity
        )

    def _clamp_actuator_control(self, actuator_id: int, value: float) -> float:
        if int(self._model.actuator_ctrllimited[actuator_id]):
            lower, upper = (float(v) for v in self._model.actuator_ctrlrange[actuator_id])
            return min(max(float(value), lower), upper)
        return float(value)

    def _seed_arm_torque_from_gravity(self) -> None:
        qvel_addresses = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        self._arm_controller.applied_torque[:] = self._gravity_compensation_torque(qvel_addresses)

    def _sync_arm_targets_to_current_position(self) -> None:
        for index, joint_id in enumerate(self._joint_ids[:6]):
            self._position_targets[index] = float(self._data.qpos[int(self._model.jnt_qposadr[joint_id])])

    def _gravity_compensation_torque(self, qvel_addresses: Sequence[int]) -> np.ndarray:
        scale = float(self._motor_parameters.arm.gravity_compensation_scale)
        if scale == 0.0:
            return np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        return np.asarray([self._data.qfrc_bias[address] for address in qvel_addresses], dtype=float) * scale

    def get_state(self) -> SimulationState:
        self._ensure_open()
        positions = []
        velocities = []
        for joint_id in self._joint_ids:
            positions.append(float(self._data.qpos[int(self._model.jnt_qposadr[joint_id])]))
            velocities.append(float(self._data.qvel[int(self._model.jnt_dofadr[joint_id])]))
        object_poses = self._scene_runtime.object_poses()
        ee_quaternion_wxyz = np.empty(4, dtype=float)
        self._mj.mju_mat2Quat(
            ee_quaternion_wxyz, self._data.site_xmat[self._ee_site_id]
        )
        return SimulationState(
            joint_names=JOINT_NAMES,
            joint_positions=tuple(positions),
            joint_velocities=tuple(velocities),
            actuator_forces=tuple(float(self._data.actuator_force[index]) for index in self._actuator_ids),
            end_effector_position=tuple(float(value) for value in self._data.site_xpos[self._ee_site_id]),
            end_effector_orientation=(
                *(float(value) for value in ee_quaternion_wxyz[1:]),
                float(ee_quaternion_wxyz[0]),
            ),
            gripper_width=max(0.0, min(0.09, positions[-2] - positions[-1])),
            object_poses=MappingProxyType(object_poses),
            simulation_time=float(self._data.time),
        )

    def get_control_status(self) -> ControlStatus:
        """Return an immutable diagnostic snapshot without changing control state."""
        self._ensure_open()
        state = self.get_state()
        remaining = None
        if self._control_mode == "raw_torque" and self._raw_torque_deadline is not None:
            remaining = max(0.0, self._raw_torque_deadline - state.simulation_time)
        return ControlStatus(
            mode=self._control_mode,
            joint_targets=tuple(float(value) for value in self._position_targets[:6]),
            joint_positions=state.joint_positions[:6],
            joint_velocities=state.joint_velocities[:6],
            requested_torques=tuple(float(value) for value in self._requested_arm_torque),
            applied_torques=tuple(float(value) for value in self._applied_arm_torque),
            saturated=tuple(bool(value) for value in self._arm_torque_saturated),
            watchdog_remaining_s=remaining,
            gripper_target_width_m=float(self._position_targets[-2] - self._position_targets[-1]),
            gripper_width_m=state.gripper_width,
            gripper_control_force_n=tuple(float(value) for value in self._gripper_control_force),
        )

    def get_contacts(self) -> tuple[ContactInfo, ...]:
        self._ensure_open()
        return self._contact_reader.read()

    def save_state(self) -> SavedSimulationState:
        self._ensure_open()
        state = np.empty(self._model_dimensions[-1], dtype=float)
        self._mj.mj_getState(self._model, self._data, state, self._state_spec)
        return SavedSimulationState(
            model_identity=id(self._model),
            model_fingerprint=self._model_fingerprint,
            model_dimensions=self._model_dimensions,
            state_spec=self._state_spec,
            state=tuple(float(value) for value in state),
            control_targets=tuple(float(value) for value in self._position_targets),
            position_integral=tuple(float(value) for value in self._arm_controller.position_integral),
            velocity_integral=tuple(float(value) for value in self._arm_controller.velocity_integral),
            applied_torque=tuple(float(value) for value in self._arm_controller.applied_torque),
            control_phase=self._control_phase,
            control_mode=self._control_mode,
            raw_torque_command=tuple(float(value) for value in self._raw_torque_command),
            raw_torque_requested=tuple(float(value) for value in self._raw_torque_requested),
            raw_torque_deadline=self._raw_torque_deadline,
            gripper_max_force_n=self._gripper_max_force_n,
        )

    def restore_state(self, state: SavedSimulationState) -> SimulationState:
        self._ensure_open()
        if not isinstance(state, SavedSimulationState):
            raise TypeError("state must be returned by save_state()")
        compatible = (
            state.model_identity == id(self._model)
            and state.model_fingerprint == self._model_fingerprint
            and state.model_dimensions == self._model_dimensions
            and state.state_spec == self._state_spec
            and len(state.state) == self._model_dimensions[-1]
            and len(state.control_targets) == len(JOINT_NAMES)
            and len(state.position_integral) == len(ARM_JOINT_NAMES)
            and len(state.velocity_integral) == len(ARM_JOINT_NAMES)
            and len(state.applied_torque) == len(ARM_JOINT_NAMES)
            and state.control_mode in CONTROL_MODES
            and len(state.raw_torque_command) == len(ARM_JOINT_NAMES)
            and len(state.raw_torque_requested) == len(ARM_JOINT_NAMES)
        )
        if not compatible:
            raise ValueError("saved state must belong to the same MuJoCo model instance")
        self._mj.mj_setState(
            self._model, self._data, np.asarray(state.state, dtype=float), self._state_spec
        )
        self._position_targets[:] = np.asarray(state.control_targets, dtype=float)
        self._arm_controller.position_integral[:] = np.asarray(state.position_integral, dtype=float)
        self._arm_controller.velocity_integral[:] = np.asarray(state.velocity_integral, dtype=float)
        self._arm_controller.applied_torque[:] = np.asarray(state.applied_torque, dtype=float)
        self._control_phase = int(state.control_phase) % self._control_steps_per_update
        self._control_mode = state.control_mode
        self._raw_torque_command[:] = np.asarray(state.raw_torque_command, dtype=float)
        self._raw_torque_requested[:] = np.asarray(state.raw_torque_requested, dtype=float)
        self._raw_torque_deadline = state.raw_torque_deadline
        self._gripper_max_force_n = state.gripper_max_force_n
        self._applied_arm_torque[:] = np.asarray(
            [self._data.ctrl[actuator_id] for actuator_id in self._actuator_ids[:6]],
            dtype=float,
        )
        if self._control_mode == "raw_torque":
            self._requested_arm_torque[:] = self._raw_torque_requested
            self._arm_torque_saturated[:] = ~np.isclose(
                self._requested_arm_torque,
                self._applied_arm_torque,
                atol=1e-12,
                rtol=0.0,
            )
        else:
            self._requested_arm_torque[:] = self._applied_arm_torque
            self._arm_torque_saturated.fill(False)
        self._gripper_control_force[:] = tuple(
            float(self._data.ctrl[actuator_id]) for actuator_id in self._actuator_ids[-2:]
        )
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def set_object_pose(
        self,
        body_name: str,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        zero_velocity: bool = True,
    ) -> tuple[float, ...]:
        self._ensure_open()
        return self._scene_runtime.set_object_pose(
            body_name,
            position,
            orientation,
            zero_velocity=zero_velocity,
        )

    def randomize_scene(
        self,
        seed: int | None = None,
        *,
        cube_xy_bounds: Sequence[Sequence[float]] = ((0.22, 0.38), (-0.14, 0.14)),
        cube_z: float = 0.04,
        reach_target_bounds: Sequence[Sequence[float]] = (
            (0.18, 0.45),
            (-0.22, 0.22),
            (0.08, 0.35),
        ),
    ) -> RandomizedScene:
        self._ensure_open()
        return self._scene_runtime.randomize(
            seed,
            cube_xy_bounds=cube_xy_bounds,
            cube_z=cube_z,
            reach_target_bounds=reach_target_bounds,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._data = None
        self._model = None

    def __enter__(self) -> "RebotArmMujoco":
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
