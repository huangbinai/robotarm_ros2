from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_loads_firmware_parameters_calibration_and_urdf_effort_limits():
    from rebotarm_simulation.motor_control import load_motor_control_parameters

    parameters = load_motor_control_parameters(ROOT)

    assert parameters.rate_hz == 100.0
    assert parameters.arm[0].pos_kp == 150.0
    assert parameters.arm[0].vel_kp_raw == 0.0125
    assert parameters.arm[3].vel_kp_raw == 0.0008
    assert parameters.arm[0].effort_limit_nm == 27.0
    assert parameters.arm[3].effort_limit_nm == 7.0
    assert parameters.arm[0].firmware_to_torque_scale != 1.0
    assert parameters.gripper.move_kp == 5.0
    assert parameters.gripper.closing_kp == 0.0
    assert parameters.gripper.closing_kd == 0.5
    assert parameters.gripper.hold_kp == 5.0
    assert parameters.gripper.motor_radians_per_opening_m == pytest.approx(-5.0 / 0.09)


def test_pos_vel_is_cascaded_pi_saturated_by_velocity_and_urdf_effort():
    from rebotarm_simulation.motor_control import PosVelController, load_motor_control_parameters

    parameters = load_motor_control_parameters(ROOT)
    controller = PosVelController(parameters)
    torque = controller.compute(
        target=np.ones(6), position=np.zeros(6), velocity=np.zeros(6), dt=0.01
    )

    assert torque.shape == (6,)
    assert np.all(np.abs(controller.target_velocity) <= np.array([5, 5, 5, 3, 3, 3]))
    assert np.all(np.abs(torque) <= np.array([27, 27, 27, 7, 7, 7]))


def test_pos_vel_antiwindup_does_not_accumulate_while_output_is_saturated():
    from rebotarm_simulation.motor_control import PosVelController, load_motor_control_parameters

    controller = PosVelController(load_motor_control_parameters(ROOT))
    for _ in range(1000):
        controller.compute(np.ones(6) * 10, np.zeros(6), np.zeros(6), 0.01)

    assert np.all(np.isfinite(controller.position_integral))
    assert np.all(np.abs(controller.position_integral) <= controller.position_integral_limit)
    assert np.all(np.abs(controller.velocity_integral) <= controller.velocity_integral_limit)


@pytest.mark.parametrize(
    ("mode", "kp", "kd"),
    [("move", 5.0, 1.0), ("closing", 0.0, 0.5), ("hold", 5.0, 1.0)],
)
def test_gripper_uses_hardware_mode_specific_mit_gains(mode, kp, kd):
    from rebotarm_simulation.motor_control import GripperMitController, load_motor_control_parameters

    controller = GripperMitController(load_motor_control_parameters(ROOT))
    result = controller.compute(
        target=0.03,
        position=0.04,
        velocity=0.01,
        mode=mode,
        feedforward_torque=0.4 if mode != "move" else 0.0,
    )

    assert result.kp == kp
    assert result.kd == kd
    assert abs(result.motor_torque_nm) <= 1.5
    assert result.opening_force_n == pytest.approx(
        result.motor_torque_nm
        * controller.parameters.motor_radians_per_opening_m
        * controller.parameters.transmission_efficiency
    )


def test_gripper_clips_displacement_and_feedforward_torque_safely():
    from rebotarm_simulation.motor_control import GripperMitController, load_motor_control_parameters

    controller = GripperMitController(load_motor_control_parameters(ROOT))
    result = controller.compute(
        target=1.0,
        position=0.0,
        velocity=0.0,
        mode="closing",
        feedforward_torque=100.0,
    )

    assert result.target_displacement_m == 0.09
    assert result.motor_torque_nm == 1.5


def test_rejects_unknown_gripper_mode():
    from rebotarm_simulation.motor_control import GripperMitController, load_motor_control_parameters

    controller = GripperMitController(load_motor_control_parameters(ROOT))
    with pytest.raises(ValueError, match="unsupported gripper mode"):
        controller.compute(0.0, 0.0, 0.0, mode="position")
