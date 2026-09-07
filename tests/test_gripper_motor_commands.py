from __future__ import annotations

from types import SimpleNamespace

import pytest

from rebotarmcontroller.gripper_motor_commands import (
    GripperMotorCommand,
    dispatch_gripper_motor_command,
    resolve_gripper_motor_command,
    send_safe_gripper_mit,
)


def _raw_command(**overrides):
    values = {
        "mode": 0,
        "pos": -2.0,
        "vel": 0.2,
        "kp": 3.0,
        "kd": 0.4,
        "tau": 0.5,
        "vlim": 1.0,
        "use_pos": False,
        "use_vel": False,
        "use_kp": False,
        "use_kd": False,
        "use_tau": False,
        "use_vlim": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_resolve_gripper_motor_command_uses_feedback_and_config_defaults() -> None:
    command = resolve_gripper_motor_command(
        _raw_command(),
        feedback_state=SimpleNamespace(pos=-1.2, vel=0.3),
        config=SimpleNamespace(kp=4.0, kd=0.8, vlim=0.6),
        open_soft_limit_rad=-4.9,
        torque_limit_nm=1.5,
    )

    assert command == GripperMotorCommand(
        mode=0,
        position_rad=-1.2,
        velocity_rad_s=0.3,
        kp=4.0,
        kd=0.8,
        torque_nm=0.0,
        velocity_limit_rad_s=0.6,
    )


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"use_pos": True, "pos": float("nan")}, "must be finite"),
        ({"use_kp": True, "kp": -0.1}, "must be non-negative"),
        ({"use_vlim": True, "vlim": 0.0}, "vlim must be positive"),
        ({"use_pos": True, "pos": -5.0}, "outside calibrated range"),
        ({"use_tau": True, "tau": 1.6}, "torque command exceeds"),
    ),
)
def test_resolve_gripper_motor_command_rejects_unsafe_values(
    overrides: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        resolve_gripper_motor_command(
            _raw_command(**overrides),
            feedback_state=SimpleNamespace(pos=-1.0, vel=0.0),
            config=SimpleNamespace(kp=4.0, kd=0.8, vlim=0.6),
            open_soft_limit_rad=-4.9,
            torque_limit_nm=1.5,
        )


@pytest.mark.parametrize(
    ("command", "expected"),
    (
        (
            GripperMotorCommand(0, -1.0, 0.2, 3.0, 0.4, 0.5, 0.8),
            ("mit", (-1.0, 0.2, 3.0, 0.4, 0.5)),
        ),
        (
            GripperMotorCommand(1, -1.0, 0.2, 3.0, 0.4, 0.5, 0.8),
            ("pos_vel", (-1.0, 0.8)),
        ),
        (
            GripperMotorCommand(2, -1.0, 0.2, 3.0, 0.4, 0.5, 0.8),
            ("vel", (0.2,)),
        ),
    ),
)
def test_dispatch_gripper_motor_command_preserves_sdk_modes(
    command: GripperMotorCommand,
    expected: tuple[str, tuple[float, ...]],
) -> None:
    calls = []
    motor = SimpleNamespace(
        send_mit=lambda *args: calls.append(("mit", args)),
        send_pos_vel=lambda *args: calls.append(("pos_vel", args)),
        send_vel=lambda *args: calls.append(("vel", args)),
    )

    dispatch_gripper_motor_command(motor, command)

    assert calls == [expected]


def test_safe_gripper_mit_clamps_position_and_total_torque() -> None:
    calls = []
    motor = SimpleNamespace(send_mit=lambda *args: calls.append(args))

    send_safe_gripper_mit(
        motor,
        position_rad=-6.0,
        velocity_rad_s=0.0,
        kp=5.0,
        kd=1.0,
        torque_feedforward_nm=1.0,
        torque_limit_nm=0.4,
        current_position_rad=-4.8,
        current_velocity_rad_s=0.0,
        open_soft_limit_rad=-4.9,
        maximum_torque_nm=1.5,
    )

    position, velocity, kp, kd, feedforward = calls[0]
    position_term = kp * (position - (-4.8)) + kd * -0.0
    assert position == -4.9
    assert velocity == 0.0
    assert position_term + feedforward == pytest.approx(0.4)
