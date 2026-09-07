from __future__ import annotations

from types import SimpleNamespace

import pytest

from rebotarmcontroller.joint_motor_commands import (
    JointMotorCommand,
    dispatch_joint_motor_command,
    resolve_joint_motor_command,
)


def _raw_command(**overrides):
    values = {
        "mode": 0,
        "pos": 0.2,
        "vel": 0.3,
        "kp": 4.0,
        "kd": 0.8,
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


def test_resolve_joint_motor_command_uses_feedback_and_joint_defaults() -> None:
    command = resolve_joint_motor_command(
        "joint2",
        _raw_command(),
        feedback_state=SimpleNamespace(pos=-0.4, vel=0.1),
        config=SimpleNamespace(kp=5.0, kd=1.0, vlim=0.6),
        position_limits_rad=(-3.14, 0.02),
    )

    assert command == JointMotorCommand(
        mode=0,
        position_rad=-0.4,
        velocity_rad_s=0.1,
        kp=5.0,
        kd=1.0,
        torque_nm=0.0,
        velocity_limit_rad_s=0.6,
    )


def test_resolve_joint_motor_command_reports_all_nonfinite_fields_in_order() -> None:
    with pytest.raises(ValueError) as error:
        resolve_joint_motor_command(
            "joint2",
            _raw_command(
                use_pos=True,
                pos=float("nan"),
                use_tau=True,
                tau=float("inf"),
            ),
            feedback_state=SimpleNamespace(pos=-0.4, vel=0.1),
            config=SimpleNamespace(kp=5.0, kd=1.0, vlim=0.6),
            position_limits_rad=(-3.14, 0.02),
        )

    assert str(error.value).endswith("pos, tau")


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"use_kd": True, "kd": -0.1}, "must be non-negative"),
        ({"use_vlim": True, "vlim": 0.0}, "vlim must be positive"),
        ({"use_pos": True, "pos": 0.021}, "joint2 position command"),
    ),
)
def test_resolve_joint_motor_command_rejects_unsafe_values(
    overrides: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        resolve_joint_motor_command(
            "joint2",
            _raw_command(**overrides),
            feedback_state=SimpleNamespace(pos=-0.4, vel=0.1),
            config=SimpleNamespace(kp=5.0, kd=1.0, vlim=0.6),
            position_limits_rad=(-3.14, 0.02),
        )


@pytest.mark.parametrize(
    ("command", "expected"),
    (
        (
            JointMotorCommand(0, -0.4, 0.2, 5.0, 1.0, 0.3, 0.6),
            ("mit", (-0.4, 0.2, 5.0, 1.0, 0.3)),
        ),
        (
            JointMotorCommand(1, -0.4, 0.2, 5.0, 1.0, 0.3, 0.6),
            ("pos_vel", (-0.4, 0.6)),
        ),
        (
            JointMotorCommand(2, -0.4, 0.2, 5.0, 1.0, 0.3, 0.6),
            ("vel", (0.2,)),
        ),
    ),
)
def test_dispatch_joint_motor_command_preserves_sdk_modes(
    command: JointMotorCommand,
    expected: tuple[str, tuple[float, ...]],
) -> None:
    calls = []
    motor = SimpleNamespace(
        send_mit=lambda *args: calls.append(("mit", args)),
        send_pos_vel=lambda *args: calls.append(("pos_vel", args)),
        send_vel=lambda *args: calls.append(("vel", args)),
    )

    dispatch_joint_motor_command(motor, command, joint_name="joint2")

    assert calls == [expected]


def test_dispatch_joint_velocity_requires_sdk_support() -> None:
    command = JointMotorCommand(2, -0.4, 0.2, 5.0, 1.0, 0.3, 0.6)

    with pytest.raises(RuntimeError, match="joint2 does not support send_vel"):
        dispatch_joint_motor_command(
            SimpleNamespace(),
            command,
            joint_name="joint2",
        )
