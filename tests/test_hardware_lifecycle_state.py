from __future__ import annotations

import pytest

from rebotarmcontroller.hardware_lifecycle_state import (
    HardwareLifecycleField,
    HardwareLifecycleState,
)


@pytest.mark.parametrize(
    ("connected", "enabled", "lifecycle_state", "expected"),
    [
        (False, False, "DISCONNECTED", False),
        (True, False, "CONNECTED_DISABLED", False),
        (True, True, "ENABLING", False),
        (True, True, "ENABLED_HOLD", True),
        (True, True, "TRAJECTORY_RUNNING", True),
        (True, True, "DISABLING", False),
    ],
)
def test_ready_for_motion_requires_enabled_motion_lifecycle(
    connected: bool,
    enabled: bool,
    lifecycle_state: str,
    expected: bool,
) -> None:
    state = HardwareLifecycleState(
        connected=connected,
        enabled=enabled,
        lifecycle_state=lifecycle_state,
    )

    assert state.ready_for_motion is expected


def test_state_machine_updates_motion_lifecycle_without_overwriting_disabling() -> None:
    state = HardwareLifecycleState(
        connected=True,
        enabled=True,
        lifecycle_state="ENABLED_HOLD",
    )

    state.set_state_machine("TRAJ_RUNNING")
    assert state.lifecycle_state == "TRAJECTORY_RUNNING"

    state.set_state_machine("IDLE")
    assert state.lifecycle_state == "ENABLED_HOLD"

    state.lifecycle_state = "DISABLING"
    state.set_state_machine("IDLE")
    assert state.lifecycle_state == "DISABLING"


def test_lifecycle_and_state_machine_reject_unknown_values() -> None:
    state = HardwareLifecycleState()

    with pytest.raises(ValueError, match="unsupported lifecycle state"):
        state.set_lifecycle_state("UNKNOWN")
    with pytest.raises(ValueError, match="unsupported state machine value"):
        state.set_state_machine("UNKNOWN")


def test_connection_and_enable_guards_keep_existing_messages() -> None:
    disconnected = HardwareLifecycleState()
    disabled = HardwareLifecycleState(
        connected=True,
        lifecycle_state="CONNECTED_DISABLED",
    )

    with pytest.raises(RuntimeError, match="hardware is not connected"):
        disconnected.require_connected()
    with pytest.raises(RuntimeError, match="explicit enable"):
        disabled.require_enabled()


def test_legacy_private_field_descriptor_uses_lifecycle_state() -> None:
    class Owner:
        _enabled = HardwareLifecycleField("enabled")

    owner = Owner()
    owner._enabled = True

    assert owner._enabled is True
    assert owner._lifecycle.enabled is True
