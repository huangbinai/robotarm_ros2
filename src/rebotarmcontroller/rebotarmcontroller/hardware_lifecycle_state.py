from __future__ import annotations

from dataclasses import dataclass
LIFECYCLE_STATES = frozenset(
    {
        "DISCONNECTED",
        "CONNECTED_DISABLED",
        "ENABLING",
        "ENABLED_HOLD",
        "TRAJECTORY_RUNNING",
        "DISABLING",
    }
)

STATE_MACHINE_VALUES = frozenset(
    {
        "IDLE",
        "TRAJ_RUNNING",
        "LOWLEVEL_STREAMING",
        "GRAVITY_COMP",
        "MODE_TRANSITION",
    }
)


@dataclass
class HardwareLifecycleState:
    """Connection and command lifecycle state for the hardware controller."""

    connected: bool = False
    enabled: bool = False
    lifecycle_state: str = "DISCONNECTED"
    state_machine: str = "IDLE"

    @property
    def ready_for_motion(self) -> bool:
        return bool(
            self.connected
            and self.enabled
            and self.lifecycle_state in ("ENABLED_HOLD", "TRAJECTORY_RUNNING")
        )

    def set_state_machine(self, state: str) -> None:
        if state not in STATE_MACHINE_VALUES:
            raise ValueError(f"unsupported state machine value: {state}")
        self.state_machine = state
        lifecycle_can_move = bool(
            self.connected
            and self.enabled
            and self.lifecycle_state not in ("DISABLING", "DISCONNECTED")
        )
        if state == "TRAJ_RUNNING" and lifecycle_can_move:
            self.set_lifecycle_state("TRAJECTORY_RUNNING")
        elif state == "IDLE" and lifecycle_can_move:
            self.set_lifecycle_state("ENABLED_HOLD")

    def set_lifecycle_state(self, state: str) -> None:
        if state not in LIFECYCLE_STATES:
            raise ValueError(f"unsupported lifecycle state: {state}")
        self.lifecycle_state = state

    def require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("hardware is not connected")

    def require_enabled(self) -> None:
        self.require_connected()
        if not self.enabled:
            raise RuntimeError("hardware is disabled; call explicit enable first")
