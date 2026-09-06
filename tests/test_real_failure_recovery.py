from __future__ import annotations

from types import SimpleNamespace

from rebotarm_motion.real_failure_recovery import recover_real_failure


def _status(*, healthy: bool = True):
    return SimpleNamespace(
        enabled=True,
        control_loop_active=True,
        per_joint_status_code=[1] * 6 if healthy else [1, 1, 1, 1, 1, 2],
        error_codes=[] if healthy else ["feedback fault"],
    )


class _Node:
    def __init__(self, *, healthy=True, return_success=True) -> None:
        self.latest_status = _status(healthy=healthy)
        self.stop_client = "stop"
        self.disable_client = "disable"
        self.return_success = return_success
        self.calls = []

    def call_trigger(self, client, label):
        self.calls.append((client, label))
        if client == self.disable_client:
            self.latest_status.enabled = False
            self.latest_status.control_loop_active = False
        return {"label": label, "success": True}

    def hold_and_collect(self, _duration):
        return None

    def _status_payload(self):
        return {"enabled": self.latest_status.enabled}

    def canonical_positions(self):
        return [0.1] * 6

    def execute_leg(self, command):
        self.calls.append(("execute", command["label"]))
        return {"success": self.return_success, "result": "failed"}


def test_recoverable_failure_keeps_healthy_arm_enabled_when_return_is_disabled() -> None:
    node = _Node()
    report = {}
    assert recover_real_failure(
        node=node,
        report=report,
        baseline=[0.0] * 6,
        allow_controlled_return=False,
        legs_key="legs",
        command_label="return",
    )
    assert ("disable", "disable") not in node.calls
    assert report["failure_recovery"]["outcome"] == (
        "healthy_enabled_hold_requires_operator_recovery"
    )


def test_critical_feedback_requests_protective_disable() -> None:
    node = _Node(healthy=False)
    report = {}
    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=[0.0] * 6,
        allow_controlled_return=True,
        legs_key="legs",
        command_label="return",
    )
    assert still_enabled is False
    assert ("disable", "disable") in node.calls


def test_failed_controlled_return_keeps_healthy_hold() -> None:
    node = _Node(return_success=False)
    report = {}
    assert recover_real_failure(
        node=node,
        report=report,
        baseline=[0.0] * 6,
        allow_controlled_return=True,
        legs_key="legs",
        command_label="return",
        duration_sec=1.0,
    )
    assert ("disable", "disable") not in node.calls
    assert report["failure_recovery"]["outcome"] == "return_failed_healthy_enabled_hold"
