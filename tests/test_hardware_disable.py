from __future__ import annotations

from rebotarmcontroller.hardware_disable import (
    attempt_verified_disable,
    disable_unique_controller_buses,
)


def test_verified_disable_preserves_command_and_feedback_order() -> None:
    events = []

    attempt = attempt_verified_disable(
        stop_control_loop=lambda: events.append("stop"),
        disable_all_motors=lambda: events.append("disable"),
        verify_disabled_feedback=lambda: events.append("verify"),
    )

    assert attempt.verified is True
    assert attempt.error_detail == ""
    assert events == ["stop", "disable", "verify"]


def test_verified_disable_continues_commands_and_skips_unsafe_verification() -> None:
    events = []

    def fail_stop() -> None:
        events.append("stop")
        raise RuntimeError("loop failure")

    def fail_disable() -> None:
        events.append("disable")
        raise RuntimeError("bus failure")

    attempt = attempt_verified_disable(
        stop_control_loop=fail_stop,
        disable_all_motors=fail_disable,
        verify_disabled_feedback=lambda: events.append("verify"),
    )

    assert attempt.verified is False
    assert attempt.errors == (
        "stop control loop: loop failure",
        "disable command: bus failure",
    )
    assert attempt.error_detail == (
        "stop control loop: loop failure; disable command: bus failure"
    )
    assert events == ["stop", "disable"]


def test_verified_disable_reports_feedback_failure_after_commands() -> None:
    events = []

    def fail_feedback() -> None:
        events.append("verify")
        raise RuntimeError("stale status")

    attempt = attempt_verified_disable(
        stop_control_loop=lambda: events.append("stop"),
        disable_all_motors=lambda: events.append("disable"),
        verify_disabled_feedback=fail_feedback,
    )

    assert attempt.errors == ("disabled feedback: stale status",)
    assert events == ["stop", "disable", "verify"]


def test_controller_bus_disable_deduplicates_by_identity_and_continues() -> None:
    events = []

    class Controller:
        def __init__(self, name: str, *, fails: bool = False) -> None:
            self.name = name
            self.fails = fails

        def disable_all(self) -> None:
            events.append(self.name)
            if self.fails:
                raise RuntimeError(f"{self.name} failed")

    first = Controller("first")
    failing = Controller("failing", fails=True)
    last = Controller("last")

    errors = disable_unique_controller_buses(
        [first, first, failing, last, failing]
    )

    assert events == ["first", "failing", "last"]
    assert errors == ("Controller: failing failed",)
