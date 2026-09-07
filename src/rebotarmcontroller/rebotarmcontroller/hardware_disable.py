from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass


@dataclass(frozen=True)
class VerifiedDisableAttempt:
    """Result of a best-effort disable followed by feedback verification."""

    errors: tuple[str, ...] = ()

    @property
    def verified(self) -> bool:
        return not self.errors

    @property
    def error_detail(self) -> str:
        return "; ".join(self.errors)


def attempt_verified_disable(
    *,
    stop_control_loop: Callable[[], None],
    disable_all_motors: Callable[[], None],
    verify_disabled_feedback: Callable[[], None],
) -> VerifiedDisableAttempt:
    """Try every safe command stage and verify only after commands succeed."""

    errors: list[str] = []
    try:
        stop_control_loop()
    except Exception as exc:
        errors.append(f"stop control loop: {exc}")
    try:
        disable_all_motors()
    except Exception as exc:
        errors.append(f"disable command: {exc}")
    if not errors:
        try:
            verify_disabled_feedback()
        except Exception as exc:
            errors.append(f"disabled feedback: {exc}")
    return VerifiedDisableAttempt(tuple(errors))


def disable_unique_controller_buses(controllers: Iterable[object]) -> tuple[str, ...]:
    """Disable each physical controller object once, continuing after failures."""

    unique_controllers: list[object] = []
    for controller in controllers:
        if all(controller is not existing for existing in unique_controllers):
            unique_controllers.append(controller)

    errors: list[str] = []
    for controller in unique_controllers:
        try:
            controller.disable_all()
        except Exception as exc:
            errors.append(f"{type(controller).__name__}: {exc}")
    return tuple(errors)
