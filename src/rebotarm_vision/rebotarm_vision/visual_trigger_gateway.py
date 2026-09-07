from __future__ import annotations

from collections.abc import Callable
from typing import Any


class VisualTriggerGateway:
    """Call and request Trigger services used by visual grasp execution."""

    def __init__(
        self,
        *,
        request_factory: Callable[[], Any],
        wait_for_future: Callable[[Any, float], bool],
        monotonic: Callable[[], float],
        sleep: Callable[[float], None],
        ok: Callable[[], bool],
        warn: Callable[[str], None],
    ) -> None:
        self._request_factory = request_factory
        self._wait_for_future = wait_for_future
        self._monotonic = monotonic
        self._sleep = sleep
        self._ok = ok
        self._warn = warn

    def call(
        self,
        client: Any,
        label: str,
        *,
        availability_timeout_sec: float,
        response_timeout_sec: float,
        interruptible: bool,
    ) -> tuple[bool, str]:
        try:
            if not client.wait_for_service(timeout_sec=availability_timeout_sec):
                return False, f"{label} service unavailable"
            future = client.call_async(self._request_factory())
            if interruptible:
                completed = self._wait_for_future(future, response_timeout_sec)
            else:
                completed = self._wait_non_interruptible(
                    future,
                    response_timeout_sec,
                )
            if not completed:
                return False, f"{label} service call timed out"
            result = future.result()
            if result is None:
                return False, f"{label} returned no result"
            return bool(result.success), str(result.message)
        except Exception as exc:
            return False, f"{type(exc).__name__}: {exc}"

    def request(self, client: Any, label: str, *, timeout_sec: float = 0.2) -> None:
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return
            client.call_async(self._request_factory())
        except Exception as exc:
            self._warn(f"failed to request {label}: {exc}")

    def _wait_non_interruptible(self, future: Any, timeout_sec: float) -> bool:
        deadline = self._monotonic() + max(0.0, float(timeout_sec))
        while self._ok() and not future.done() and self._monotonic() < deadline:
            self._sleep(0.02)
        return bool(future.done())
