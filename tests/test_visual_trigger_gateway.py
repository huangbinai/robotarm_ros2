from __future__ import annotations

from types import SimpleNamespace

from rebotarm_vision.visual_trigger_gateway import VisualTriggerGateway


class _Future:
    def __init__(self, result, *, done: bool = True) -> None:
        self._result = result
        self._done = done

    def done(self) -> bool:
        return self._done

    def result(self):
        return self._result


class _Client:
    def __init__(self, future=None, *, available=True, error=None) -> None:
        self.future = future
        self.available = available
        self.error = error
        self.requests = []
        self.wait_timeouts = []

    def wait_for_service(self, *, timeout_sec):
        if self.error is not None:
            raise self.error
        self.wait_timeouts.append(timeout_sec)
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def _gateway(*, interruptible_result=True, warnings=None):
    warnings = [] if warnings is None else warnings
    clock = iter((0.0, 0.0, 1.0, 2.0, 3.0))
    return VisualTriggerGateway(
        request_factory=SimpleNamespace,
        wait_for_future=lambda _future, _timeout: interruptible_result,
        monotonic=lambda: next(clock),
        sleep=lambda _seconds: None,
        ok=lambda: True,
        warn=warnings.append,
    )


def test_interruptible_trigger_call_uses_injected_waiter() -> None:
    client = _Client(_Future(SimpleNamespace(success=True, message="ok")))
    gateway = _gateway()

    result = gateway.call(
        client,
        "visual_ready plan",
        availability_timeout_sec=20.0,
        response_timeout_sec=65.0,
        interruptible=True,
    )

    assert result == (True, "ok")
    assert client.wait_timeouts == [20.0]
    assert len(client.requests) == 1


def test_non_interruptible_call_waits_independently_of_execution_state() -> None:
    client = _Client(_Future(SimpleNamespace(success=False, message="rejected")))
    gateway = _gateway(interruptible_result=False)

    result = gateway.call(
        client,
        "protective disable",
        availability_timeout_sec=1.0,
        response_timeout_sec=1.0,
        interruptible=False,
    )

    assert result == (False, "rejected")


def test_trigger_call_reports_unavailable_timeout_and_exception() -> None:
    unavailable = _gateway().call(
        _Client(available=False),
        "safe_home",
        availability_timeout_sec=2.0,
        response_timeout_sec=3.0,
        interruptible=True,
    )
    timeout = _gateway(interruptible_result=False).call(
        _Client(_Future(None, done=False)),
        "safe_home",
        availability_timeout_sec=2.0,
        response_timeout_sec=3.0,
        interruptible=True,
    )
    failed = _gateway().call(
        _Client(error=RuntimeError("serial unavailable")),
        "safe_home",
        availability_timeout_sec=2.0,
        response_timeout_sec=3.0,
        interruptible=True,
    )

    assert unavailable == (False, "safe_home service unavailable")
    assert timeout == (False, "safe_home service call timed out")
    assert failed == (False, "RuntimeError: serial unavailable")


def test_fire_and_forget_request_warns_on_exception() -> None:
    warnings = []
    gateway = _gateway(warnings=warnings)

    gateway.request(
        _Client(error=RuntimeError("not ready")),
        "trajectory_stop",
    )

    assert warnings == ["failed to request trajectory_stop: not ready"]
