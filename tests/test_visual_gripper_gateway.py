from __future__ import annotations

from types import SimpleNamespace

import pytest

from rebotarm_vision.visual_gripper_gateway import VisualGripperGateway


class _Future:
    def __init__(self, result) -> None:
        self._result = result

    def result(self):
        return self._result


class _Client:
    def __init__(self, result=None, *, available: bool = True) -> None:
        self.result = result
        self.available = available
        self.requests = []
        self.wait_timeouts = []

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        self.wait_timeouts.append(timeout_sec)
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return _Future(self.result)


def _gateway(position_client, grasp_client, *, wait_result=True):
    waits = []

    def wait_for_future(future, timeout):
        waits.append((future, timeout))
        return wait_result

    gateway = VisualGripperGateway(
        position_client=position_client,
        grasp_client=grasp_client,
        position_request_factory=SimpleNamespace,
        grasp_request_factory=SimpleNamespace,
        wait_for_future=wait_for_future,
        service_timeout_sec=20.0,
    )
    return gateway, waits


def test_sends_position_request_and_normalizes_result() -> None:
    position_client = _Client(
        SimpleNamespace(success=True, reached_position=0.081)
    )
    gateway, waits = _gateway(position_client, _Client())

    result, error = gateway.set_position(position_m=0.085, max_effort=0.4)

    assert error == ""
    assert result is not None
    assert result.success is True
    assert result.reached_position == pytest.approx(0.081)
    request = position_client.requests[0]
    assert request.position == pytest.approx(0.085)
    assert request.max_effort == pytest.approx(0.4)
    assert waits[0][1] == pytest.approx(20.0)


def test_sends_grasp_request_with_original_timeout_and_force_clamping() -> None:
    grasp_client = _Client(
        SimpleNamespace(
            success=True,
            contact_detected=True,
            contact_position=0.04,
            reached_position=0.039,
            hold_force=0.4,
            message="contact detected",
        )
    )
    gateway, waits = _gateway(_Client(), grasp_client)

    result, error = gateway.grasp(
        close_force=-1.0,
        hold_force=-2.0,
        close_timeout_sec=8.0,
        min_close_time_sec=0.08,
        velocity_threshold=0.04,
        min_closure_distance_m=0.006,
    )

    assert error == ""
    assert result is not None and result.contact_detected is True
    request = grasp_client.requests[0]
    assert request.close_force == 0.0
    assert request.hold_force == 0.0
    assert request.close_timeout_sec == pytest.approx(8.0)
    assert waits[0][1] == pytest.approx(28.0)


@pytest.mark.parametrize(
    ("available", "wait_result", "expected"),
    [
        (False, True, "gripper service unavailable"),
        (True, False, "gripper service call timed out"),
    ],
)
def test_position_service_failures_are_reported(available, wait_result, expected) -> None:
    client = _Client(available=available)
    gateway, _waits = _gateway(client, _Client(), wait_result=wait_result)

    result, error = gateway.set_position(position_m=0.04, max_effort=0.3)

    assert result is None
    assert error == expected


def test_missing_grasp_result_is_reported() -> None:
    gateway, _waits = _gateway(_Client(), _Client(result=None))

    result, error = gateway.grasp(
        close_force=0.4,
        hold_force=0.4,
        close_timeout_sec=2.0,
        min_close_time_sec=0.08,
        velocity_threshold=0.04,
        min_closure_distance_m=0.006,
    )

    assert result is None
    assert error == "gripper grasp service returned no result"
