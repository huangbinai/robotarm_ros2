from __future__ import annotations

from types import SimpleNamespace

import pytest

from rebotarm_vision.visual_grasp_sequence import PoseTarget, VisualGraspStage
from rebotarm_vision.visual_motion_gateway import VisualMotionGateway


class _Future:
    def __init__(self, result) -> None:
        self._result = result

    def result(self):
        return self._result


class _Client:
    def __init__(self, *, available: bool = True, result=None) -> None:
        self.available = available
        self.result = result
        self.requests = []
        self.wait_timeouts = []

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        self.wait_timeouts.append(timeout_sec)
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return _Future(self.result)


def _stage() -> VisualGraspStage:
    return VisualGraspStage(
        name="approach_grasp",
        kind="move",
        pose=PoseTarget(
            position=(0.3, 0.1, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
        ),
    )


def _gateway(client, *, wait_result: bool = True):
    waits = []

    def wait_for_future(future, timeout):
        waits.append((future, timeout))
        return wait_result

    gateway = VisualMotionGateway(
        client=client,
        request_factory=SimpleNamespace,
        target_message_factory=lambda pose, frame: (pose, frame),
        wait_for_future=wait_for_future,
        target_frame="base_link",
        service_timeout_sec=20.0,
        motion_result_timeout_sec=45.0,
        acceleration_scaling=lambda: 0.08,
    )
    return gateway, waits


def test_builds_and_sends_execute_pose_request() -> None:
    result = SimpleNamespace(success=True, stage="execute", message="ok")
    client = _Client(result=result)
    gateway, waits = _gateway(client)

    outcome = gateway.send(_stage(), execute=True, velocity_scaling=0.04)

    assert outcome == (True, "execute: ok")
    request = client.requests[0]
    assert request.target_pose[1] == "base_link"
    assert request.velocity_scaling == pytest.approx(0.04)
    assert request.acceleration_scaling == pytest.approx(0.08)
    assert request.timeout_sec == pytest.approx(45.0)
    assert request.execute is True
    assert waits == [(waits[0][0], 65.0)]


def test_reports_missing_pose_timeout_and_missing_result() -> None:
    client = _Client(result=None)
    gateway, _waits = _gateway(client)
    missing = VisualGraspStage(name="move", kind="move")

    assert gateway.send(missing, execute=False, velocity_scaling=0.1) == (
        False,
        "missing move pose",
    )
    assert gateway.send(_stage(), execute=False, velocity_scaling=0.1) == (
        False,
        "motion execution returned no result",
    )

    timeout_gateway, _ = _gateway(client, wait_result=False)
    assert timeout_gateway.send(_stage(), execute=False, velocity_scaling=0.1) == (
        False,
        "motion execution service call timed out",
    )


def test_service_availability_uses_service_timeout() -> None:
    client = _Client(available=False)
    gateway, _waits = _gateway(client)

    assert gateway.service_available() is False
    assert client.wait_timeouts == [20.0]
