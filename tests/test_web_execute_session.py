from __future__ import annotations

from types import SimpleNamespace

from rebotarm_teleop.web_execute import WebExecuteDecision
from rebotarm_teleop.web_execute_session import WebExecuteSession


class _Future:
    def __init__(self, result=None, *, error: Exception | None = None) -> None:
        self._result = result
        self._error = error
        self.callbacks = []

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def add_done_callback(self, callback) -> None:
        self.callbacks.append(callback)

    def complete(self) -> None:
        for callback in tuple(self.callbacks):
            callback(self)


class _GoalHandle:
    def __init__(self, *, accepted=True, result_future=None) -> None:
        self.accepted = accepted
        self.result_future = result_future or _Future()

    def get_result_async(self):
        return self.result_future


class _Client:
    def __init__(self, stop_result=None) -> None:
        self.stop_result = stop_result
        self.stop_calls = []

    def stop(self, goal_handle, *, trajectory_stop_client):
        self.stop_calls.append((goal_handle, trajectory_stop_client))
        return self.stop_result


def _decision() -> WebExecuteDecision:
    return WebExecuteDecision(
        accepted=True,
        message="accepted",
        joint_names=("joint1",),
        positions=(0.2,),
        duration=1.0,
        max_delta=0.2,
        max_delta_limit=1.0,
    )


def _session(client=None, statuses=None, stop_client=None) -> WebExecuteSession:
    statuses = [] if statuses is None else statuses
    return WebExecuteSession(
        client=client or _Client(),
        trajectory_stop_client=stop_client or object(),
        status_sink=statuses.append,
        max_joint_speed_source=lambda: 1.5,
        successful_result_code=0,
    )


def test_execution_lifecycle_publishes_active_accepted_and_done() -> None:
    statuses = []
    action_result = _Future(
        SimpleNamespace(
            status=4,
            result=SimpleNamespace(error_code=0, error_string=""),
        )
    )
    goal_future = _Future(_GoalHandle(result_future=action_result))
    session = _session(statuses=statuses)
    execution = {
        "accepted": True,
        "decision": _decision(),
        "goal_future": goal_future,
        "status": {"state": "active", "message": "accepted"},
    }

    session.observe_execution(execution)
    goal_future.complete()
    action_result.complete()

    assert [status["state"] for status in statuses] == [
        "active",
        "accepted",
        "done",
    ]
    assert statuses[-1]["max_joint_speed_rad_s"] == 1.5


def test_rejected_goal_and_result_exception_publish_failure_status() -> None:
    rejected_statuses = []
    rejected = _Future(_GoalHandle(accepted=False))
    rejected_session = _session(statuses=rejected_statuses)
    rejected_session.observe_execution(
        {
            "accepted": True,
            "decision": _decision(),
            "goal_future": rejected,
            "status": {"state": "active"},
        }
    )
    rejected.complete()

    failed_statuses = []
    failed_result = _Future(error=RuntimeError("result failed"))
    accepted = _Future(_GoalHandle(result_future=failed_result))
    failed_session = _session(statuses=failed_statuses)
    failed_session.observe_execution(
        {
            "accepted": True,
            "decision": _decision(),
            "goal_future": accepted,
            "status": {"state": "active"},
        }
    )
    accepted.complete()
    failed_result.complete()

    assert rejected_statuses[-1] == {
        "state": "rejected",
        "message": "trajectory goal rejected",
    }
    assert failed_statuses[-1] == {
        "state": "failed",
        "message": "result failed",
    }


def test_stop_uses_active_goal_and_observes_cancel_response() -> None:
    statuses = []
    cancel_future = _Future(SimpleNamespace(goals_canceling=[object()]))
    stop_result = {
        "accepted": True,
        "message": "trajectory cancel requested",
        "trajectory_stop_requested": True,
        "status": {"state": "cancel_requested", "message": "requested"},
        "cancel_future": cancel_future,
        "clear_goal_handle": True,
    }
    client = _Client(stop_result)
    stop_client = object()
    session = _session(client=client, statuses=statuses, stop_client=stop_client)
    goal_handle = _GoalHandle()
    goal_future = _Future(goal_handle)
    session.observe_execution(
        {
            "accepted": True,
            "decision": _decision(),
            "goal_future": goal_future,
            "status": {"state": "active"},
        }
    )
    goal_future.complete()

    response = session.stop()
    cancel_future.complete()

    assert client.stop_calls == [(goal_handle, stop_client)]
    assert response == {
        "accepted": True,
        "state": "cancel_requested",
        "message": "trajectory cancel requested",
        "trajectory_stop_requested": True,
    }
    assert statuses[-1] == {
        "state": "cancel_requested",
        "message": "trajectory cancel accepted",
    }
