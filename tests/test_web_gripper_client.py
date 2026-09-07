from __future__ import annotations

from types import SimpleNamespace

from rebotarm_teleop.web_gripper_client import WebGripperClient


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


class _Goal:
    def __init__(self) -> None:
        self.command = SimpleNamespace(position=0.0, max_effort=0.0)


class _ActionClient:
    def __init__(self, *, available=True, goal_future=None) -> None:
        self.available = available
        self.goal_future = goal_future or _Future()
        self.goals = []

    def wait_for_server(self, timeout_sec: float) -> bool:
        self.timeout_sec = timeout_sec
        return self.available

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return self.goal_future


def _client(action_client=None, statuses=None) -> WebGripperClient:
    return WebGripperClient(
        action_client=action_client or _ActionClient(),
        goal_factory=_Goal,
        status_sink=None if statuses is None else statuses.append,
    )


def _set_position(client: WebGripperClient, *, use_hardware=True) -> dict:
    return client.set_position(
        {"confirm": "SET_GRIPPER", "position": 0.03, "max_effort": 0.4},
        use_hardware=use_hardware,
        gripper_limits=(0.0, 0.085),
        default_max_effort=0.3,
        max_effort_limit=1.5,
    )


def test_simulated_position_does_not_send_action_goal() -> None:
    action_client = _ActionClient()
    client = _client(action_client)

    result = _set_position(client, use_hardware=False)

    assert result["accepted"] is True
    assert result["simulated_position"] == 0.03
    assert result["status"]["state"] == "done"
    assert action_client.goals == []


def test_hardware_position_builds_goal_and_reports_unavailable_server() -> None:
    available = _ActionClient()
    accepted = _set_position(_client(available))
    unavailable = _set_position(_client(_ActionClient(available=False)))

    assert accepted["accepted"] is True
    assert available.goals[0].command.position == 0.03
    assert available.goals[0].command.max_effort == 0.4
    assert unavailable["accepted"] is False
    assert unavailable["status"]["state"] == "unavailable"


def test_action_lifecycle_publishes_accepted_and_done_status() -> None:
    statuses = []
    result_future = _Future(
        SimpleNamespace(
            result=SimpleNamespace(
                reached_goal=True,
                position=0.029,
                effort=0.35,
            )
        )
    )
    goal_handle = _GoalHandle(result_future=result_future)
    goal_future = _Future(goal_handle)
    client = _client(_ActionClient(goal_future=goal_future), statuses)
    command = _set_position(client)

    client.observe_result(command)
    goal_future.complete()
    result_future.complete()

    assert statuses == [
        {
            "state": "accepted",
            "message": "gripper goal accepted by controller",
            "position": 0.03,
            "max_effort": 0.4,
        },
        {
            "state": "done",
            "message": "gripper result reached=True",
            "position": 0.029,
            "max_effort": 0.4,
            "effort": 0.35,
        },
    ]


def test_action_lifecycle_reports_goal_and_result_failures() -> None:
    goal_statuses = []
    goal_future = _Future(error=RuntimeError("goal failed"))
    goal_client = _client(_ActionClient(goal_future=goal_future), goal_statuses)
    goal_command = _set_position(goal_client)
    goal_client.observe_result(goal_command)
    goal_future.complete()

    result_statuses = []
    result_future = _Future(error=RuntimeError("result failed"))
    accepted_goal = _Future(_GoalHandle(result_future=result_future))
    result_client = _client(
        _ActionClient(goal_future=accepted_goal),
        result_statuses,
    )
    result_command = _set_position(result_client)
    result_client.observe_result(result_command)
    accepted_goal.complete()
    result_future.complete()

    assert goal_statuses == [{"state": "failed", "message": "goal failed"}]
    assert result_statuses[-1] == {
        "state": "failed",
        "message": "result failed",
    }
