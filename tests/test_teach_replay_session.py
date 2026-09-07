from __future__ import annotations

from rebotarm_motion.replay_runtime_monitor import ReplayRuntimeMonitorConfig
from rebotarm_teach.teach_replay_session import TeachReplaySession


class _Future:
    def __init__(self) -> None:
        self._callbacks = []
        self._result = None

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)

    def resolve(self, result) -> None:
        self._result = result
        for callback in list(self._callbacks):
            callback(self)

    def result(self):
        return self._result


class _Duration:
    sec = 0
    nanosec = 0


class _Point:
    def __init__(self, position: float) -> None:
        self.positions = [position]
        self.time_from_start = _Duration()


class _Trajectory:
    def __init__(self) -> None:
        self.joint_names = ["joint1"]
        self.points = [_Point(0.0)]


class _Goal:
    def __init__(self) -> None:
        self.trajectory = None


class _ActionClient:
    def __init__(self, *, available: bool = True) -> None:
        self.available = available
        self.goal_future = _Future()
        self.goals = []

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self.available

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return self.goal_future


class _StopFuture:
    def done(self) -> bool:
        return True

    def result(self):
        return type("Response", (), {"success": True, "message": "stopped"})()


class _StopClient:
    def __init__(self) -> None:
        self.calls = 0

    def wait_for_service(self, timeout_sec: float) -> bool:
        return True

    def call_async(self, request):
        self.calls += 1
        return _StopFuture()


class _GoalHandle:
    def __init__(self, *, accepted: bool = True) -> None:
        self.accepted = accepted
        self.result_future = _Future()
        self.cancel_future = _Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


def _monitor_config() -> ReplayRuntimeMonitorConfig:
    return ReplayRuntimeMonitorConfig(
        enabled=True,
        start_grace_sec=0.0,
        violation_grace_sec=0.2,
        max_tracking_error_rad=0.1,
        max_live_velocity_rad_s=3.0,
    )


def _session(*, available: bool = True, times=None):
    statuses = []
    action_client = _ActionClient(available=available)
    stop_client = _StopClient()
    clock_values = iter(times or [10.0])
    session = TeachReplaySession(
        action_client=action_client,
        trajectory_stop_client=stop_client,
        goal_factory=_Goal,
        status_sink=statuses.append,
        status_source=lambda: statuses[-1] if statuses else {},
        monotonic=lambda: next(clock_values),
    )
    return session, action_client, stop_client, statuses


def test_session_rejects_start_when_action_server_is_unavailable() -> None:
    session, action_client, _, statuses = _session(available=False)

    result = session.start(
        _Trajectory(),
        info_payload={"path": "record.jsonl"},
        monitor_config=_monitor_config(),
    )

    assert result["accepted"] is False
    assert result["state"] == "unavailable"
    assert action_client.goals == []
    assert statuses == []


def test_session_owns_goal_acceptance_and_success_result_status() -> None:
    session, action_client, _, statuses = _session()
    trajectory = _Trajectory()
    goal_handle = _GoalHandle()

    result = session.start(
        trajectory,
        info_payload={"path": "record.jsonl", "start_band": "direct", "max_error": 0.01},
        monitor_config=_monitor_config(),
    )
    action_client.goal_future.resolve(goal_handle)
    goal_handle.result_future.resolve(
        type(
            "WrappedResult",
            (),
            {"status": 4, "result": type("Result", (), {"error_code": 0, "error_string": ""})()},
        )()
    )

    assert result == {"accepted": True, "state": "starting"}
    assert action_client.goals[0].trajectory is trajectory
    assert statuses[0]["state"] == "replaying"
    assert statuses[-1]["state"] == "done"
    assert statuses[-1]["record_path"] == "record.jsonl"


def test_session_preserves_runtime_safety_stop_reason_in_action_result() -> None:
    session, action_client, stop_client, statuses = _session(times=[10.0, 11.0, 11.3])
    goal_handle = _GoalHandle()
    session.start(
        _Trajectory(),
        info_payload={"path": "record.jsonl", "start_band": "direct"},
        monitor_config=_monitor_config(),
    )
    action_client.goal_future.resolve(goal_handle)

    joints = {"joint1": {"position": 1.0, "velocity": 0.0}}
    session.check_tracking(joints)
    session.check_tracking(joints)
    goal_handle.result_future.resolve(
        type(
            "WrappedResult",
            (),
            {"status": 5, "result": type("Result", (), {"error_code": 0, "error_string": ""})()},
        )()
    )

    assert stop_client.calls == 1
    assert goal_handle.cancel_calls == 1
    assert statuses[-2]["state"] == "safety_stop"
    assert statuses[-1]["state"] == "safety_stop"
    assert "action canceled after runtime monitor stop" in statuses[-1]["message"]


def test_session_stop_owns_cancel_callback_and_clears_finished_goal() -> None:
    session, action_client, stop_client, statuses = _session()
    goal_handle = _GoalHandle()
    session.start(
        _Trajectory(),
        info_payload={"path": "record.jsonl"},
        monitor_config=_monitor_config(),
    )
    action_client.goal_future.resolve(goal_handle)

    result = session.stop()
    goal_handle.cancel_future.resolve(
        type("CancelResponse", (), {"goals_canceling": []})()
    )

    assert result["accepted"] is True
    assert stop_client.calls == 1
    assert statuses[-1] == {
        "state": "done",
        "message": "teach replay already finished before cancel",
    }
