from __future__ import annotations

from rebotarm_teleop.web_keyboard_client import (
    WebKeyboardClient,
    keyboard_decision_response,
)


class _Duration:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class _TrajectoryPoint:
    def __init__(self) -> None:
        self.positions = []
        self.time_from_start = _Duration()


class _Trajectory:
    def __init__(self) -> None:
        self.joint_names = []
        self.points = []


class _Goal:
    def __init__(self) -> None:
        self.trajectory = None


class _Future:
    def add_done_callback(self, callback) -> None:
        self.callback = callback


class _ActionClient:
    def __init__(self, *, available: bool = True) -> None:
        self.available = available
        self.goals = []

    def wait_for_server(self, timeout_sec: float) -> bool:
        self.timeout_sec = timeout_sec
        return self.available

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _Future()


def _client(action_client=None) -> WebKeyboardClient:
    return WebKeyboardClient(
        action_client=action_client or _ActionClient(),
        joint_names=("joint1", "joint2"),
        joint_limits={"joint1": (-1.0, 1.0), "joint2": (-1.0, 1.0)},
        joint_velocity_limits={"joint1": 2.0, "joint2": 2.0},
        trajectory_factory=_Trajectory,
        trajectory_point_factory=_TrajectoryPoint,
        follow_goal_factory=_Goal,
        default_step_rad=0.02,
        default_duration=0.2,
        default_speed_rad_s=0.5,
    )


def _enable(client: WebKeyboardClient) -> dict:
    return client.enable(
        {},
        min_step_rad=0.005,
        max_step_rad=0.1,
        min_duration=0.1,
        max_duration=2.0,
        max_speed_rad_s=2.0,
    )


def _prepare(client: WebKeyboardClient):
    return client.prepare_command(
        {"confirm": "KEYBOARD_TELEOP", "key": "1"},
        current_positions={"joint1": 0.1, "joint2": -0.1},
        default_step_rad=0.02,
        min_step_rad=0.005,
        max_step_rad=0.1,
        default_duration=0.2,
        min_duration=0.1,
        max_duration=2.0,
        max_speed_rad_s=2.0,
    )


def test_enable_clamps_settings_and_prepare_uses_them() -> None:
    client = _client()

    enabled = client.enable(
        {"step_rad": 1.0, "duration": 0.01, "max_joint_speed_rad_s": 5.0},
        min_step_rad=0.005,
        max_step_rad=0.1,
        min_duration=0.1,
        max_duration=2.0,
        max_speed_rad_s=2.0,
    )
    request, decision = _prepare(client)

    assert enabled["step_rad"] == 0.1
    assert enabled["duration"] == 0.1
    assert enabled["max_joint_speed_rad_s"] == 2.0
    assert request["step_rad"] == 0.1
    assert request["duration"] == 0.1
    assert decision.accepted is True


def test_disabled_client_rejects_command_before_dispatch() -> None:
    client = _client()

    _request, decision = _prepare(client)

    assert decision.accepted is False
    assert "disabled" in decision.message


def test_send_builds_single_point_keyboard_trajectory() -> None:
    action_client = _ActionClient()
    client = _client(action_client)
    _enable(client)
    _request, decision = _prepare(client)

    result = client.send(decision)

    assert result["accepted"] is True
    assert len(action_client.goals) == 1
    trajectory = action_client.goals[0].trajectory
    assert trajectory.joint_names == ["joint1", "joint2"]
    assert trajectory.points[0].positions == [0.12000000000000001, -0.1]
    assert trajectory.points[0].time_from_start.sec == 0
    assert trajectory.points[0].time_from_start.nanosec == 200_000_000
    assert keyboard_decision_response(decision)["key"] == "1"


def test_send_reports_unavailable_action_server_without_goal() -> None:
    action_client = _ActionClient(available=False)
    client = _client(action_client)
    _enable(client)
    _request, decision = _prepare(client)

    result = client.send(decision)

    assert result == {
        "accepted": False,
        "message": "follow_joint_trajectory action unavailable",
        "goal_future": None,
    }
    assert action_client.goals == []
