from __future__ import annotations

import threading
import time
from contextlib import suppress
from typing import Any, Callable

from rebotarm_motion.replay_runtime_monitor import (
    ReplayRuntimeMonitor,
    ReplayRuntimeMonitorConfig,
)

from .teach_replay_client import TeachReplayClient


class TeachReplaySession:
    """Own the asynchronous lifecycle of one dashboard teach replay action."""

    def __init__(
        self,
        *,
        action_client: Any,
        trajectory_stop_client: Any,
        goal_factory: Callable[[], Any],
        status_sink: Callable[[dict], None],
        status_source: Callable[[], dict],
        replay_client: TeachReplayClient | None = None,
        runtime_monitor: ReplayRuntimeMonitor | None = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._action_client = action_client
        self._trajectory_stop_client = trajectory_stop_client
        self._goal_factory = goal_factory
        self._status_sink = status_sink
        self._status_source = status_source
        self._replay_client = replay_client or TeachReplayClient()
        self._runtime_monitor = runtime_monitor or ReplayRuntimeMonitor()
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._goal_handle = None
        self._trajectory = None
        self._started_at: float | None = None
        self._info_payload: dict = {}
        self._trajectory_points = 0
        self._monitor_config: ReplayRuntimeMonitorConfig | None = None

    def start(
        self,
        trajectory: Any,
        *,
        info_payload: dict,
        monitor_config: ReplayRuntimeMonitorConfig,
        wait_timeout_sec: float = 0.1,
    ) -> dict:
        if not self._action_client.wait_for_server(timeout_sec=wait_timeout_sec):
            return {
                "accepted": False,
                "state": "unavailable",
                "message": "follow_joint_trajectory action unavailable",
            }
        goal = self._goal_factory()
        goal.trajectory = trajectory
        try:
            future = self._action_client.send_goal_async(goal)
        except Exception as exc:
            return {
                "accepted": False,
                "state": "failed",
                "message": f"failed to send teach replay goal: {exc}",
            }
        with self._lock:
            self._info_payload = dict(info_payload)
            self._trajectory_points = len(getattr(trajectory, "points", []))
            self._monitor_config = monitor_config
            self._trajectory = trajectory
        future.add_done_callback(self._on_goal_response)
        return {"accepted": True, "state": "starting"}

    def stop(self) -> dict:
        with self._lock:
            goal_handle = self._goal_handle
        result = self._replay_client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        future = result.pop("cancel_future", None)
        if future is not None:
            future.add_done_callback(self._on_cancel_response)
        return result

    def check_tracking(self, joints: dict) -> None:
        with self._lock:
            goal_handle = self._goal_handle
            trajectory = self._trajectory
            started_at = self._started_at
            config = self._monitor_config
        if goal_handle is None or config is None:
            return
        decision = self._runtime_monitor.check(
            trajectory=trajectory,
            started_at=started_at,
            joints=joints,
            now=self._monotonic(),
            config=config,
        )
        if not decision.should_stop:
            return
        self._request_controller_trajectory_stop(timeout_sec=0.2)
        with suppress(Exception):
            goal_handle.cancel_goal_async()
        self._status_sink(decision.status)

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._status_sink({"state": "failed", "message": str(exc)})
            self._clear()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._status_sink(
                {"state": "rejected", "message": "teach replay goal rejected"}
            )
            self._clear()
            return
        with self._lock:
            self._goal_handle = goal_handle
            info_payload = dict(self._info_payload)
            points = self._trajectory_points
            monitor_config = self._monitor_config
            self._started_at = self._monotonic()
            self._runtime_monitor.reset()
        self._status_sink(
            {
                "state": "replaying",
                "message": "teach replay goal accepted",
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "runtime_monitor": {
                    "enabled": bool(monitor_config.enabled) if monitor_config else False,
                    "max_tracking_error_rad": (
                        float(monitor_config.max_tracking_error_rad) if monitor_config else 0.0
                    ),
                    "max_live_velocity_rad_s": (
                        float(monitor_config.max_live_velocity_rad_s) if monitor_config else 0.0
                    ),
                },
                "dry_run": False,
            }
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_cancel_response(self, future: Any) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._status_sink({"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "teach replay cancel accepted"
            if goals_canceling
            else "teach replay already finished before cancel"
        )
        self._status_sink({"state": state, "message": message})
        if not goals_canceling:
            self._clear()

    def _on_result(self, future: Any) -> None:
        previous_replay = self._status_source()
        with self._lock:
            monitor_stop_requested = self._runtime_monitor.stop_requested
            info_payload = dict(self._info_payload)
            points = self._trajectory_points
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._status_sink({"state": "failed", "message": str(exc)})
            self._clear()
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "safety_stop" if monitor_stop_requested else "canceled"
        else:
            state = "failed"
        message = f"teach replay result status={status}, error_code={error_code}: {error_string}"
        runtime_monitor = previous_replay.get("runtime_monitor") if isinstance(previous_replay, dict) else None
        if status == 5 and monitor_stop_requested:
            previous_message = str(previous_replay.get("message", "")) if isinstance(previous_replay, dict) else ""
            message = (
                f"action canceled after runtime monitor stop: {previous_message}"
                if previous_message
                else "action canceled after runtime monitor stop"
            )
        self._status_sink(
            {
                "state": state,
                "message": message,
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "runtime_monitor": runtime_monitor,
                "dry_run": False,
            }
        )
        self._clear()

    def _clear(self) -> None:
        with self._lock:
            self._goal_handle = None
            self._trajectory = None
            self._started_at = None
            self._info_payload = {}
            self._trajectory_points = 0
            self._monitor_config = None
            self._runtime_monitor.reset()

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        try:
            if not self._trajectory_stop_client.wait_for_service(
                timeout_sec=min(timeout_sec, 0.2)
            ):
                return False
            self._trajectory_stop_client.call_async(self._trigger_request())
            return True
        except Exception:
            return False

    @staticmethod
    def _trigger_request() -> Any:
        from std_srvs.srv import Trigger

        return Trigger.Request()
