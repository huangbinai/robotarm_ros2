from __future__ import annotations

import threading
from typing import Any, Callable

from .web_execute import WebExecuteDecision
from .web_teleop_client import WebTeleopClient


class WebExecuteSession:
    """Own the asynchronous lifecycle of dashboard web trajectory actions."""

    def __init__(
        self,
        *,
        client: WebTeleopClient,
        trajectory_stop_client: Any,
        status_sink: Callable[[dict], None],
        max_joint_speed_source: Callable[[], float],
        successful_result_code: int,
    ) -> None:
        self._client = client
        self._trajectory_stop_client = trajectory_stop_client
        self._status_sink = status_sink
        self._max_joint_speed_source = max_joint_speed_source
        self._successful_result_code = int(successful_result_code)
        self._lock = threading.Lock()
        self._goal_handle = None

    def observe_execution(self, execution: dict) -> None:
        if not execution.get("accepted"):
            return
        future = execution.get("goal_future")
        if future is None:
            return
        decision = execution["decision"]
        future.add_done_callback(
            lambda completed: self._on_goal_response(completed, decision)
        )
        self._status_sink(execution["status"])

    def stop(self) -> dict:
        with self._lock:
            goal_handle = self._goal_handle
        result = self._client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        future = result.get("cancel_future")
        if future is not None:
            future.add_done_callback(self._on_cancel_response)
        self._status_sink(result["status"])
        if result.get("clear_goal_handle"):
            self.clear_goal_handle()
        return {
            "accepted": bool(result["accepted"]),
            "state": result.get("state", result["status"].get("state", "")),
            "message": str(result.get("message", "")),
            "trajectory_stop_requested": bool(
                result.get("trajectory_stop_requested", False)
            ),
        }

    def clear_goal_handle(self) -> None:
        with self._lock:
            self._goal_handle = None

    def _on_cancel_response(self, future: Any) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._status_sink({"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "trajectory cancel accepted"
            if goals_canceling
            else "trajectory already finished before cancel"
        )
        self._status_sink({"state": state, "message": message})

    def _on_goal_response(
        self,
        future: Any,
        decision: WebExecuteDecision,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._status_sink({"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._status_sink(
                {"state": "rejected", "message": "trajectory goal rejected"}
            )
            return
        with self._lock:
            self._goal_handle = goal_handle
        self._status_sink(
            {
                "state": "accepted",
                "message": "trajectory goal accepted by controller",
                **self._decision_status(decision),
            }
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_result(completed, decision)
        )

    def _on_result(self, future: Any, decision: WebExecuteDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            status = int(result_response.status)
            error_code = int(getattr(result, "error_code", 0))
            error_string = str(getattr(result, "error_string", ""))
        except Exception as exc:
            self._status_sink({"state": "failed", "message": str(exc)})
            self.clear_goal_handle()
            return
        if error_code == self._successful_result_code:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        self.clear_goal_handle()
        self._status_sink(
            {
                "state": state,
                "message": (
                    f"trajectory result status={status}, error_code={error_code}: "
                    f"{error_string}"
                ),
                **self._decision_status(decision),
            }
        )

    def _decision_status(self, decision: WebExecuteDecision) -> dict:
        return {
            "max_delta": decision.max_delta,
            "max_delta_limit": decision.max_delta_limit,
            "duration": decision.duration,
            "max_joint_speed_rad_s": float(self._max_joint_speed_source()),
        }
