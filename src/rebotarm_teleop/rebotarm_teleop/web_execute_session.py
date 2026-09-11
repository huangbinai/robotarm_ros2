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
        self._lock = threading.RLock()
        self._goal_handle = None
        self._generation = 0
        self._terminal = False
        self._stop_pending = False

    def observe_execution(self, execution: dict) -> None:
        if not execution.get("accepted"):
            return
        future = execution.get("goal_future")
        if future is None:
            return
        decision = execution["decision"]
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._terminal = False
            self._stop_pending = False
            self._goal_handle = None
            self._status_sink(execution["status"])
        future.add_done_callback(
            lambda completed: self._on_goal_response(completed, decision, generation)
        )

    def _publish(self, status, generation, *, terminal=False):
        with self._lock:
            if generation != self._generation or self._terminal:
                return
            if terminal:
                self._terminal = True
                self._goal_handle = None
            self._status_sink(status)

    def stop(self) -> dict:
        with self._lock:
            goal_handle = self._goal_handle
            generation = self._generation
            self._stop_pending = True
        result = self._client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        future = result.get("cancel_future")
        self._publish(result["status"], generation)
        if future is not None:
            future.add_done_callback(lambda completed: self._on_cancel_response(completed, generation))
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
            self._generation += 1
            self._terminal = True

    def _on_cancel_response(self, future: Any, generation: int) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._publish({"state": "cancel_unconfirmed", "message": str(exc)}, generation)
            return
        state = "cancel_requested" if goals_canceling else "cancel_unconfirmed"
        message = (
            "trajectory cancel accepted"
            if goals_canceling
            else "cancel response did not confirm cancellation; waiting for trajectory result"
        )
        self._publish({"state": state, "message": message}, generation)

    def _on_goal_response(
        self,
        future: Any,
        decision: WebExecuteDecision,
        generation: int,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish({"state": "failed", "message": str(exc)}, generation, terminal=True)
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish({"state": "rejected", "message": "trajectory goal rejected"}, generation, terminal=True)
            return
        with self._lock:
            if generation != self._generation:
                return
            self._goal_handle = goal_handle
            stop_pending = self._stop_pending
        self._publish(
            {
                "state": "accepted",
                "message": "trajectory goal accepted by controller",
                **self._decision_status(decision),
            }, generation,
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_result(completed, decision, generation)
        )
        if stop_pending:
            self.stop()

    def _on_result(self, future: Any, decision: WebExecuteDecision, generation: int) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            status = int(result_response.status)
            error_code = int(getattr(result, "error_code", 0))
            error_string = str(getattr(result, "error_string", ""))
        except Exception as exc:
            self._publish({"state": "failed", "message": str(exc)}, generation, terminal=True)
            return
        if status == 5:
            state = "canceled"
        elif (
            status == 6 and self._stop_pending and error_code == -1
            and error_string == "simulation trajectory stopped by service"
        ):
            # Service stop can win the race with Action cancellation. Only the
            # backend's explicit stop result confirms this state, not the RPC ack.
            state = "stopped"
        elif status == 4 and error_code == self._successful_result_code:
            state = "done"
        else:
            state = "failed"
        self._publish(
            {
                "state": state,
                "message": (
                    f"trajectory result status={status}, error_code={error_code}: "
                    f"{error_string}"
                ),
                **self._decision_status(decision),
            }, generation, terminal=True,
        )

    def _decision_status(self, decision: WebExecuteDecision) -> dict:
        return {
            "max_delta": decision.max_delta,
            "max_delta_limit": decision.max_delta_limit,
            "duration": decision.duration,
            "max_joint_speed_rad_s": float(self._max_joint_speed_source()),
        }
