from __future__ import annotations

from typing import Any, Callable

from .web_execute import WebGripperDecision, validate_web_gripper_request


def gripper_decision_response(decision: WebGripperDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "position": decision.position,
        "max_effort": decision.max_effort,
    }


class WebGripperClient:
    """Validate web gripper commands and own their action result lifecycle."""

    def __init__(
        self,
        *,
        action_client: Any | None,
        goal_factory: Callable[[], Any] | None,
        status_sink: Callable[[dict], None] | None = None,
    ) -> None:
        self._action_client = action_client
        self._goal_factory = goal_factory
        self._status_sink = status_sink

    def set_position(
        self,
        payload: dict,
        *,
        use_hardware: bool,
        gripper_limits: tuple[float, float],
        default_max_effort: float,
        max_effort_limit: float,
    ) -> dict:
        decision = validate_web_gripper_request(
            payload,
            gripper_limits=gripper_limits,
            default_max_effort=default_max_effort,
            max_effort_limit=max_effort_limit,
        )
        if not decision.accepted:
            return self._result(
                accepted=False,
                decision=decision,
                status={"state": "rejected", "message": decision.message},
            )
        if not use_hardware:
            return self._result(
                accepted=True,
                decision=decision,
                status={
                    "state": "done",
                    "message": (
                        f"simulated gripper position={decision.position:.4f} m"
                    ),
                    "position": decision.position,
                    "max_effort": decision.max_effort,
                    "simulated": True,
                },
                simulated_position=float(decision.position),
            )
        if self._action_client is None or self._goal_factory is None:
            return self._unavailable_result(decision)
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            return self._unavailable_result(decision)

        goal = self._goal_factory()
        goal.command.position = float(decision.position)
        goal.command.max_effort = float(decision.max_effort)
        return self._result(
            accepted=True,
            decision=decision,
            status={
                "state": "active",
                "message": decision.message,
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
            goal_future=self._action_client.send_goal_async(goal),
        )

    def observe_result(self, result: dict) -> None:
        if self._status_sink is None or not result.get("accepted"):
            return
        future = result.get("goal_future")
        if future is None:
            return
        decision = result["decision"]
        future.add_done_callback(
            lambda completed: self._on_goal_response(completed, decision)
        )

    def _on_goal_response(
        self,
        future: Any,
        decision: WebGripperDecision,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status({"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status(
                {"state": "rejected", "message": "gripper goal rejected"}
            )
            return
        self._publish_status(
            {
                "state": "accepted",
                "message": "gripper goal accepted by controller",
                "position": decision.position,
                "max_effort": decision.max_effort,
            }
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_result(completed, decision)
        )

    def _on_result(self, future: Any, decision: WebGripperDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            reached = bool(getattr(result, "reached_goal", False))
            position = float(getattr(result, "position", decision.position))
            effort = float(getattr(result, "effort", 0.0))
        except Exception as exc:
            self._publish_status({"state": "failed", "message": str(exc)})
            return
        self._publish_status(
            {
                "state": "done" if reached else "failed",
                "message": f"gripper result reached={reached}",
                "position": position,
                "max_effort": decision.max_effort,
                "effort": effort,
            }
        )

    def _publish_status(self, status: dict) -> None:
        if self._status_sink is not None:
            self._status_sink(status)

    @staticmethod
    def _result(
        *,
        accepted: bool,
        decision: WebGripperDecision,
        status: dict,
        goal_future: Any | None = None,
        simulated_position: float | None = None,
    ) -> dict:
        return {
            "accepted": accepted,
            "decision": decision,
            "response": gripper_decision_response(decision),
            "status": status,
            "goal_future": goal_future,
            "simulated_position": simulated_position,
        }

    def _unavailable_result(self, decision: WebGripperDecision) -> dict:
        message = "gripper command action unavailable"
        result = self._result(
            accepted=False,
            decision=decision,
            status={"state": "unavailable", "message": message},
        )
        result["response"] = {"accepted": False, "message": message}
        return result
