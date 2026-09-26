"""Search a bounded set of bottle grasp poses with MoveIt and offline MuJoCo.

Start only the MoveIt demo with fake joint states. No trajectory action is used.
"""

from __future__ import annotations

import argparse
import json
import math
import time

from rebotarm_simulation.grasp_search_physics import evaluate_grasp_paths


JOINTS = tuple(f"joint{index}" for index in range(1, 7))
START = (0.0, -0.1, -0.2, 0.2, 0.0, 0.0)
SEED = (0.0, -1.306, -1.052, 0.933, 0.0, 0.0)
SPECS = (
    (0.010, 0.130, 0.56),
    (0.015, 0.140, 0.56),
    (0.020, 0.150, 0.56),
    (0.010, 0.130, 0.64),
    (0.015, 0.140, 0.64),
    (0.020, 0.150, 0.64),
    (0.010, 0.170, 0.56),
    (0.015, 0.180, 0.56),
    (0.020, 0.190, 0.56),
)


def _wait(executor, future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("MoveIt service timed out")
    return future.result()


def _vector(joint_state):
    by_name = dict(zip(joint_state.name, joint_state.position))
    if any(name not in by_name for name in JOINTS):
        raise ValueError("IK did not return all six arm joints")
    return tuple(float(by_name[name]) for name in JOINTS)


def run_search(*, limit: int = 6, timeout: float = 10.0, close_width_m: float = 0.02) -> dict:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
    from rclpy.action import get_action_server_names_and_types_by_node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    from rebotarm_motion.mujoco_moveit_acceptance import _build_motion_plan_request
    from rebotarm_simulation.mujoco_sim import RebotArmMujoco

    if not isinstance(limit, int) or not 1 <= limit <= len(SPECS):
        raise ValueError(f"limit must be between 1 and {len(SPECS)}")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be positive")
    with RebotArmMujoco() as sim:
        bottle_xyz = sim.get_state().object_poses["bottle"][:3]
    rclpy.init()
    node = Node("mujoco_grasp_search")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ik = node.create_client(GetPositionIK, "/compute_ik")
    validity = node.create_client(GetStateValidity, "/check_state_validity")
    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    results = []
    try:
        for client in (ik, validity, planner):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"MoveIt service unavailable: {client.srv_name}")
        owners = []
        for name, namespace in node.get_node_names_and_namespaces():
            for action_name, _ in get_action_server_names_and_types_by_node(node, name, namespace):
                if action_name == "/rebotarm/follow_joint_trajectory":
                    owners.append((name, namespace))
        if owners:
            raise RuntimeError(f"search requires planning-only MoveIt; action owner found: {owners}")

        for index, (x_offset, grasp_z, pitch) in enumerate(SPECS[:limit]):
            candidate = {"index": index, "x_offset_m": x_offset, "grasp_z_m": grasp_z, "pitch_rad": pitch}
            paths = []
            current = START
            try:
                for stage, z in (("pregrasp", grasp_z + 0.10), ("grasp", grasp_z), ("lift", grasp_z + 0.08)):
                    request = GetPositionIK.Request()
                    target = request.ik_request
                    target.group_name = "arm"
                    target.ik_link_name = "end_link"
                    target.pose_stamped = PoseStamped()
                    target.pose_stamped.header.frame_id = "base_link"
                    target.pose_stamped.pose.position.x = bottle_xyz[0] + x_offset
                    target.pose_stamped.pose.position.y = bottle_xyz[1]
                    target.pose_stamped.pose.position.z = bottle_xyz[2] + z
                    target.pose_stamped.pose.orientation.y = math.sin(pitch)
                    target.pose_stamped.pose.orientation.w = math.cos(pitch)
                    target.robot_state.joint_state = JointState(name=list(JOINTS), position=list(SEED))
                    target.avoid_collisions = False
                    solution = _wait(executor, ik.call_async(request), timeout)
                    if solution.error_code.val != 1:
                        raise RuntimeError(f"{stage}: IK error {solution.error_code.val}")
                    check = GetStateValidity.Request()
                    check.group_name = "arm"
                    check.robot_state = solution.solution
                    if not _wait(executor, validity.call_async(check), timeout).valid:
                        raise RuntimeError(f"{stage}: invalid MoveIt state")
                    waypoint = _vector(solution.solution.joint_state)
                    query = _build_motion_plan_request(
                        service_type=GetMotionPlan, joint_names=JOINTS,
                        target_positions=waypoint, group_name="arm", start_positions=current,
                    )
                    planned = _wait(executor, planner.call_async(query), timeout).motion_plan_response
                    path = planned.trajectory.joint_trajectory
                    if planned.error_code.val != 1 or not path.points:
                        raise RuntimeError(f"{stage}: MoveIt plan error {planned.error_code.val}")
                    paths.append(path)
                    current = waypoint
                candidate["physics"] = evaluate_grasp_paths(
                    paths, initial_arm_positions=START, close_width_m=close_width_m
                )
                candidate["status"] = "simulated"
                candidate["planned_points"] = [len(path.points) for path in paths]
            except (RuntimeError, ValueError, TimeoutError) as exc:
                candidate["status"] = "rejected"
                candidate["reason"] = str(exc)
            results.append(candidate)
        ranked = sorted(
            (item for item in results if item["status"] == "simulated"),
            key=lambda item: (-item["physics"]["score"], item["index"]),
        )
        return {
            "candidate_count": len(results), "evaluated_count": len(ranked),
            "stable_lift_count": sum(item["physics"]["stable_lift"] for item in ranked),
            "best_index": ranked[0]["index"] if ranked else None,
            "best_is_stable_lift": bool(ranked and ranked[0]["physics"]["stable_lift"]),
            "candidates": results,
        }
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=6)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--close-width-m", type=float, default=0.02)
    parser.add_argument("--detailed", action="store_true", help="include per-phase physics metrics")
    parser.add_argument("--report", help="write complete JSON report to this path")
    args = parser.parse_args(argv)
    report = run_search(limit=args.limit, timeout=args.timeout, close_width_m=args.close_width_m)
    if args.report:
        from pathlib import Path
        target = Path(args.report)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    if not args.detailed:
        report["candidates"] = [
            {
                **{key: value for key, value in item.items() if key != "physics"},
                **({
                    "score": item["physics"]["score"],
                    "stable_lift": item["physics"]["stable_lift"],
                    "final_bottle_lift_m": item["physics"]["final_bottle_lift_m"],
                    "close_bilateral_steps": item["physics"]["close_bilateral_steps"],
                    "lift_bilateral_steps": item["physics"]["lift_bilateral_steps"],
                    "hold_bilateral_steps": item["physics"]["hold_bilateral_steps"],
                } if "physics" in item else {}),
            }
            for item in report["candidates"]
        ]
    print(json.dumps(report, sort_keys=True, indent=2))
    return 0 if report["evaluated_count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
