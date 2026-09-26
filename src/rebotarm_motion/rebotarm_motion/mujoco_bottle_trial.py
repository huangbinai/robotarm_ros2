"""Run a staged bottle contact trial against the verified MuJoCo ROS backend."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque

from .mujoco_moveit_acceptance import (
    ARM_JOINT_NAMES,
    _build_motion_plan_request,
    require_simulation_server,
)


# The local end_link pose was measured from the canonical bottle scene with the
# gripper aligned around the bottle shoulder. It is a simulation fixture, not a
# real-arm grasp calibration.
_IK_SEED = (0.0, -1.306, -1.052, 0.933, 0.0, 0.0)
_END_LINK_ORIENTATION_XYZW = (0.0, 0.5593, 0.0, 0.8290)
_MIN_LIFT_M = 0.02


def bottle_target(bottle_pose, *, stage: str) -> tuple[float, float, float]:
    if stage not in {"pregrasp", "grasp"}:
        raise ValueError(f"unknown stage: {stage}")
    x = float(bottle_pose.position.x)
    y = float(bottle_pose.position.y)
    z = float(bottle_pose.position.z)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("bottle pose must be finite")
    if not (0.26 <= x <= 0.30 and abs(y) <= 0.02 and abs(z) <= 0.01):
        raise ValueError("this trial supports only the canonical tabletop bottle placement")
    return (x + 0.015, y, z + (0.24 if stage == "pregrasp" else 0.14))


def safe_to_run_trial(bottle_pose, *, max_xy_offset_m: float = 0.015) -> bool:
    return math.hypot(bottle_pose.position.x - 0.28, bottle_pose.position.y) <= max_xy_offset_m


def classify_grasp(bilateral_contact: bool, bottle_lift_m: float) -> str:
    if bilateral_contact and bottle_lift_m >= _MIN_LIFT_M:
        return "contact_and_lift"
    if bilateral_contact:
        return "bilateral_contact_without_lift"
    return "no_bilateral_contact"


def _wait(executor, future, timeout: float):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("simulation service/action timed out")
    return future.result()


def _arm_state(message) -> tuple[float, ...]:
    values = dict(zip(message.name, message.position))
    if any(name not in values for name in ARM_JOINT_NAMES):
        raise RuntimeError("MuJoCo joint state is incomplete")
    return tuple(float(values[name]) for name in ARM_JOINT_NAMES)


def run_trial(*, timeout: float = 30.0, close_width_m: float = 0.04) -> dict:
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rebotarm_msgs.srv import GetSimulationGraspState, SetGripper
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be positive and finite")
    if not math.isfinite(close_width_m) or not 0.0 <= close_width_m <= 0.09:
        raise ValueError("close width must be within [0, 0.09] m")

    rclpy.init()
    node = Node("mujoco_bottle_trial")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    joint_states = deque(maxlen=10)
    node.create_subscription(JointState, "/rebotarm/joint_states", lambda msg: joint_states.append(msg), 10)
    action = ActionClient(node, FollowJointTrajectory, "/rebotarm/follow_joint_trajectory")
    observe = node.create_client(GetSimulationGraspState, "/rebotarm/sim/grasp_state")
    gripper = node.create_client(SetGripper, "/rebotarm/gripper/set")
    stop = node.create_client(Trigger, "/rebotarm/trajectory_stop")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    validity = node.create_client(GetStateValidity, "/check_state_validity")
    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")

    def snapshot():
        response = _wait(executor, observe.call_async(GetSimulationGraspState.Request()), timeout)
        if not response.success:
            raise RuntimeError(response.message)
        return response

    def set_width(width):
        request = SetGripper.Request()
        request.position = width
        response = _wait(executor, gripper.call_async(request), timeout)
        if not response.success:
            raise RuntimeError(f"MuJoCo gripper rejected {width:.3f} m")
        return float(response.reached_position)

    def solve(stage, bottle, current):
        from geometry_msgs.msg import PoseStamped

        target = bottle_target(bottle, stage=stage)
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = "arm"
        ik_request.ik_link_name = "end_link"
        ik_request.pose_stamped = PoseStamped()
        ik_request.pose_stamped.header.frame_id = "base_link"
        ik_request.pose_stamped.pose.position.x = target[0]
        ik_request.pose_stamped.pose.position.y = target[1]
        ik_request.pose_stamped.pose.position.z = target[2]
        orientation = ik_request.pose_stamped.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = _END_LINK_ORIENTATION_XYZW
        ik_request.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        ik_request.robot_state.joint_state.position = list(_IK_SEED)
        ik_request.avoid_collisions = False
        result = _wait(executor, ik.call_async(request), timeout)
        if int(result.error_code.val) != 1:
            raise RuntimeError(f"{stage} IK failed: {result.error_code.val}")
        candidate = _arm_state(result.solution.joint_state)
        check = GetStateValidity.Request()
        check.group_name = "arm"
        check.robot_state = result.solution
        if not _wait(executor, validity.call_async(check), timeout).valid:
            raise RuntimeError(f"{stage} IK state is invalid in MoveIt")
        plan = _build_motion_plan_request(
            service_type=GetMotionPlan,
            joint_names=ARM_JOINT_NAMES,
            target_positions=candidate,
            group_name="arm",
            start_positions=current,
        )
        response = _wait(executor, planner.call_async(plan), timeout).motion_plan_response
        if int(response.error_code.val) != 1 or not response.trajectory.joint_trajectory.points:
            raise RuntimeError(f"{stage} MoveIt planning failed: {response.error_code.val}")
        return target, candidate, response.trajectory.joint_trajectory

    report = {"ok": False, "stage": "preflight", "arm_phases": []}
    active_goal = None
    verified_sim_backend = False
    try:
        for client in (observe, gripper, stop, ik, validity, planner):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"required service unavailable: {client.srv_name}")
        if not action.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("MuJoCo trajectory action unavailable")
        require_simulation_server(node, executor, "/rebotarm/follow_joint_trajectory", timeout)
        verified_sim_backend = True
        deadline = time.monotonic() + timeout
        while not joint_states and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if not joint_states:
            raise RuntimeError("fresh MuJoCo joint state unavailable")

        baseline = snapshot()
        bottle = baseline.bottle_pose
        report["initial_bottle_xyz_m"] = [bottle.position.x, bottle.position.y, bottle.position.z]
        # Validate the fixture before sending any gripper or arm command.
        bottle_target(bottle, stage="grasp")
        if not safe_to_run_trial(bottle):
            raise RuntimeError("bottle moved away from canonical start; restart the simulation launch")
        report["stage"] = "open_gripper"
        report["open_width_m"] = set_width(0.09)

        for stage in ("pregrasp", "grasp"):
            report["stage"] = stage
            observed = snapshot().bottle_pose
            drift = math.dist(
                (observed.position.x, observed.position.y, observed.position.z),
                (bottle.position.x, bottle.position.y, bottle.position.z),
            )
            if drift > 0.015:
                raise RuntimeError(f"bottle moved {drift:.3f} m before {stage}")
            target, candidate, trajectory = solve(stage, observed, _arm_state(joint_states[-1]))
            require_simulation_server(node, executor, "/rebotarm/follow_joint_trajectory", timeout)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            handle = _wait(executor, action.send_goal_async(goal), timeout)
            if not handle.accepted:
                raise RuntimeError(f"{stage} trajectory rejected")
            active_goal = handle
            duration = trajectory.points[-1].time_from_start
            budget = float(duration.sec) + float(duration.nanosec) * 1e-9 + 10.0
            result = _wait(executor, handle.get_result_async(), max(timeout, budget))
            active_goal = None
            phase = {
                "stage": stage, "target_end_link_xyz_m": target,
                "ik_joint_positions_rad": candidate,
                "planned_points": len(trajectory.points),
                "action_status": int(result.status),
                "action_error_code": int(result.result.error_code),
            }
            report["arm_phases"].append(phase)
            if phase["action_error_code"] != 0 or phase["action_status"] != 4:
                raise RuntimeError(f"{stage} trajectory did not succeed")

        report["stage"] = "close_gripper"
        report["close_width_m"] = set_width(close_width_m)
        max_left = max_right = max_bottle = 0
        bilateral_same_frame = False
        max_force = max_penetration = 0.0
        final = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            final = snapshot()
            max_left = max(max_left, final.left_finger_contact_count)
            max_right = max(max_right, final.right_finger_contact_count)
            max_bottle = max(max_bottle, final.bottle_contact_count)
            bilateral_same_frame |= bool(
                final.left_finger_contact_count and final.right_finger_contact_count
            )
            max_force = max(max_force, final.max_bottle_contact_force_n)
            max_penetration = max(max_penetration, final.max_bottle_penetration_m)
        report["contact"] = {
            "bottle_count_peak": max_bottle,
            "left_finger_peak": max_left,
            "right_finger_peak": max_right,
            "bilateral_same_frame": bilateral_same_frame,
            "max_force_n": max_force,
            "max_penetration_m": max_penetration,
        }
        report["final_bottle_xyz_m"] = [
            final.bottle_pose.position.x, final.bottle_pose.position.y, final.bottle_pose.position.z
        ]
        report["bottle_displacement_m"] = math.dist(
            report["initial_bottle_xyz_m"], report["final_bottle_xyz_m"]
        )
        report["bottle_lift_m"] = max(
            0.0, final.bottle_pose.position.z - bottle.position.z
        )
        report["stage"] = "observed"
        report["ok"] = True
        report["bilateral_finger_contact"] = bilateral_same_frame
        report["grasp_result"] = classify_grasp(
            bilateral_same_frame, report["bottle_lift_m"]
        )
        return report
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        if verified_sim_backend and active_goal is not None:
            try:
                _wait(executor, active_goal.cancel_goal_async(), 3.0)
            except Exception as cancel_exc:
                report["cancel_error"] = str(cancel_exc)
        if verified_sim_backend:
            try:
                _wait(executor, stop.call_async(Trigger.Request()), 3.0)
            except Exception as stop_exc:
                report["stop_error"] = str(stop_exc)
        return report
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--close-width-m", type=float, default=0.04)
    args = parser.parse_args(argv)
    report = run_trial(timeout=args.timeout, close_width_m=args.close_width_m)
    print(json.dumps(report, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
