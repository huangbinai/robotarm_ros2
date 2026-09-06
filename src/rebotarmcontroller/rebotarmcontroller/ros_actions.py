from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose
from trajectory_msgs.msg import JointTrajectoryPoint

from .conversions import pose_to_xyz_rpy
from .gripper_safety import active_gripper_failure_reason
from .runtime_parameters import validate_move_to_pose_goal
from .trajectory_safety import (
    TrajectorySafetyLimits,
    interpolate_trajectory,
    validate_trajectory,
)


class ArmActions:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._command_arbiter = hardware.command_arbiter
        self._trajectory_limits = TrajectorySafetyLimits(
            position_min=np.asarray(
                node.get_parameter("trajectory_safety.position_min_rad").value
            ),
            position_max=np.asarray(
                node.get_parameter("trajectory_safety.position_max_rad").value
            ),
            max_velocity=np.asarray(
                node.get_parameter("trajectory_safety.max_velocity_rad_s").value
            ),
            max_acceleration=np.asarray(
                node.get_parameter(
                    "trajectory_safety.max_acceleration_rad_s2"
                ).value
            ),
            start_tolerance_rad=float(
                node.get_parameter("trajectory_safety.start_tolerance_rad").value
            ),
        )
        self._goal_tolerance_rad = self._positive_parameter(
            "trajectory_safety.goal_tolerance_rad"
        )
        self._settle_timeout_sec = self._positive_parameter(
            "trajectory_safety.settle_timeout_sec"
        )
        self._sample_period_sec = self._positive_parameter(
            "trajectory_safety.sample_period_sec"
        )
        self._goal_leases = {}
        self._namespace = namespace
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self._execute_move_to_pose_exclusive,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_move_to_pose,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self._execute_follow_joint_trajectory_exclusive,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_follow_joint_trajectory,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self._execute_gripper_command_exclusive,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_gripper_command,
            callback_group=node.reentrant_group,
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self._node.get_parameter(name).value)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def arm_goal_callback(self, _goal_request):
        if not self._hardware.ready_for_motion:
            self._node.get_logger().warning(
                "Rejecting arm motion goal: hardware is not explicitly enabled and ready"
            )
            return GoalResponse.REJECT
        return (
            GoalResponse.ACCEPT
            if self._command_arbiter.available("arm")
            else GoalResponse.REJECT
        )

    def gripper_goal_callback(self, _goal_request):
        if not self._hardware.ready_for_motion:
            self._node.get_logger().warning(
                "Rejecting gripper motion goal: hardware is not explicitly enabled and ready"
            )
            return GoalResponse.REJECT
        return (
            GoalResponse.ACCEPT
            if self._command_arbiter.available("gripper")
            else GoalResponse.REJECT
        )

    def cancel_move_to_pose(self, _goal_handle):
        self._stop_move_to_pose_motion()
        self._node.publish_arm_status()
        return CancelResponse.ACCEPT

    def cancel_follow_joint_trajectory(self, _goal_handle):
        self._hardware.stop_active_motion()
        self._node.publish_arm_status()
        return CancelResponse.ACCEPT

    def cancel_gripper_command(self, _goal_handle):
        self._hardware.stop_gripper_motion()
        return CancelResponse.ACCEPT

    def _execute_move_to_pose_exclusive(self, goal_handle):
        lease = self._command_arbiter.acquire("arm", "move_to_pose")
        if lease is None:
            result = MoveToPose.Result()
            goal_handle.abort()
            result.success = False
            result.message = f"arm command busy: {self._command_arbiter.owner('arm')}"
            self._set_move_to_pose_final_pose(result)
            return result
        try:
            self._goal_leases[id(goal_handle)] = lease
            return self.execute_move_to_pose(goal_handle)
        finally:
            self._goal_leases.pop(id(goal_handle), None)
            if self._command_arbiter.release(lease):
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()

    def execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        try:
            requested_duration = validate_move_to_pose_goal(goal)
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            if not np.all(
                np.isfinite(np.asarray((x, y, z, roll, pitch, yaw), dtype=np.float64))
            ):
                raise ValueError("move_to_pose converted target pose must be finite")
            self._hardware.set_state_machine("TRAJ_RUNNING")
            self._node.publish_arm_status()
            self._hardware.ensure_pos_vel_control()
            ok = self._hardware.endpos_ctrl.move_to_traj(
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
                requested_duration,
            )
        except Exception as exc:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._set_move_to_pose_final_pose(result)
            return result

        if not ok:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = "trajectory planning failed"
            self._set_move_to_pose_final_pose(result)
            return result

        start = time.monotonic()
        motion_deadline = start + requested_duration + self._settle_timeout_sec
        feedback = MoveToPose.Feedback()
        while bool(getattr(self._hardware.endpos_ctrl, "_moving", False)):
            if self._move_to_pose_interrupted(goal_handle, result):
                return result
            if time.monotonic() >= motion_deadline:
                self._stop_move_to_pose_motion()
                goal_handle.abort()
                result.success = False
                result.message = (
                    "move_to_pose execution timed out before trajectory sender completed"
                )
                self._set_move_to_pose_final_pose(result)
                return result

            feedback.current_pose = self._hardware.current_pose()
            elapsed = float(time.monotonic() - start)
            if requested_duration > 0.0:
                feedback.progress = max(0.0, min(1.0, elapsed / requested_duration))
            else:
                traj = getattr(self._hardware.endpos_ctrl, "_traj", [])
                if traj:
                    idx = float(getattr(self._hardware.endpos_ctrl, "_traj_idx", 0))
                    feedback.progress = max(0.0, min(1.0, idx / float(len(traj))))
                else:
                    feedback.progress = 1.0
            feedback.time_elapsed = elapsed
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        # Cancellation, explicit stops, and protective disable can all clear
        # ``_moving``.  Recheck the action state before treating that as normal
        # trajectory completion.
        if self._move_to_pose_interrupted(goal_handle, result):
            return result

        trajectory = list(getattr(self._hardware.endpos_ctrl, "_traj", []))
        if not trajectory:
            goal_handle.abort()
            result.success = False
            result.message = "move_to_pose completed without a final joint target"
            self._set_move_to_pose_final_pose(result)
            return result
        final_target = np.asarray(trajectory[-1], dtype=np.float64)
        settle_deadline = time.monotonic() + self._settle_timeout_sec
        max_error = float("inf")
        while time.monotonic() < settle_deadline:
            if self._move_to_pose_interrupted(goal_handle, result):
                return result
            positions, _velocities, _effort = self._hardware.get_joint_state()
            current = np.asarray(positions, dtype=np.float64)
            if current.shape != final_target.shape or not np.all(np.isfinite(current)):
                self._stop_move_to_pose_motion()
                goal_handle.abort()
                result.success = False
                result.message = "move_to_pose final joint feedback is invalid"
                self._set_move_to_pose_final_pose(result)
                return result
            max_error = float(np.max(np.abs(current - final_target)))
            if max_error <= self._goal_tolerance_rad:
                break
            time.sleep(0.05)
        else:
            self._stop_move_to_pose_motion()
            goal_handle.abort()
            result.success = False
            result.message = (
                "move_to_pose goal not reached within tolerance "
                f"(max error {max_error:.3f} rad > "
                f"{self._goal_tolerance_rad:.3f} rad)"
            )
            self._set_move_to_pose_final_pose(result)
            return result

        result.success = True
        result.message = "move_to_pose complete"
        self._set_move_to_pose_final_pose(result)
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def _stop_move_to_pose_motion(self) -> None:
        self._hardware.endpos_ctrl._stop_send.set()
        self._hardware.endpos_ctrl._moving = False
        if self._hardware.ready_for_motion:
            try:
                self._hardware.hold_current_position()
                self._hardware.set_state_machine("IDLE")
            except Exception as exc:
                self._node.get_logger().error(
                    f"move_to_pose stop could not establish position hold: {exc}"
                )

    def _move_to_pose_interrupted(self, goal_handle, result) -> bool:
        if goal_handle.is_cancel_requested:
            self._stop_move_to_pose_motion()
            self._node.publish_arm_status()
            goal_handle.canceled()
            result.success = False
            result.message = "canceled"
            self._set_move_to_pose_final_pose(result)
            return True
        if (
            self._hardware.state_machine != "TRAJ_RUNNING"
            or not self._hardware.ready_for_motion
            or not self._goal_lease_is_current(goal_handle)
        ):
            self._stop_move_to_pose_motion()
            goal_handle.abort()
            result.success = False
            result.message = "preempted or hardware no longer ready"
            self._set_move_to_pose_final_pose(result)
            return True
        return False

    def _set_move_to_pose_final_pose(self, result) -> None:
        try:
            result.final_pose = self._hardware.current_pose()
        except Exception as exc:
            self._node.get_logger().error(
                f"move_to_pose final pose unavailable: {exc}"
            )

    def _execute_follow_joint_trajectory_exclusive(self, goal_handle):
        lease = self._command_arbiter.acquire("arm", "follow_joint_trajectory")
        if lease is None:
            result = FollowJointTrajectory.Result()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"arm command busy: {self._command_arbiter.owner('arm')}"
            return result
        try:
            self._goal_leases[id(goal_handle)] = lease
            return self.execute_follow_joint_trajectory(goal_handle)
        finally:
            self._goal_leases.pop(id(goal_handle), None)
            if self._command_arbiter.release(lease):
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()

    def execute_follow_joint_trajectory(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory

        if not trajectory.joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        try:
            current = self._current_positions_for(list(trajectory.joint_names))
            sample_times, sample_positions = self._validated_trajectory(
                list(trajectory.joint_names),
                trajectory.points,
                current,
            )
        except Exception as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        trajectory_done = False
        start = time.monotonic()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)

        self._hardware.set_state_machine("TRAJ_RUNNING")
        self._node.publish_arm_status()
        try:
            self._hardware.ensure_pos_vel_control()

            final_time = sample_times[-1]
            while True:
                if self._trajectory_stopped(goal_handle, result):
                    return result
                elapsed = min(max(time.monotonic() - start, 0.0), final_time)
                target = interpolate_trajectory(sample_times, sample_positions, elapsed)
                self._set_endpos_target(list(trajectory.joint_names), target)

                desired = JointTrajectoryPoint()
                desired.positions = [float(v) for v in target]
                feedback.desired = desired
                feedback.actual = self._actual_point(list(trajectory.joint_names))
                feedback.error = self._error_point(desired, feedback.actual)
                goal_handle.publish_feedback(feedback)
                if elapsed >= final_time:
                    break
                time.sleep(min(self._sample_period_sec, final_time - elapsed))
            trajectory_done = True

            if not trajectory_done:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory interrupted"
                return result

            self._set_endpos_target(list(trajectory.joint_names), sample_positions[-1])
            ok, max_error = self._wait_until_goal_reached(
                goal_handle,
                list(trajectory.joint_names),
                sample_positions[-1],
                self._goal_tolerance_rad,
                result,
            )
            if not ok:
                if result.error_string in ("canceled", "preempted"):
                    return result
                self._hardware.hold_current_position()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = (
                    "trajectory goal not reached within tolerance "
                    f"(max error {max_error:.3f} rad > "
                    f"{self._goal_tolerance_rad:.3f} rad)"
                )
                return result
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "follow_joint_trajectory complete"
            if self._goal_lease_is_current(goal_handle):
                self._hardware.set_state_machine("IDLE")
            return result
        except Exception as exc:
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

    def _set_endpos_target(self, joint_names: list[str], positions: np.ndarray) -> None:
        if set(joint_names) != set(self._hardware.joint_names):
            raise ValueError(f"trajectory joints must match {self._hardware.joint_names}")
        by_name = {name: float(pos) for name, pos in zip(joint_names, positions)}
        ordered = np.array(
            [by_name[name] for name in self._hardware.joint_names],
            dtype=np.float64,
        )
        self._hardware.endpos_ctrl._q_target[:] = ordered

    def _trajectory_stopped(self, goal_handle, result) -> bool:
        if goal_handle.is_cancel_requested:
            self._hardware.stop_active_motion()
            self._node.publish_arm_status()
            goal_handle.canceled()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "canceled"
            return True
        if self._hardware.state_machine != "TRAJ_RUNNING":
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "preempted"
            return True
        if not self._goal_lease_is_current(goal_handle):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "preempted"
            return True
        return False

    def _goal_lease_is_current(self, goal_handle) -> bool:
        lease = self._goal_leases.get(id(goal_handle))
        return lease is None or self._command_arbiter.is_current(lease)

    def _current_positions_for(self, joint_names: list[str]) -> np.ndarray:
        if len(joint_names) != len(set(joint_names)):
            raise ValueError("joint_names must not contain duplicates")
        if set(joint_names) != set(self._hardware.joint_names):
            raise ValueError(
                f"trajectory joints must match {self._hardware.joint_names}"
            )
        current, _, _ = self._hardware.get_joint_state()
        by_name = {
            name: float(pos)
            for name, pos in zip(self._hardware.joint_names, current)
        }
        return np.array([by_name[name] for name in joint_names], dtype=np.float64)

    def _validated_trajectory(
        self,
        joint_names: list[str],
        points: list[JointTrajectoryPoint],
        current: np.ndarray,
    ) -> tuple[list[float], list[np.ndarray]]:
        return validate_trajectory(
            joint_names,
            points,
            current,
            self._trajectory_limits,
        )

    def _wait_until_time(self, goal_handle, target_time: float, result) -> bool:
        while time.monotonic() < target_time:
            if goal_handle.is_cancel_requested:
                self._hardware.stop_active_motion()
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "canceled"
                return False
            if self._hardware.state_machine != "TRAJ_RUNNING":
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "preempted"
                return False
            time.sleep(0.01)
        return True

    def _wait_until_goal_reached(
        self,
        goal_handle,
        joint_names: list[str],
        target: np.ndarray,
        goal_tolerance: float,
        result,
    ) -> tuple[bool, float]:
        deadline = time.monotonic() + self._settle_timeout_sec
        max_error = float("inf")
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._hardware.stop_active_motion()
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "canceled"
                return False, 0.0
            if self._hardware.state_machine != "TRAJ_RUNNING":
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "preempted"
                return False, 0.0
            actual = self._current_positions_for(joint_names)
            max_error = float(np.max(np.abs(actual - target)))
            if max_error <= goal_tolerance:
                return True, max_error
            time.sleep(0.05)
        return False, max_error

    def _actual_point(self, joint_names: list[str] | None = None) -> JointTrajectoryPoint:
        pos, vel, _ = self._hardware.get_joint_state()
        if joint_names is not None:
            by_name = {
                name: i
                for i, name in enumerate(self._hardware.joint_names)
            }
            pos = [pos[by_name[name]] for name in joint_names]
            vel = [vel[by_name[name]] for name in joint_names]
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in pos]
        point.velocities = [float(v) for v in vel]
        return point

    @staticmethod
    def _error_point(desired: JointTrajectoryPoint, actual: JointTrajectoryPoint) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [
            float(a - d) for d, a in zip(desired.positions, actual.positions)
        ]
        if desired.velocities and actual.velocities:
            point.velocities = [
                float(a - d) for d, a in zip(desired.velocities, actual.velocities)
            ]
        return point

    def _execute_gripper_command_exclusive(self, goal_handle):
        lease = self._command_arbiter.acquire("gripper", "gripper_command")
        if lease is None:
            result = GripperCommand.Result()
            goal_handle.abort()
            result.position = self._hardware.gripper_position_m()
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result
        try:
            return self.execute_gripper_command(goal_handle)
        finally:
            self._command_arbiter.release(lease)

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position, goal.max_effort)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        timeout_sec = max(self._hardware.gripper_target_timeout_sec(), 0.1)
        last_pos = self._hardware.gripper_position_m()
        stalled = False
        while time.monotonic() - start < timeout_sec:
            if goal_handle.is_cancel_requested:
                self._hardware.stop_gripper_motion()
                goal_handle.canceled()
                result.position = self._hardware.gripper_position_m()
                result.effort = self._hardware.get_gripper_state()[2]
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos = self._hardware.gripper_position_m()
            _raw_pos, _raw_vel, effort, status_code = (
                self._hardware.get_gripper_state()
            )
            failure = active_gripper_failure_reason(
                command_error=self._hardware.gripper_command_error,
                status_code=status_code,
            )
            if failure is not None:
                self._hardware.stop_gripper_motion(failure)
                goal_handle.abort()
                result.position = pos
                result.effort = effort
                result.stalled = False
                result.reached_goal = False
                return result
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        result.position = self._hardware.gripper_position_m()
        result.effort = self._hardware.get_gripper_state()[2]
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        if result.reached_goal:
            goal_handle.succeed()
        else:
            self._hardware.stop_gripper_motion("gripper action timeout")
            goal_handle.abort()
        return result
