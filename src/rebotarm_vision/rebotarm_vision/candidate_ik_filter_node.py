from __future__ import annotations

from copy import deepcopy
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan

from .candidate_filter_policy import filter_candidate_array_by_reachability
from .grasp_preview_sender_node import _transform_from_msg, transform_pose_message
from .visual_grasp_pose_policy import (
    BaseAxisGraspPolicyConfig,
    OfficialGeometryGraspPolicyConfig,
    build_base_axis_grasp_targets,
    build_hybrid_geometry_grasp_targets,
    build_official_geometry_grasp_targets,
)
from .visual_grasp_sequence import PoseTarget


def _quat_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half = float(yaw_rad) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _normalize_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-9:
        raise ValueError("quaternion must be non-zero")
    return (x / norm, y / norm, z / norm, w / norm)


def _pose_from_target(target: PoseTarget) -> Pose:
    pose = Pose()
    pose.position.x = float(target.position[0])
    pose.position.y = float(target.position[1])
    pose.position.z = float(target.position[2])
    pose.orientation.x = float(target.orientation[0])
    pose.orientation.y = float(target.orientation[1])
    pose.orientation.z = float(target.orientation[2])
    pose.orientation.w = float(target.orientation[3])
    return pose


class CandidateIkFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_candidate_ik_filter")
        self._callback_group = ReentrantCallbackGroup()
        self.declare_parameter("input_topic", "/grasp/candidates")
        self.declare_parameter("output_topic", "/grasp/filtered_candidates")
        self.declare_parameter("output_plan_topic", "/grasp/filtered_plan")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("moveit_ik_service", "/compute_ik")
        self.declare_parameter("joint_state_topic", "/rebotarm/visual_joint_states")
        self.declare_parameter("collision_check_enabled", True)
        self.declare_parameter("collision_check_service", "/check_state_validity")
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("pose_policy", "hybrid_geometry_with_base_axis_fallback")
        self.declare_parameter("fixed_grasp_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        self.declare_parameter("orientation_yaw_offsets_rad", [0.0, 3.141592653589793])
        self.declare_parameter("candidate_grasp_z_offsets_m", [0.0, 0.03])
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.01, 0.0])
        self.declare_parameter("pregrasp_base_z_offset_m", 0.05)
        self.declare_parameter("grasp_base_z_offset_m", 0.0)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self._latest_joint_state: JointState | None = None
        self._warned_missing_joint_state = False
        self._filter_busy = False
        self._warned_filter_busy = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ik_client = self.create_client(
            GetPositionIK,
            str(self.get_parameter("moveit_ik_service").value),
            callback_group=self._callback_group,
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
            callback_group=self._callback_group,
        )
        self._candidates_pub = self.create_publisher(
            GraspCandidateArray,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self._plan_pub = self.create_publisher(
            GraspPlan,
            str(self.get_parameter("output_plan_topic").value),
            10,
        )
        self.create_subscription(
            GraspCandidateArray,
            self._input_topic,
            self._on_candidates,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            10,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "candidate IK filter ready: "
            f"input={self._input_topic}, output={str(self.get_parameter('output_topic').value)}, "
            f"plan={str(self.get_parameter('output_plan_topic').value)}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))

    def _tuple4(self, name: str) -> tuple[float, float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 4:
            raise ValueError(f"{name} must contain exactly 4 values")
        return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))

    def _on_joint_state(self, msg: JointState) -> None:
        if self._valid_joint_state(msg):
            self._latest_joint_state = msg
            self._warned_missing_joint_state = False
            return
        if not self._warned_missing_joint_state:
            self.get_logger().warn(
                "candidate IK filter ignored empty or incomplete joint state; "
                "waiting before calling MoveIt IK"
            )
            self._warned_missing_joint_state = True

    def _valid_joint_state(self, msg: JointState | None) -> bool:
        if msg is None:
            return False
        if not msg.name or not msg.position:
            return False
        if len(msg.position) < len(msg.name):
            return False
        return all(math.isfinite(float(value)) for value in msg.position[: len(msg.name)])

    def _on_candidates(self, msg: GraspCandidateArray) -> None:
        if self._filter_busy:
            if not self._warned_filter_busy:
                self.get_logger().warn(
                    "candidate IK filter is still processing previous candidates; dropping this frame"
                )
                self._warned_filter_busy = True
            return
        self._filter_busy = True
        try:
            self._on_candidates_unlocked(msg)
        finally:
            self._filter_busy = False
            self._warned_filter_busy = False

    def _on_candidates_unlocked(self, msg: GraspCandidateArray) -> None:
        if not msg.candidates:
            self._publish_filtered(msg, [])
            return
        reachable = []
        reachable_targets: list[tuple[PoseTarget, PoseTarget]] = []
        for candidate in msg.candidates:
            try:
                accepted_target = None
                for pregrasp, grasp, variant_label in self._candidate_target_variants(msg, candidate.pose):
                    is_reachable = (
                        self._check_ik_and_collision(pregrasp, f"{variant_label}/pregrasp")
                        and self._check_ik_and_collision(grasp, f"{variant_label}/grasp")
                    )
                    if is_reachable:
                        accepted_target = (pregrasp, grasp)
                        self.get_logger().info(f"candidate IK filter accepted {variant_label}")
                        break
                reachable.append(accepted_target is not None)
                if accepted_target is not None:
                    reachable_targets.append(accepted_target)
            except Exception as exc:
                self.get_logger().warn(f"candidate IK filter rejected candidate: {exc}")
                reachable.append(False)
        self._publish_filtered(msg, reachable, reachable_targets)

    def _list_float_parameter(self, name: str) -> list[float]:
        values = list(self.get_parameter(name).value)
        return [float(value) for value in values]

    def _candidate_target_variants(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
    ) -> list[tuple[PoseTarget, PoseTarget, str]]:
        variants: list[tuple[PoseTarget, PoseTarget, str]] = []
        pose_policy = str(self.get_parameter("pose_policy").value).strip()
        if pose_policy in ("hybrid_geometry", "hybrid_geometry_with_base_axis_fallback"):
            variants.extend(self._hybrid_geometry_target_variants(candidates, pose))
        if pose_policy in ("official_geometry", "official_geometry_with_base_axis_fallback"):
            variants.extend(self._official_geometry_target_variants(candidates, pose))
        if (
            pose_policy
            in (
                "base_axis",
                "official_geometry_with_base_axis_fallback",
                "hybrid_geometry_with_base_axis_fallback",
            )
            or not variants
        ):
            variants.extend(self._base_axis_target_variants(candidates, pose))
        return variants

    def _hybrid_geometry_target_variants(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
    ) -> list[tuple[PoseTarget, PoseTarget, str]]:
        variants: list[tuple[PoseTarget, PoseTarget, str]] = []
        base_orientation = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        yaw_offsets = self._list_float_parameter("orientation_yaw_offsets_rad")
        z_offsets = self._list_float_parameter("candidate_grasp_z_offsets_m")
        if not yaw_offsets:
            yaw_offsets = [0.0]
        if not z_offsets:
            z_offsets = [0.0]
        for yaw_index, yaw_offset in enumerate(yaw_offsets):
            orientation = _normalize_quaternion(
                _quat_multiply(_yaw_quaternion(yaw_offset), base_orientation)
            )
            for z_index, z_offset in enumerate(z_offsets):
                pregrasp, grasp = self._build_targets(
                    candidates,
                    pose,
                    orientation_xyzw=orientation,
                    grasp_z_offset_extra_m=float(z_offset),
                    use_hybrid_geometry=True,
                )
                variants.append((pregrasp, grasp, f"hybrid_geometry_yaw{yaw_index}_z{z_index}"))
        return variants

    def _official_geometry_target_variants(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
    ) -> list[tuple[PoseTarget, PoseTarget, str]]:
        variants: list[tuple[PoseTarget, PoseTarget, str]] = []
        base_orientation = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        yaw_offsets = self._list_float_parameter("orientation_yaw_offsets_rad")
        z_offsets = self._list_float_parameter("candidate_grasp_z_offsets_m")
        if not yaw_offsets:
            yaw_offsets = [0.0]
        if not z_offsets:
            z_offsets = [0.0]
        for yaw_index, yaw_offset in enumerate(yaw_offsets):
            orientation = _normalize_quaternion(
                _quat_multiply(_yaw_quaternion(yaw_offset), base_orientation)
            )
            for z_index, z_offset in enumerate(z_offsets):
                pregrasp, grasp = self._build_targets(
                    candidates,
                    pose,
                    orientation_xyzw=orientation,
                    grasp_z_offset_extra_m=float(z_offset),
                    use_candidate_approach_axis=True,
                )
                variants.append((pregrasp, grasp, f"official_geometry_yaw{yaw_index}_z{z_index}"))
        return variants

    def _base_axis_target_variants(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
    ) -> list[tuple[PoseTarget, PoseTarget, str]]:
        variants: list[tuple[PoseTarget, PoseTarget, str]] = []
        base_orientation = self._tuple4("fixed_grasp_orientation_xyzw")
        yaw_offsets = self._list_float_parameter("orientation_yaw_offsets_rad")
        z_offsets = self._list_float_parameter("candidate_grasp_z_offsets_m")
        if not yaw_offsets:
            yaw_offsets = [0.0]
        if not z_offsets:
            z_offsets = [0.0]
        for yaw_index, yaw_offset in enumerate(yaw_offsets):
            orientation = _normalize_quaternion(
                _quat_multiply(_yaw_quaternion(yaw_offset), base_orientation)
            )
            for z_index, z_offset in enumerate(z_offsets):
                pregrasp, grasp = self._build_targets(
                    candidates,
                    pose,
                    orientation_xyzw=orientation,
                    grasp_z_offset_extra_m=float(z_offset),
                )
                variants.append((pregrasp, grasp, f"base_axis_yaw{yaw_index}_z{z_index}"))
        return variants

    def _build_targets(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
        *,
        orientation_xyzw: tuple[float, float, float, float] | None = None,
        grasp_z_offset_extra_m: float = 0.0,
        use_hybrid_geometry: bool = False,
        use_candidate_approach_axis: bool = False,
    ) -> tuple[PoseTarget, PoseTarget]:
        grasp_pose = deepcopy(pose)
        source_frame = str(candidates.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            grasp_pose = transform_pose_message(grasp_pose, _transform_from_msg(tf_msg))
        position_xyz = (
            float(grasp_pose.position.x),
            float(grasp_pose.position.y),
            float(grasp_pose.position.z),
        )
        grasp_z_offset_m = float(self.get_parameter("grasp_base_z_offset_m").value) + float(grasp_z_offset_extra_m)
        base_axis_config = BaseAxisGraspPolicyConfig(
            fixed_orientation_xyzw=orientation_xyzw or self._tuple4("fixed_grasp_orientation_xyzw"),
            approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
            pregrasp_distance_m=float(self.get_parameter("base_pregrasp_distance_m").value),
            tcp_offset_xyz=self._tuple3("tcp_offset_xyz"),
            target_base_offset_xyz=self._tuple3("target_base_offset_xyz"),
            pregrasp_z_offset_m=float(self.get_parameter("pregrasp_base_z_offset_m").value),
            grasp_z_offset_m=grasp_z_offset_m,
        )
        if use_hybrid_geometry:
            return build_hybrid_geometry_grasp_targets(
                grasp_position_xyz=position_xyz,
                candidate_orientation_xyzw=orientation_xyzw
                or (
                    float(grasp_pose.orientation.x),
                    float(grasp_pose.orientation.y),
                    float(grasp_pose.orientation.z),
                    float(grasp_pose.orientation.w),
                ),
                config=base_axis_config,
            )
        if use_candidate_approach_axis:
            return build_official_geometry_grasp_targets(
                grasp_position_xyz=position_xyz,
                grasp_orientation_xyzw=orientation_xyzw
                or (
                    float(grasp_pose.orientation.x),
                    float(grasp_pose.orientation.y),
                    float(grasp_pose.orientation.z),
                    float(grasp_pose.orientation.w),
                ),
                config=OfficialGeometryGraspPolicyConfig(
                    pregrasp_distance_m=float(self.get_parameter("base_pregrasp_distance_m").value),
                    tcp_offset_xyz=self._tuple3("tcp_offset_xyz"),
                    target_base_offset_xyz=self._tuple3("target_base_offset_xyz"),
                    pregrasp_z_offset_m=float(self.get_parameter("pregrasp_base_z_offset_m").value),
                    grasp_z_offset_m=grasp_z_offset_m,
                ),
            )
        return build_base_axis_grasp_targets(
            grasp_position_xyz=position_xyz,
            config=base_axis_config,
        )

    def _check_ik_and_collision(self, target: PoseTarget, label: str) -> bool:
        solution = self._solve_ik(target, label)
        if solution is None:
            return False
        return self._check_state_validity(solution, label)

    def _solve_ik(self, target: PoseTarget, label: str):
        if not self._ik_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn("candidate IK filter IK service unavailable")
            return None
        if not self._valid_joint_state(self._latest_joint_state):
            if not self._warned_missing_joint_state:
                self.get_logger().warn(
                    "candidate IK filter has no valid joint state yet; "
                    "skipping IK to avoid empty MoveIt RobotState"
                )
                self._warned_missing_joint_state = True
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = str(self.get_parameter("moveit_group_name").value)
        request.ik_request.ik_link_name = str(self.get_parameter("ee_frame_id").value)
        request.ik_request.robot_state.joint_state = deepcopy(self._latest_joint_state)
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = self._target_frame
        request.ik_request.pose_stamped.pose = _pose_from_target(target)
        request.ik_request.avoid_collisions = False
        future = self._ik_client.call_async(request)
        deadline = time.monotonic() + max(self._service_timeout_sec, 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(
                f"candidate IK filter IK timed out for {label}: "
                f"target=({target.position[0]:.3f}, {target.position[1]:.3f}, {target.position[2]:.3f})"
            )
            return None
        result = future.result()
        if result is None:
            self.get_logger().warn(f"candidate IK filter IK failed for {label}: empty service result")
            return None
        error_code = int(getattr(result.error_code, "val", 99999))
        if error_code != 1:
            self.get_logger().warn(
                f"candidate IK filter IK failed for {label}: error_code={error_code}, "
                f"target=({target.position[0]:.3f}, {target.position[1]:.3f}, {target.position[2]:.3f})"
            )
            return None
        return getattr(result, "solution", None)

    def _check_state_validity(self, robot_state, label: str) -> bool:
        if not bool(self.get_parameter("collision_check_enabled").value):
            return True
        if robot_state is None:
            return False
        if not self._state_validity_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn("candidate IK filter state validity service unavailable")
            return False
        request = GetStateValidity.Request()
        request.group_name = str(self.get_parameter("collision_group_name").value)
        request.robot_state = robot_state
        future = self._state_validity_client.call_async(request)
        deadline = time.monotonic() + max(self._service_timeout_sec, 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(f"candidate IK filter state validity timed out for {label}")
            return False
        result = future.result()
        if result is None:
            self.get_logger().warn(f"candidate IK filter state validity failed for {label}: empty service result")
            return False
        valid = bool(getattr(result, "valid", False))
        if not valid:
            self.get_logger().warn(f"candidate IK filter state validity failed for {label}: state invalid")
        return valid

    def _publish_filtered(
        self,
        original: GraspCandidateArray,
        reachable: list[bool],
        reachable_targets: list[tuple[PoseTarget, PoseTarget]] | None = None,
    ) -> None:
        filtered = filter_candidate_array_by_reachability(original, reachable)
        self._candidates_pub.publish(filtered)
        plan = self._plan_from_filtered(filtered, reachable_targets or [])
        self._plan_pub.publish(plan)

    def _plan_from_filtered(
        self,
        filtered: GraspCandidateArray,
        reachable_targets: list[tuple[PoseTarget, PoseTarget]],
    ) -> GraspPlan:
        plan = GraspPlan()
        plan.header = filtered.header
        plan.source = "candidate_ik_filter"
        if filtered.best_index < 0 or not filtered.candidates:
            plan.valid = False
            plan.reason = "no IK-reachable grasp candidates"
            return plan
        candidate = filtered.candidates[int(filtered.best_index)]
        plan.candidate = candidate
        if reachable_targets:
            pregrasp, grasp = reachable_targets[int(filtered.best_index)]
            plan.pregrasp_pose = _pose_from_target(pregrasp)
            plan.grasp_pose = _pose_from_target(grasp)
            plan.header.frame_id = self._target_frame
        else:
            plan.grasp_pose = candidate.pose
            plan.pregrasp_pose = candidate.pose
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.reason = ""
        return plan


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CandidateIkFilterNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
