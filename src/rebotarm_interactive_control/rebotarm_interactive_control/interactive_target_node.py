from __future__ import annotations

from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.srv import SetMode
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .command_models import PoseTarget
from .execution_coordinator import InteractiveCoordinator
from .mode_manager import parse_control_mode
from .parameter_helpers import build_joint_limits, sensor_qos_kwargs
from .pose_math import quaternion_to_rpy, rpy_to_quaternion
from .pose_preview_solver import PosePreviewSolver
from .preview_manager import PreviewManager


class InteractiveTargetNode(Node):
    """Phase-1 coordination node: preview inputs now, delegate execution only on demand."""

    def __init__(self) -> None:
        super().__init__("interactive_control")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("mode", "simulation")
        self.declare_parameter("default_move_duration", 2.0)
        self.declare_parameter("marker_frame_id", "base_link")
        self.declare_parameter(
            "interactive_marker_namespace",
            "rebotarm/interactive_control/ee_target",
        )
        self.declare_parameter("interactive_marker_name", "ee_target")
        self.declare_parameter("interactive_marker_scale", 0.28)
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "joint_lower_limits",
            [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._default_duration = float(self.get_parameter("default_move_duration").value)
        self._marker_frame_id = str(self.get_parameter("marker_frame_id").value)
        self._marker_namespace = str(
            self.get_parameter("interactive_marker_namespace").value
        ).strip("/")
        self._marker_name = str(self.get_parameter("interactive_marker_name").value)
        self._marker_scale = float(self.get_parameter("interactive_marker_scale").value)

        joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        lower_limits = tuple(
            float(v) for v in self.get_parameter("joint_lower_limits").value
        )
        upper_limits = tuple(
            float(v) for v in self.get_parameter("joint_upper_limits").value
        )
        joint_limits = build_joint_limits(
            joint_names=joint_names,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )

        workspace_root = Path(__file__).resolve().parents[3]
        pose_solver = None
        try:
            pose_solver = PosePreviewSolver(workspace_root=workspace_root)
            self.get_logger().info("pose preview solver enabled")
        except Exception as exc:  # pragma: no cover - depends on local ROS/Python env
            self.get_logger().warn(f"pose preview solver disabled: {exc}")

        preview_manager = PreviewManager(
            joint_names=joint_names,
            joint_limits=joint_limits,
            initial_positions=tuple(0.0 for _ in joint_names),
            pose_solver=pose_solver,
        )
        self._pose_solver = pose_solver
        self._coordinator = InteractiveCoordinator(
            preview_manager=preview_manager,
            default_mode=parse_control_mode(str(self.get_parameter("mode").value)),
        )
        self._interactive_server = None
        self._interactive_classes = None

        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/interactive_control/status",
            10,
        )
        self._preview_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/interactive_control/preview",
            10,
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointTrajectory,
            f"/{self._arm_namespace}/interactive_control/joint_target",
            self._on_joint_target,
            10,
        )
        self.create_subscription(
            Pose,
            f"/{self._arm_namespace}/interactive_control/pose_target",
            self._on_pose_target,
            10,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )

        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/execute_preview",
            self._execute_preview,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/estop",
            self._trigger_estop,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/interactive_control/reset_estop",
            self._reset_estop,
        )
        self.create_service(
            SetMode,
            f"/{self._arm_namespace}/interactive_control/set_mode",
            self._set_mode,
        )
        self._setup_interactive_marker()

        self.get_logger().info(
            f"interactive control ready: namespace=/{self._arm_namespace}, "
            f"mode={self._coordinator.mode.value}"
        )
        self._publish_status("interactive control idle")

    def preview_joint_target(self, joint_targets: dict[str, float]) -> None:
        preview = self._coordinator.preview_joint_target(joint_targets)
        self._publish_preview(preview.message, preview.joint_positions)
        self._sync_marker_to_current_pose_if_idle()

    def preview_pose_target(self, pose_target: PoseTarget) -> None:
        preview = self._coordinator.preview_pose_target(pose_target)
        self._publish_preview(preview.message, preview.joint_positions)
        if preview.reachable:
            self._set_marker_pose_from_pose_target(pose_target)

    def _on_joint_target(self, msg: JointTrajectory) -> None:
        if not msg.joint_names or not msg.points:
            self._publish_status("ignored empty joint target")
            return
        point = msg.points[-1]
        if len(point.positions) != len(msg.joint_names):
            self._publish_status("ignored joint target with mismatched positions")
            return
        joint_targets = {
            str(name): float(value)
            for name, value in zip(msg.joint_names, point.positions)
        }
        self.preview_joint_target(joint_targets)

    def _on_pose_target(self, msg: Pose) -> None:
        roll, pitch, yaw = quaternion_to_rpy(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        pose_target = PoseTarget(
            x=float(msg.position.x),
            y=float(msg.position.y),
            z=float(msg.position.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        self.preview_pose_target(pose_target)

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        self._coordinator._preview_manager.sync_current_positions(  # noqa: SLF001
            {
                str(name): float(pos)
                for name, pos in zip(msg.name, msg.position)
            }
        )
        last_preview = self._coordinator.last_preview
        if last_preview is None or last_preview.command_type != "pose":
            self._sync_marker_to_current_pose_if_idle()

    def _execute_preview(self, _request, response):
        decision = self._coordinator.execute_preview(duration=self._default_duration)
        response.success = bool(decision.accepted)
        response.message = decision.message
        if not decision.accepted or decision.request is None:
            self._publish_status(decision.message)
            return response

        request = decision.request
        if request.preview_only:
            self._publish_status(
                "simulation preview accepted: "
                + ", ".join(f"{v:.3f}" for v in request.joint_positions)
            )
            self._coordinator.execution_finished()
            return response

        if not self._trajectory_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = "follow_joint_trajectory action unavailable"
            self._coordinator.execution_finished()
            self._publish_status(response.message)
            return response

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(request.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(request.joint_positions)
        secs = max(0.0, float(request.duration))
        point.time_from_start.sec = int(secs)
        point.time_from_start.nanosec = int((secs - int(secs)) * 1e9)
        goal.trajectory.points = [point]

        send_future = self._trajectory_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        self._publish_status("real execution goal sent")
        return response

    def _set_mode(self, request, response):
        try:
            mode = parse_control_mode(request.mode)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            self._publish_status(response.message)
            return response

        self._coordinator.set_mode(mode)
        response.success = True
        response.message = f"interactive mode set to {mode.value}"
        self._publish_status(response.message)
        return response

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - ROS future path
            self._coordinator.execution_finished()
            self._publish_status(f"failed to send trajectory goal: {exc}")
            return
        if not goal_handle.accepted:
            self._coordinator.execution_finished()
            self._publish_status("trajectory goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._coordinator.execution_finished()
        try:
            wrapped_result = future.result()
            status = getattr(wrapped_result, "status", "unknown")
            result = getattr(wrapped_result, "result", None)
            message = getattr(result, "error_string", "trajectory finished")
            self._publish_status(f"trajectory result status={status}: {message}")
        except Exception as exc:  # pragma: no cover - ROS future path
            self._publish_status(f"trajectory result retrieval failed: {exc}")
        self._sync_marker_to_current_pose_if_idle()

    def _trigger_estop(self, _request, response):
        self._coordinator.trigger_estop()
        response.success = True
        response.message = "interactive estop latched"
        self._publish_status(response.message)
        return response

    def _reset_estop(self, _request, response):
        self._coordinator.reset_estop()
        response.success = True
        response.message = "interactive estop reset"
        self._publish_status(response.message)
        return response

    def _publish_preview(self, message: str, joint_positions: tuple[float, ...]) -> None:
        payload = (
            f"state={self._coordinator.execution_state.value}; "
            f"message={message}; "
            f"joints={[round(v, 6) for v in joint_positions]}"
        )
        msg = String()
        msg.data = payload
        self._preview_pub.publish(msg)

    def _publish_status(self, message: str) -> None:
        payload = (
            f"mode={self._coordinator.mode.value}; "
            f"state={self._coordinator.execution_state.value}; "
            f"message={message}"
        )
        msg = String()
        msg.data = payload
        self._status_pub.publish(msg)

    def _setup_interactive_marker(self) -> None:
        try:
            from interactive_markers.interactive_marker_server import (  # pylint: disable=import-outside-toplevel
                InteractiveMarkerServer,
            )
            from visualization_msgs.msg import (  # pylint: disable=import-outside-toplevel
                InteractiveMarker,
                InteractiveMarkerControl,
                InteractiveMarkerFeedback,
                Marker,
            )
        except Exception as exc:  # pragma: no cover - depends on ROS env
            self.get_logger().warn(f"interactive marker disabled: {exc}")
            return

        self._interactive_classes = (
            InteractiveMarker,
            InteractiveMarkerControl,
            InteractiveMarkerFeedback,
            Marker,
        )
        self._interactive_server = InteractiveMarkerServer(self, self._marker_namespace)

        marker = self._build_interactive_marker(self._default_pose_message())
        self._interactive_server.insert(marker, feedback_callback=self._on_marker_feedback)
        self._interactive_server.applyChanges()
        self.get_logger().info(
            f"interactive marker ready: /{self._marker_namespace}/update"
        )

    def _build_interactive_marker(self, pose: Pose):
        assert self._interactive_classes is not None
        (
            InteractiveMarker,
            InteractiveMarkerControl,
            _InteractiveMarkerFeedback,
            Marker,
        ) = self._interactive_classes

        marker = InteractiveMarker()
        marker.header.frame_id = self._marker_frame_id
        marker.name = self._marker_name
        marker.description = "reBotArm EE Target"
        marker.scale = self._marker_scale
        marker.pose = pose

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.interaction_mode = InteractiveMarkerControl.NONE
        visual_control.markers.extend(self._make_visible_markers(Marker))
        marker.controls.append(visual_control)

        for name, orientation, mode in (
            ("move_plane", (1.0, 0.0, 1.0, 0.0), InteractiveMarkerControl.MOVE_PLANE),
            ("move_x", (1.0, 1.0, 0.0, 0.0), InteractiveMarkerControl.MOVE_AXIS),
            ("move_z", (1.0, 0.0, 1.0, 0.0), InteractiveMarkerControl.MOVE_AXIS),
            ("rotate_z", (1.0, 0.0, 1.0, 0.0), InteractiveMarkerControl.ROTATE_AXIS),
        ):
            control = InteractiveMarkerControl()
            control.name = name
            control.orientation.w = orientation[0]
            control.orientation.x = orientation[1]
            control.orientation.y = orientation[2]
            control.orientation.z = orientation[3]
            control.interaction_mode = mode
            marker.controls.append(control)

        return marker

    def _make_visible_markers(self, marker_cls):
        markers = []

        center = marker_cls()
        center.type = marker_cls.SPHERE
        center.scale.x = self._marker_scale * 0.7
        center.scale.y = self._marker_scale * 0.7
        center.scale.z = self._marker_scale * 0.7
        center.color.r = 1.0
        center.color.g = 0.85
        center.color.b = 0.10
        center.color.a = 1.0
        markers.append(center)

        label = marker_cls()
        label.type = marker_cls.TEXT_VIEW_FACING
        label.text = "EE Target"
        label.scale.z = self._marker_scale * 0.35
        label.pose.position.z = self._marker_scale * 0.7
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        markers.append(label)

        axis_length = self._marker_scale * 1.1
        axis_shaft = self._marker_scale * 0.15
        axis_head = self._marker_scale * 0.24
        axis_offset = axis_length * 0.5

        for axis_name, rgba, position, orientation in (
            (
                "x",
                (0.95, 0.25, 0.25, 0.95),
                (axis_offset, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            ),
            (
                "y",
                (0.20, 0.85, 0.25, 0.95),
                (0.0, axis_offset, 0.0),
                (0.0, 0.0, 0.70710678, 0.70710678),
            ),
            (
                "z",
                (0.20, 0.45, 0.95, 0.95),
                (0.0, 0.0, axis_offset),
                (0.0, 0.70710678, 0.0, 0.70710678),
            ),
        ):
            axis = marker_cls()
            axis.ns = "ee_target_axes"
            axis.id = ord(axis_name)
            axis.type = marker_cls.ARROW
            axis.scale.x = axis_length
            axis.scale.y = axis_shaft
            axis.scale.z = axis_head
            axis.pose.position.x = position[0]
            axis.pose.position.y = position[1]
            axis.pose.position.z = position[2]
            axis.pose.orientation.x = orientation[0]
            axis.pose.orientation.y = orientation[1]
            axis.pose.orientation.z = orientation[2]
            axis.pose.orientation.w = orientation[3]
            axis.color.r = rgba[0]
            axis.color.g = rgba[1]
            axis.color.b = rgba[2]
            axis.color.a = rgba[3]
            markers.append(axis)

        return markers

    def _on_marker_feedback(self, feedback) -> None:
        if self._interactive_classes is None:
            return
        _InteractiveMarker, _InteractiveMarkerControl, InteractiveMarkerFeedback, _Marker = (
            self._interactive_classes
        )
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        pose_target = self._pose_target_from_pose(feedback.pose)
        self.preview_pose_target(pose_target)

    def _pose_target_from_pose(self, pose: Pose) -> PoseTarget:
        roll, pitch, yaw = quaternion_to_rpy(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        return PoseTarget(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def _default_pose_message(self) -> Pose:
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.w = 1.0
        return pose

    def _set_marker_pose_from_pose_target(self, pose_target: PoseTarget) -> None:
        if self._interactive_server is None:
            return
        pose = Pose()
        pose.position.x = pose_target.x
        pose.position.y = pose_target.y
        pose.position.z = pose_target.z
        qx, qy, qz, qw = rpy_to_quaternion(
            pose_target.roll,
            pose_target.pitch,
            pose_target.yaw,
        )
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self._interactive_server.setPose(self._marker_name, pose)
        self._interactive_server.applyChanges()

    def _sync_marker_to_current_pose_if_idle(self) -> None:
        if self._interactive_server is None or self._pose_solver is None:
            return
        last_preview = self._coordinator.last_preview
        if last_preview is not None and last_preview.command_type == "pose":
            return
        try:
            pose_target = self._pose_solver.compute_pose_target(
                self._coordinator._preview_manager.current_positions  # noqa: SLF001
            )
        except Exception as exc:  # pragma: no cover - depends on local SDK/ROS env
            self.get_logger().warn(f"failed to sync marker pose: {exc}")
            return
        self._set_marker_pose_from_pose_target(pose_target)

    def destroy_node(self):
        if self._interactive_server is not None:
            try:
                self._interactive_server.shutdown()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractiveTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
