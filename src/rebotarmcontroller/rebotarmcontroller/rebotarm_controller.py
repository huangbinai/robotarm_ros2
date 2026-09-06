from __future__ import annotations

import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .mode_transition_policy import ModeTransitionConfig
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices
from .runtime_parameters import arm_namespace, command_arbitration, finite_rate_hz
from .teach_recorder import InternalTeachRecorder


class reBotArmController(Node):
    def __init__(self) -> None:
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("arm_config", "")
        self.declare_parameter("gripper_config", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("hardware_feedback_rate_hz", 50.0)
        self.declare_parameter("feedback_stale_timeout_sec", 0.15)
        self.declare_parameter("gripper_position_torque_cap_nm", 1.0)
        self.declare_parameter("gripper_position_max_speed_rad_s", 0.5)
        self.declare_parameter("gripper_position_timeout_margin_sec", 1.5)
        self.declare_parameter("grasp_hold_timeout_sec", 30.0)
        self.declare_parameter("gripper_contact_torque_min_nm", 0.0)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("shutdown_safe_home", True)
        self.declare_parameter(
            "trajectory_safety.position_min_rad",
            [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14],
        )
        self.declare_parameter(
            "trajectory_safety.position_max_rad",
            [2.8, 0.0, 0.0, 1.57, 1.57, 3.14],
        )
        self.declare_parameter(
            "trajectory_safety.max_velocity_rad_s",
            [3.0, 3.0, 3.0, 1.8, 1.8, 1.8],
        )
        self.declare_parameter(
            "trajectory_safety.max_acceleration_rad_s2",
            [5.0, 5.0, 5.0, 5.0, 5.0, 5.0],
        )
        self.declare_parameter("trajectory_safety.start_tolerance_rad", 0.10)
        self.declare_parameter("trajectory_safety.goal_tolerance_rad", 0.03)
        self.declare_parameter("trajectory_safety.settle_timeout_sec", 2.0)
        self.declare_parameter("trajectory_safety.sample_period_sec", 0.01)
        self.declare_parameter("teach_record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("teach_record_rate_hz", 150.0)
        self.declare_parameter("teach_record_require_gravity_comp", True)
        self.declare_parameter("mode_transition.enabled", True)
        self.declare_parameter("mode_transition.allow_velocity_mode", False)
        self.declare_parameter("mode_transition.enter.ramp_duration_sec", 0.35)
        self.declare_parameter("mode_transition.enter.max_start_velocity_rad_s", 0.05)
        self.declare_parameter("mode_transition.exit.damping_duration_sec", 0.15)
        self.declare_parameter("mode_transition.exit.blend_duration_sec", 0.35)
        self.declare_parameter("mode_transition.exit.max_lock_velocity_rad_s", 0.05)
        self.declare_parameter("mode_transition.exit.velocity_wait_timeout_sec", 1.0)
        self.declare_parameter("mode_transition.mit.gravity_kp", 7.0)
        self.declare_parameter("mode_transition.mit.gravity_kd", 0.8)
        self.declare_parameter("mode_transition.mit.hold_kp", 12.0)
        self.declare_parameter("mode_transition.mit.hold_kd", 1.2)
        self.declare_parameter("mode_transition.pos_vel.settle_duration_sec", 0.15)
        self.declare_parameter("mode_transition.safety.max_position_jump_rad", 0.02)
        self.declare_parameter("mode_transition.safety.feedback_timeout_sec", 0.10)
        self.declare_parameter("mode_transition.safety.transition_timeout_sec", 2.0)

        arm_config = self.get_parameter("arm_config").value or None
        gripper_config = self.get_parameter("gripper_config").value or None
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = arm_namespace(self.get_parameter("arm_namespace").value)
        joint_state_rate = finite_rate_hz(
            "joint_state_rate",
            self.get_parameter("joint_state_rate").value,
        )
        hardware_feedback_rate_hz = float(
            self.get_parameter("hardware_feedback_rate_hz").value
        )
        feedback_stale_timeout_sec = float(
            self.get_parameter("feedback_stale_timeout_sec").value
        )
        gripper_position_torque_cap_nm = float(
            self.get_parameter("gripper_position_torque_cap_nm").value
        )
        gripper_position_max_speed_rad_s = float(
            self.get_parameter("gripper_position_max_speed_rad_s").value
        )
        gripper_position_timeout_margin_sec = float(
            self.get_parameter("gripper_position_timeout_margin_sec").value
        )
        grasp_hold_timeout_sec = float(
            self.get_parameter("grasp_hold_timeout_sec").value
        )
        gripper_contact_torque_min_nm = float(
            self.get_parameter("gripper_contact_torque_min_nm").value
        )
        teach_record_path = str(self.get_parameter("teach_record_path").value)
        teach_record_rate_hz = finite_rate_hz(
            "teach_record_rate_hz",
            self.get_parameter("teach_record_rate_hz").value,
        )
        teach_record_require_gravity_comp = bool(
            self.get_parameter("teach_record_require_gravity_comp").value
        )
        cmd_arbitration = command_arbitration(
            self.get_parameter("cmd_arbitration").value
        )
        mode_transition_config = ModeTransitionConfig(
            enabled=bool(self.get_parameter("mode_transition.enabled").value),
            allow_velocity_mode=bool(
                self.get_parameter("mode_transition.allow_velocity_mode").value
            ),
            enter_ramp_duration_sec=float(
                self.get_parameter("mode_transition.enter.ramp_duration_sec").value
            ),
            enter_max_velocity_rad_s=float(
                self.get_parameter("mode_transition.enter.max_start_velocity_rad_s").value
            ),
            exit_damping_duration_sec=float(
                self.get_parameter("mode_transition.exit.damping_duration_sec").value
            ),
            exit_blend_duration_sec=float(
                self.get_parameter("mode_transition.exit.blend_duration_sec").value
            ),
            exit_max_lock_velocity_rad_s=float(
                self.get_parameter("mode_transition.exit.max_lock_velocity_rad_s").value
            ),
            exit_velocity_wait_timeout_sec=float(
                self.get_parameter("mode_transition.exit.velocity_wait_timeout_sec").value
            ),
            gravity_kp=float(self.get_parameter("mode_transition.mit.gravity_kp").value),
            gravity_kd=float(self.get_parameter("mode_transition.mit.gravity_kd").value),
            hold_kp=float(self.get_parameter("mode_transition.mit.hold_kp").value),
            hold_kd=float(self.get_parameter("mode_transition.mit.hold_kd").value),
            pos_vel_settle_duration_sec=float(
                self.get_parameter("mode_transition.pos_vel.settle_duration_sec").value
            ),
            max_position_jump_rad=float(
                self.get_parameter("mode_transition.safety.max_position_jump_rad").value
            ),
            feedback_timeout_sec=float(
                self.get_parameter("mode_transition.safety.feedback_timeout_sec").value
            ),
            transition_timeout_sec=float(
                self.get_parameter("mode_transition.safety.transition_timeout_sec").value
            ),
        )
        self.hardware = None
        self.joint_state_publisher = None
        self.arm_services = None
        self.teach_recorder = None
        self.arm_actions = None
        self.motor_passthrough = None
        self.hardware = HardwareManager(
            arm_cfg=arm_config,
            gripper_cfg=gripper_config,
            channel=channel,
            mode_transition_config=mode_transition_config,
            hardware_feedback_rate_hz=hardware_feedback_rate_hz,
            feedback_stale_timeout_sec=feedback_stale_timeout_sec,
            gripper_position_torque_cap_nm=gripper_position_torque_cap_nm,
            gripper_position_max_speed_rad_s=gripper_position_max_speed_rad_s,
            gripper_position_timeout_margin_sec=gripper_position_timeout_margin_sec,
            grasp_hold_timeout_sec=grasp_hold_timeout_sec,
            gripper_contact_torque_min_nm=gripper_contact_torque_min_nm,
        )
        try:
            self.hardware.connect()
        except Exception as exc:
            self.get_logger().error(f"hardware connect failed; disabled before exit: {exc}")
            raise

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.teach_recorder = InternalTeachRecorder(
            self,
            self.hardware,
            self.arm_namespace,
            record_path=teach_record_path,
            rate_hz=teach_record_rate_hz,
            require_gravity_comp=teach_record_require_gravity_comp,
        )
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}, "
            f"lifecycle={self.hardware.lifecycle_state}; explicit enable required"
        )

    def publish_arm_status(self) -> None:
        self.joint_state_publisher.publish_status()

    def shutdown(self) -> None:
        if self.teach_recorder is not None:
            self.teach_recorder.shutdown()
        if self.hardware is None:
            return
        if bool(self.get_parameter("shutdown_safe_home").value):
            self._conditional_safe_home_before_shutdown()
        if not self.hardware.shutdown():
            self.get_logger().error(
                "hardware shutdown finished without verified disable/disconnect; "
                f"errors={self.hardware.error_codes}"
            )

    def _conditional_safe_home_before_shutdown(self) -> None:
        if not (self.hardware is not None and self.hardware.connected and self.hardware.enabled):
            return
        initial_state = self.hardware.state_machine
        try:
            self.hardware.stop_active_motion()
        except Exception as exc:
            self.get_logger().error(f"shutdown trajectory stop/hold failed; disabling: {exc}")
            return

        allowed, reason = self._safe_home_shutdown_allowed(initial_state)
        if not allowed:
            self.get_logger().warn(f"shutdown safe_home skipped: {reason}; disabling")
            return

        try:
            self.get_logger().warn("shutdown requested: running conditional safe_home before disable")
            self.hardware.stop_gravity_compensation()
            self.hardware.ensure_pos_vel_control()
            if not self.hardware.safe_home():
                raise TimeoutError("shutdown safe_home timed out")
            self.get_logger().info("shutdown conditional safe_home complete")
        except Exception as exc:
            self.get_logger().error(f"shutdown safe_home failed; disabling anyway: {exc}")

    def _safe_home_shutdown_allowed(self, initial_state: str) -> tuple[bool, str]:
        if self.hardware is None:
            return False, "hardware unavailable"
        if initial_state in ("LOWLEVEL_STREAMING", "GRAVITY_COMP"):
            return False, f"unsafe controller state {initial_state}"
        if self.hardware.gripper_active:
            return False, f"gripper active in {self.hardware.gripper_mode}"
        try:
            positions, velocities, _effort = self.hardware.get_joint_state()
        except Exception as exc:
            return False, f"joint state unavailable: {exc}"
        expected = len(self.hardware.joint_names)
        if len(positions) != expected or len(velocities) != expected:
            return False, "joint state size mismatch"
        if not all(math.isfinite(float(value)) for value in positions):
            return False, "joint positions are not finite"
        if not all(math.isfinite(float(value)) for value in velocities):
            return False, "joint velocities are not finite"
        return True, "ok"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = reBotArmController()
        executor.add_node(node)
        executor.spin()
    finally:
        if node is not None:
            node.shutdown()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
