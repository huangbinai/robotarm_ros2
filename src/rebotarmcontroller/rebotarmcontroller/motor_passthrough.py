from __future__ import annotations

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import JointMotorCmd


class MotorPassthrough:
    def __init__(self, node, hardware, namespace: str, arbitration: str) -> None:
        self._node = node
        self._hardware = hardware
        self._arbitration = arbitration
        self._arm_owner = "low_level_stream"
        self._gripper_owner = "low_level_gripper_stream"
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subscriptions = []
        for joint_name in hardware.joint_names:
            self._subscriptions.append(
                node.create_subscription(
                    JointMotorCmd,
                    f"/{namespace}/joints/{joint_name}/cmd",
                    self._make_joint_callback(joint_name),
                    qos,
                    callback_group=node.reentrant_group,
                )
            )
        if hardware.has_gripper:
            self._subscriptions.append(
                node.create_subscription(
                    JointMotorCmd,
                    f"/{namespace}/gripper/cmd",
                    self._gripper_callback,
                    qos,
                    callback_group=node.reentrant_group,
                )
            )

    def _make_joint_callback(self, joint_name: str):
        def _callback(msg: JointMotorCmd) -> None:
            owner = self._hardware.command_arbiter.owner("arm")
            if owner is not None and owner != self._arm_owner:
                if self._arbitration == "reject":
                    self._node.get_logger().warn(
                        f"rejecting /joints/{joint_name}/cmd while {owner} owns the arm"
                    )
                    return
                self._node.get_logger().warn(
                    f"preempting {owner} for /joints/{joint_name}/cmd"
                )
                self._hardware.stop_active_motion()
                self._hardware.command_arbiter.force_release("arm")

            lease = self._hardware.command_arbiter.acquire("arm", self._arm_owner)
            if lease is None:
                self._node.get_logger().warn(
                    f"rejecting /joints/{joint_name}/cmd because arm ownership changed"
                )
                return

            try:
                self._hardware.send_joint_motor_cmd(joint_name, msg)
            except Exception as exc:
                self._node.get_logger().warn(f"joint cmd failed for {joint_name}: {exc}")
            finally:
                self._node.publish_arm_status()

        return _callback

    def _gripper_callback(self, msg: JointMotorCmd) -> None:
        lease = self._hardware.command_arbiter.acquire(
            "gripper", self._gripper_owner
        )
        if lease is None:
            self._node.get_logger().warn(
                "rejecting /gripper/cmd while another gripper command is active"
            )
            return
        try:
            self._hardware.send_gripper_motor_cmd(msg)
        except Exception as exc:
            self._node.get_logger().warn(f"gripper cmd failed: {exc}")
        finally:
            self._node.publish_arm_status()
