"""Behavior and installed-resource checks for the package boundary migration.

No controller is instantiated and no launch is executed. ROS messages and
launch descriptions are used as data; all command transports are in-memory.
"""
from concurrent.futures import Future
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from rebotarm_msgs.srv import SetGripper
from sensor_msgs.msg import JointState

from rebotarm_dashboard.web_robot_assets import rewrite_package_mesh_uris
from rebotarm_simulation.simulation_config import SimulationConfig
from rebotarm_teach.teach_recorder_node import TeachRecorderNode
from rebotarm_teleop.web_gripper_client import WebGripperClient


def joint_state(seconds, position=0.1):
    message = JointState()
    message.header.stamp.sec = seconds
    message.name = ["joint1"]
    message.position = [position]
    message.velocity = [0.0]
    message.effort = [0.0]
    return message


def recorder_probe():
    return SimpleNamespace(
        _recording_active=True, _latest_joint_state=None, _require_gravity_comp=True,
        _arm_state="GRAVITY_COMP", _motor_status={}, _first_sample_stamp=None,
        _joint_names=("joint1",),
        _last_sample_stamp=None, _last_recorded_source_stamp_ns=None,
        _handle=io.StringIO(), _samples_written=0, _writing_state="", _last_write_error="",
        _publish_status=lambda *args: None,
    )


def test_recorder_does_not_invent_samples_on_timer_ticks():
    probe = recorder_probe()
    probe._latest_joint_state = joint_state(10)
    TeachRecorderNode._write_sample(probe)
    TeachRecorderNode._write_sample(probe)
    probe._latest_joint_state = joint_state(10, position=0.2)
    TeachRecorderNode._write_sample(probe)
    probe._latest_joint_state = joint_state(9)
    TeachRecorderNode._write_sample(probe)
    probe._latest_joint_state = joint_state(11)
    TeachRecorderNode._write_sample(probe)
    rows = [json.loads(line) for line in probe._handle.getvalue().splitlines()]
    assert [row["stamp"] for row in rows] == [10.0, 11.0]
    assert probe._samples_written == 2


def test_recorder_requires_recording_and_gravity_state():
    probe = recorder_probe()
    probe._latest_joint_state = joint_state(10)
    probe._arm_state = "IDLE"
    TeachRecorderNode._write_sample(probe)
    probe._arm_state = "GRAVITY_COMP"
    probe._recording_active = False
    TeachRecorderNode._write_sample(probe)
    assert probe._handle.getvalue() == ""
    probe._recording_active = True
    probe._arm_state = "IDLE"
    probe._require_gravity_comp = False
    TeachRecorderNode._write_sample(probe)
    assert probe._samples_written == 1


class ServiceTransport:
    def __init__(self, available=True):
        self.available = available
        self.future = Future()
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def send_gripper(client):
    return client.set_position(
        {"confirm": "SET_GRIPPER", "position": 0.05, "max_effort": 0.3}, use_hardware=False,
        gripper_limits=(0.0, 0.085), default_max_effort=0.3, max_effort_limit=1.0,
    )


@pytest.mark.parametrize("transport", [None, ServiceTransport(available=False)])
def test_sim_gripper_without_backend_is_not_reported_as_success(transport):
    client = WebGripperClient(
        action_client=None, goal_factory=None,
        sim_service_client=transport, sim_request_factory=SetGripper.Request,
    )
    result = send_gripper(client)
    assert not result["accepted"]
    assert result["status"]["state"] == "unavailable"


@pytest.mark.parametrize("success", [True, False])
def test_sim_gripper_result_comes_from_backend(success):
    transport = ServiceTransport()
    statuses = []
    client = WebGripperClient(
        action_client=None, goal_factory=None, status_sink=statuses.append,
        sim_service_client=transport, sim_request_factory=SetGripper.Request,
    )
    result = send_gripper(client)
    client.observe_result(result)
    assert result["status"]["state"] == "active"
    assert statuses == []
    assert transport.requests[0].position == 0.05
    response = SetGripper.Response()
    response.success = success
    response.reached_position = 0.04
    transport.future.set_result(response)
    assert statuses[-1]["state"] == ("done" if success else "failed")
    assert statuses[-1]["position"] == 0.04


def test_sim_gripper_transport_exception_is_failure():
    transport = ServiceTransport()
    statuses = []
    client = WebGripperClient(
        action_client=None, goal_factory=None, status_sink=statuses.append,
        sim_service_client=transport, sim_request_factory=SetGripper.Request,
    )
    result = send_gripper(client)
    client.observe_result(result)
    transport.future.set_exception(RuntimeError("backend disconnected"))
    assert statuses[-1]["state"] == "failed"


def test_real_gripper_keeps_action_transport():
    goals = []
    action = SimpleNamespace(
        wait_for_server=lambda **kwargs: True,
        send_goal_async=lambda goal: goals.append(goal) or Future(),
    )
    client = WebGripperClient(action_client=action, goal_factory=GripperCommand.Goal)
    result = client.set_position(
        {"confirm": "SET_GRIPPER", "position": 0.05}, use_hardware=True, gripper_limits=(0.0, 0.085),
        default_max_effort=0.3, max_effort_limit=1.0,
    )
    assert result["accepted"]
    assert goals[0].command.position == 0.05
    assert result["service_future"] is None


def launch_module(package, name):
    path = Path(get_package_share_directory(package)) / "launch" / f"{name}.launch.py"
    spec = importlib.util.spec_from_file_location(f"migration_{package}_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def composed_nodes(name, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    nodes = []

    def visit(entities, current):
        for action in entities:
            if action.condition is not None and not action.condition.evaluate(current):
                continue
            if isinstance(action, DeclareLaunchArgument):
                if action.name not in current.launch_configurations:
                    current.launch_configurations[action.name] = perform_substitutions(current, action.default_value)
            elif isinstance(action, Node):
                nodes.append((action.node_package, action.node_executable))
            elif isinstance(action, OpaqueFunction):
                visit(action.execute(current) or [], current)
            elif isinstance(action, GroupAction):
                child = LaunchContext()
                child.launch_configurations.update(current.launch_configurations)
                visit(action.get_sub_entities(), child)
            elif isinstance(action, IncludeLaunchDescription):
                child = LaunchContext()
                child.launch_configurations.update(current.launch_configurations)
                for key, value in action.launch_arguments:
                    child.launch_configurations[key] = perform_substitutions(
                        current, normalize_to_list_of_substitutions(value)
                    )
                visit(action.launch_description_source.get_launch_description(child).entities, child)

    visit(launch_module("rebotarm_bringup", name).generate_launch_description().entities, context)
    return nodes


@pytest.mark.parametrize("name,options,recorders,controllers,fakes", [
    ("rebotarm_app", {"channel": "can0", "use_rviz": "false"}, 1, 1, 0),
    ("teleop_system", {"channel": "can0", "use_hardware": "true"}, 1, 1, 0),
    ("teleop_system", {"use_hardware": "false"}, 1, 0, 0),
    ("rviz_ee_drag_real", {"channel": "can0"}, 0, 1, 0),
    ("rviz_ee_drag_sim", {}, 0, 0, 1),
    ("moveit_hardware", {"channel": "can0", "start_teach_recorder": "true"}, 1, 1, 0),
    ("teleop_system", {"use_hardware": "false", "start_teach_recorder": "true"}, 1, 0, 0),
])
def test_composed_launch_owns_recording_and_execution_once(name, options, recorders, controllers, fakes):
    nodes = composed_nodes(name, **options)
    assert nodes.count(("rebotarm_teach", "TeachRecorderNode")) == recorders
    assert nodes.count(("rebotarmcontroller", "reBotArmController")) == controllers
    assert nodes.count(("rebotarm_simulation", "rebotarm_rviz_fake_controller")) == fakes


def test_moveit_and_simulation_use_installed_shared_model():
    share = Path(get_package_share_directory("rebotarm_description"))
    canonical = share / "description/urdf/reBot-DevArm_fixend.urdf"
    model = ET.parse(canonical).getroot()
    for mesh in model.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        assert uri.startswith("package://rebotarm_description/")
        assert (share / uri.removeprefix("package://rebotarm_description/")).is_file()
    config = SimulationConfig.default()
    assert config.robot_urdf_path == canonical.resolve()
    assert config.arm_config_path == (share / "config/arm.yaml").resolve()
    assert config.gripper_config_path == (share / "config/gripper.yaml").resolve()
    # The former source copy is no longer required by the MoveIt builder.
    launch_module("rebotarm_moveit_config", "demo").generate_launch_description()
    assert "package://" not in rewrite_package_mesh_uris(canonical.read_text())


def test_shared_model_joint_limits_match_execution_boundaries():
    share = Path(get_package_share_directory("rebotarm_description"))
    model = ET.parse(share / "description/urdf/reBot-DevArm_fixend.urdf").getroot()
    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    limits = yaml.safe_load((bringup / "config/controller_safety.yaml").read_text())["reBotArmController"]["ros__parameters"]
    for index in range(6):
        joint = model.find(f"joint[@name='joint{index + 1}']/limit")
        assert float(joint.attrib["lower"]) == limits["trajectory_safety.position_min_rad"][index]
        assert float(joint.attrib["upper"]) == limits["trajectory_safety.position_max_rad"][index]
