"""Regression cases found by the end-to-end functional test round."""
from concurrent.futures import Future
from types import SimpleNamespace
import time

import pytest
from sensor_msgs.msg import JointState
from rebotarm_teach.record_joint_selection import select_record_joints
from rebotarm_teleop.web_execute_session import WebExecuteSession
from rebotarm_motion.moveit_planner import MoveItMotionPlanner


def test_recording_reorders_six_axes_and_excludes_fingers():
    names = ['right_finger_joint', 'joint3', 'joint1', 'joint2', 'joint6', 'joint4', 'joint5', 'left_finger_joint']
    message = JointState(name=names, position=[float(i) for i in range(8)], velocity=[float(i+10) for i in range(8)])
    values = select_record_joints(message, tuple(f'joint{i}' for i in range(1,7)))
    assert values == ((2.,3.,1.,5.,6.,4.), (12.,13.,11.,15.,16.,14.), ())


@pytest.mark.parametrize('fault', ['missing','duplicate','nonfinite','short_velocity'])
def test_bad_record_state_is_rejected(fault):
    message = JointState(name=['joint1','joint2'], position=[0.,0.], velocity=[0.,0.])
    if fault=='missing': message.name=['joint1','gripper']
    if fault=='duplicate': message.name=['joint1','joint1']
    if fault=='nonfinite': message.position=[float('nan'),0.]
    if fault=='short_velocity': message.velocity=[0.]
    with pytest.raises(ValueError): select_record_joints(message, ('joint1','joint2'))


class Handle:
    accepted=True
    def __init__(self): self.result=Future(); self.cancel=Future(); self.cancel_calls=0
    def get_result_async(self): return self.result
    def cancel_goal_async(self): self.cancel_calls+=1; return self.cancel


class Client:
    def stop(self,handle,**kwargs):
        return dict(accepted=True,state='cancel_requested',message='stopping',
                    trajectory_stop_requested=True,status={'state':'cancel_requested'},
                    cancel_future=handle.cancel_goal_async() if handle else None, clear_goal_handle=True)


def session():
    statuses=[]
    instance=WebExecuteSession(client=Client(),trajectory_stop_client=None,status_sink=statuses.append,
                               max_joint_speed_source=lambda:1.,successful_result_code=0)
    return instance,statuses


def observe(instance,handle=None):
    handle=handle or Handle()
    future=Future()
    decision=SimpleNamespace(max_delta=0.1,max_delta_limit=1.,duration=8.)
    instance.observe_execution(dict(accepted=True,goal_future=future,decision=decision,status={'state':'active'}))
    return future,handle


@pytest.mark.parametrize('status,code,expected',[(5,0,'canceled'),(6,0,'failed'),(4,0,'done'),(4,-1,'failed')])
def test_terminal_action_state_takes_precedence_over_error_code(status,code,expected):
    instance,statuses=session(); future,handle=observe(instance); future.set_result(handle)
    handle.result.set_result(SimpleNamespace(status=status,result=SimpleNamespace(error_code=code,error_string='test')))
    assert statuses[-1]['state']==expected


def test_late_cancel_response_cannot_overwrite_canceled_result():
    instance,statuses=session(); future,handle=observe(instance); future.set_result(handle)
    instance.stop()
    handle.result.set_result(SimpleNamespace(status=5,result=SimpleNamespace(error_code=0,error_string='stopped')))
    handle.cancel.set_result(SimpleNamespace(goals_canceling=[]))
    assert statuses[-1]['state']=='canceled'


def test_unconfirmed_cancel_does_not_claim_success():
    instance,statuses=session(); future,handle=observe(instance); future.set_result(handle)
    instance.stop(); handle.cancel.set_result(SimpleNamespace(goals_canceling=[]))
    assert statuses[-1]['state']=='cancel_unconfirmed'


def test_service_stop_terminal_is_stopped_not_completed():
    instance,statuses=session(); future,handle=observe(instance); future.set_result(handle)
    instance.stop()
    handle.result.set_result(SimpleNamespace(status=6,result=SimpleNamespace(
        error_code=-1,error_string='simulation trajectory stopped by service')))
    assert statuses[-1]['state']=='stopped'
    handle.cancel.set_result(SimpleNamespace(goals_canceling=[]))
    assert statuses[-1]['state']=='stopped'


def test_actual_failure_after_stop_request_is_not_disguised_as_stopped():
    instance,statuses=session(); future,handle=observe(instance); future.set_result(handle)
    instance.stop()
    handle.result.set_result(SimpleNamespace(status=6,result=SimpleNamespace(
        error_code=-1,error_string='hardware feedback unavailable')))
    assert statuses[-1]['state']=='failed'


def test_stop_while_goal_is_pending_cancels_when_accepted():
    instance,statuses=session(); future,handle=observe(instance)
    instance.stop(); future.set_result(handle)
    assert handle.cancel_calls==1
    assert statuses[-1]['state']=='cancel_requested'


def test_immediately_completed_goal_does_not_regress_to_active():
    instance,statuses=session(); handle=Handle()
    handle.result.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=0,error_string='done')))
    future=Future();future.set_result(handle)
    instance.observe_execution(dict(accepted=True,goal_future=future,decision=SimpleNamespace(max_delta=0.,max_delta_limit=1.,duration=1.),status={'state':'active'}))
    assert statuses[-1]['state']=='done'


def test_old_action_callbacks_cannot_overwrite_new_execution():
    instance,statuses=session()
    first,old=observe(instance);first.set_result(old)
    second,current=observe(instance);second.set_result(current)
    old.result.set_result(SimpleNamespace(status=5,result=SimpleNamespace(error_code=0,error_string='old canceled')))
    assert statuses[-1]['state']=='accepted'
    assert instance._goal_handle is current


def test_teach_cancel_response_does_not_fake_completion_or_overwrite_terminal():
    from rebotarm_teach.teach_replay_session import TeachReplaySession
    states=[]
    instance=TeachReplaySession(action_client=None,trajectory_stop_client=None,goal_factory=lambda:None,
                               status_sink=states.append,status_source=lambda:{})
    response=Future();response.set_result(SimpleNamespace(goals_canceling=[]))
    instance._on_cancel_response(response,0)
    assert states[-1]['state']=='cancel_unconfirmed'
    instance._terminal=True
    states.append({'state':'canceled'})
    instance._on_cancel_response(response,0)
    assert states[-1]['state']=='canceled'


def test_console_scripts_use_active_project_python():
    import sys
    from pathlib import Path
    from ament_index_python.packages import get_package_prefix
    for package,executable in [('rebotarm_simulation','rebotarm_mujoco_node'),('rebotarm_vision','rebotarm_graspnet_baseline_node')]:
        path=Path(get_package_prefix(package))/'lib'/package/executable
        assert path.read_text().splitlines()[0]=='#!'+sys.executable


def test_planning_timeout_uses_monotonic_clock(monkeypatch):
    clock=iter([0.,0.,0.5,1.01])
    monkeypatch.setattr('rebotarm_motion.moveit_planner.time.monotonic',lambda:next(clock))
    monkeypatch.setattr('rebotarm_motion.moveit_planner.time.sleep',lambda _duration:None)
    planner=object.__new__(MoveItMotionPlanner)
    planner._planning_time=0.
    # No ROS clock is supplied: paused /clock must not prevent timeout.
    planner._spin_until_future(Future())


def test_replay_accepted_after_stop_is_canceled():
    from rebotarm_teach.teach_replay_node import TeachReplayNode
    handle=Handle()
    future=Future();future.set_result(handle)
    node=SimpleNamespace(_stop_requested=True,_publish_status=lambda *args:None,
                         _on_replay_result=lambda _future:None)
    TeachReplayNode._on_goal_response(node,future,SimpleNamespace(points=[]))
    assert handle.cancel_calls==1


def test_service_response_can_run_while_planning_timer_waits():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from moveit_msgs.srv import GetMotionPlan
    rclpy.init(args=[])
    server=rclpy.create_node('planning_service_regression')
    client=rclpy.create_node('planning_client_regression')
    def reply(_request,response):
        from trajectory_msgs.msg import JointTrajectoryPoint
        response.motion_plan_response.error_code.val=1
        response.motion_plan_response.trajectory.joint_trajectory.joint_names=['joint1']
        response.motion_plan_response.trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.])]
        return response
    server.create_service(GetMotionPlan,'/regression/plan',reply)
    planner=MoveItMotionPlanner(client,group_name='arm',ee_frame_id='end_link',frame_id='base_link',
        planning_service='/regression/plan',planning_pipeline='ompl',planner_id='',planning_time=1.,
        num_attempts=1,goal_position_tolerance=0.01,goal_orientation_tolerance=0.01)
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(server);executor.add_node(client)
    result=[]
    def request():
        timer.cancel()
        result.append(planner._call_plan_service(GetMotionPlan.Request()))
    timer=client.create_timer(0.05,request)
    try:
        assert planner._client.wait_for_service(timeout_sec=3.)
        deadline=time.monotonic()+4
        while not result and time.monotonic()<deadline: executor.spin_once(timeout_sec=0.05)
        assert result and result[0].success
    finally:
        executor.shutdown();client.destroy_node();server.destroy_node();rclpy.shutdown()
