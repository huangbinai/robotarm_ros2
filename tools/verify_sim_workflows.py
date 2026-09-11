"""Bounded functional probes in an empty ROS domain, simulation only."""
from pathlib import Path
import json,os,signal,subprocess,sys,tempfile,time
from urllib.request import Request,urlopen
from urllib.error import HTTPError

os.environ.update(ROS_DOMAIN_ID='184',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2')
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from rebotarm_simulation.mujoco_moveit_acceptance import run_acceptance

root=Path(__file__).resolve().parents[1]
(root/'.codex_tmp').mkdir(exist_ok=True)
out=Path(tempfile.mkdtemp(prefix='functional-sim-',dir=root/'.codex_tmp'))
record=out/'record.jsonl'
children=[]; streams=[]; results={}; latest={}; statuses=[]
rclpy.init(args=[])
node=rclpy.create_node('functional_test_probe')
node.create_subscription(JointState,'/rebotarm/joint_states',lambda msg:latest.update(message=msg),qos_profile_sensor_data)
node.create_subscription(String,'/rebotarm/teleop/replay_status',lambda msg:statuses.append(json.loads(msg.data)),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))

def spin_until(predicate,seconds=10):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate(): return True
        rclpy.spin_once(node,timeout_sec=0.05)
    return bool(predicate())

def start(label,command):
    stream=(out/(label+'.log')).open('w')
    streams.append(stream)
    process=subprocess.Popen(command,cwd=root,stdout=stream,stderr=stream,start_new_session=True)
    children.append(process)
    return process

def trigger(name):
    client=node.create_client(Trigger,name)
    if not client.wait_for_service(timeout_sec=5): return {'success':False,'message':'service unavailable'}
    future=client.call_async(Trigger.Request())
    if not spin_until(future.done,5): return {'success':False,'message':'timeout'}
    response=future.result()
    return {'success':response.success,'message':response.message}

def http(path,payload=None):
    data=json.dumps(payload).encode() if payload is not None else None
    request=Request('http://127.0.0.1:18088'+path,data=data,headers={'Content-Type':'application/json'})
    try:
        response=urlopen(request,timeout=15)
    except HTTPError as exc:
        return {'http_status':exc.code,'body':json.loads(exc.read())}
    with response:
        body=response.read()
        if 'json' in response.headers.get('Content-Type',''): return json.loads(body)
        return {'bytes':len(body),'status':response.status}

def step(name,operation):
    try:
        result=operation()
        results[name]=result
        print(name,json.dumps(result,ensure_ascii=False),flush=True)
    except Exception as exc:
        results[name]={'error':type(exc).__name__+': '+str(exc)}
        print(name,json.dumps(results[name],ensure_ascii=False),flush=True)

try:
    spin_until(lambda:False,1)
    assert node.get_node_names()==[node.get_name()],node.get_node_names()
    start('mujoco',['ros2','launch','rebotarm_simulation','mujoco_sim.launch.py','show_viewer:=false'])
    start('moveit',['ros2','launch','rebotarm_bringup','core.launch.py','hardware_mode:=sim','use_hardware:=false','use_moveit_preview:=true','use_moveit_fake_joint_states:=false','start_passive_joint_state_publisher:=false','use_local_rviz:=false','use_sim_time:=true'])
    start('recorder',['ros2','run','rebotarm_teach','TeachRecorderNode','--ros-args','-p','record_path:='+str(record),'-p','start_on_launch:=false','-p','keyboard_quit_enabled:=false','-p','require_gravity_comp:=false','-p','use_sim_time:=true'])
    planner=node.create_client(GetMotionPlan,'/plan_kinematic_path')
    assert planner.wait_for_service(timeout_sec=20),'MoveIt planning unavailable'
    assert spin_until(lambda:'message' in latest,10),'no simulator feedback'
    assert not any('Controller' in name for name in node.get_node_names()),node.get_node_names()
    results['ros_nodes']=node.get_node_names()
    step('record_start',lambda:trigger('/rebotarm/teleop/teach_record/start'))
    step('moveit_execution',lambda:run_acceptance(timeout=30))
    spin_until(lambda:False,1)
    step('record_stop',lambda:trigger('/rebotarm/teleop/teach_record/stop'))
    if record.exists():
        samples=[json.loads(line) for line in record.read_text().splitlines()]
        results['record_file']={'samples':len(samples),'joint_names':samples[0]['joint_names'] if samples else [],'path':str(record)}
    start('dashboard',['ros2','run','rebotarm_dashboard','TeleopStatusPanelNode','--ros-args','-p','use_hardware:=false','-p','execution_mode:=execute','-p','web_execute_enabled:=true','-p','port:=18088','-p','record_path:='+str(record)])
    def dashboard_ready():
        try: return len(http('/api/status').get('joints',{}))>=6
        except Exception:return False
    assert spin_until(dashboard_ready,15),'dashboard did not expose joint state'
    step('dashboard_page',lambda:http('/'))
    step('dashboard_config',lambda:http('/api/config'))
    step('dashboard_urdf',lambda:http('/robot/urdf'))
    step('web_gripper',lambda:http('/api/set_gripper',{'confirm':'SET_GRIPPER','position':0.05,'max_effort':0.3}))
    spin_until(lambda:False,2)
    step('web_status_after_gripper',lambda:http('/api/status'))
    message=latest['message']; target={name:float(pos) for name,pos in zip(message.name,message.position) if name.startswith('joint')}
    invalid=dict(target);invalid['joint1']=100.
    step('web_reject_invalid',lambda:http('/api/execute_preview',{'confirm':'EXECUTE','joint_positions':invalid,'duration':2.0}))
    target['joint1']+=0.02
    step('web_joint_execute',lambda:http('/api/execute_preview',{'confirm':'EXECUTE','joint_positions':target,'duration':2.0}))
    spin_until(lambda:False,4)
    step('web_status_after_execute',lambda:http('/api/status'))
    step('web_keyboard_enable',lambda:http('/api/keyboard_enable',{'step_rad':0.01,'duration':1.0}))
    step('web_keyboard_step',lambda:http('/api/keyboard_key',{'confirm':'KEYBOARD_TELEOP','key':'1'}))
    spin_until(lambda:False,2)
    step('web_keyboard_disable',lambda:http('/api/keyboard_disable',{}))
    long_target=dict(target);long_target['joint1']+=0.1
    step('web_stop_test_start',lambda:http('/api/execute_preview',{'confirm':'EXECUTE','joint_positions':long_target,'duration':8.0}))
    spin_until(lambda:False,0.5)
    step('web_stop',lambda:http('/api/stop_execute',{}))
    spin_until(lambda:False,1)
    stop_position=float(latest['message'].position[0])
    spin_until(lambda:False,1)
    results['stop_motion_observation']={'joint1_settled_delta_rad':abs(float(latest['message'].position[0])-stop_position),'target_distance_rad':abs(float(latest['message'].position[0])-long_target['joint1'])}
    step('web_status_after_stop',lambda:http('/api/status'))
    step('teach_web_dry_run',lambda:http('/api/teach_dry_run',{'record_path':str(record)}))
    replay=start('teach_dry_run',['ros2','launch','rebotarm_bringup','teach_replay.launch.py','record_path:='+str(record),'dry_run:=true'])
    spin_until(lambda:any(s.get('state') in ('failed','rejected','dry_run','blocked') for s in statuses) or replay.poll() is not None,25)
    results['teach_node_dry_run']={'statuses':statuses[-2:],'exit_code':replay.poll()}
    if any(s.get('state')=='dry_run' for s in statuses):
        os.killpg(replay.pid,signal.SIGINT);replay.wait(timeout=8)
        statuses.clear()
        execution=start('teach_replay',['ros2','launch','rebotarm_bringup','teach_replay.launch.py','record_path:='+str(record),'dry_run:=false'])
        spin_until(lambda:any(s.get('state') in ('failed','rejected','blocked','done') for s in statuses) or execution.poll() is not None,65)
        results['teach_node_execution']={'statuses':statuses[-2:],'exit_code':execution.poll()}
except Exception as exc:
    results['runner_error']=type(exc).__name__+': '+str(exc)
finally:
    for process in reversed(children):
        if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
    for process in children:
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    for stream in streams:stream.close()
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    (out/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
    print('RESULTS:',out,flush=True)
    print('RUNNER_ERROR:',results.get('runner_error'),flush=True)
checks = {
    'standard_launch_and_moveit': bool(results.get('moveit_execution',{}).get('ok')),
    'six_axis_recording': results.get('record_file',{}).get('joint_names') == [f'joint{i}' for i in range(1,7)],
    'web_gripper': results.get('web_status_after_gripper',{}).get('teleop',{}).get('web_gripper',{}).get('state') == 'done',
    'web_joint_motion': results.get('web_status_after_execute',{}).get('teleop',{}).get('web_execute',{}).get('state') == 'done',
    'keyboard_motion': bool(results.get('web_keyboard_step',{}).get('accepted')),
    'stop_terminal': results.get('web_status_after_stop',{}).get('teleop',{}).get('web_execute',{}).get('state') in ('stopped','canceled'),
    'stop_holds_before_target': results.get('stop_motion_observation',{}).get('joint1_settled_delta_rad',1.) < 0.01 and results.get('stop_motion_observation',{}).get('target_distance_rad',0.) > 0.04,
    'web_teach_precheck': bool(results.get('teach_web_dry_run',{}).get('accepted')),
    'node_teach_precheck': any(s.get('state')=='dry_run' for s in results.get('teach_node_dry_run',{}).get('statuses',[])),
    'node_teach_execution': any(s.get('state')=='done' for s in results.get('teach_node_execution',{}).get('statuses',[])),
}
results['checks']=checks
(out/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
print('CHECKS:',json.dumps(checks),flush=True)
raise SystemExit(0 if all(checks.values()) and 'runner_error' not in results else 1)
