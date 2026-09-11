"""Live perception only: no robot, planner, or execution node."""
import os,json,sys,time,subprocess,signal,tempfile
from pathlib import Path
os.environ.update(ROS_DOMAIN_ID='185',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from rebotarm_msgs.msg import Detection2DArray,GraspCandidateArray
from rebotarm_vision.graspnet_viewer_node import GraspNetViewerNode
from rebotarm_vision.graspnet_visualization import Open3DGraspWindow
root=Path(__file__).resolve().parents[1]
(root/'.codex_tmp').mkdir(exist_ok=True)
out=Path(tempfile.mkdtemp(prefix='functional-vision-',dir=root/'.codex_tmp'))
rclpy.init(args=[])
probe=rclpy.create_node('live_vision_test_probe')
end=time.monotonic()+1
while time.monotonic()<end:rclpy.spin_once(probe,timeout_sec=0.1)
assert probe.get_node_names()==[probe.get_name()],probe.get_node_names()
latest={};counts={};candidate_counts=[];ages=[]
def observe(key):
    def callback(message):
        latest[key]=message;counts[key]=counts.get(key,0)+1
        if key=='candidates':
            candidate_counts.append(len(message.candidates))
            if message.candidates:ages.append((probe.get_clock().now().nanoseconds-(message.header.stamp.sec*10**9+message.header.stamp.nanosec))/1e9)
    return callback
for key,typ,topic,qos in [('color',Image,'/camera/color/image_raw',qos_profile_sensor_data),('depth',Image,'/camera/depth/image_raw',qos_profile_sensor_data),('info',CameraInfo,'/camera/depth/camera_info',qos_profile_sensor_data),('detections',Detection2DArray,'/grasp/detections',10),('candidates',GraspCandidateArray,'/grasp/graspnet_candidates',10)]:
    probe.create_subscription(typ,topic,observe(key),qos)
window=Open3DGraspWindow(save_directory=out,visible=False)
viewer=GraspNetViewerNode(window=window)
children=[];streams=[];saved=False;error=None
try:
    for label,cmd in [('camera-yolo',['ros2','launch','rebotarm_vision','vision.launch.py']),('graspnet',['ros2','run','rebotarm_vision','rebotarm_graspnet_baseline_node'])]:
        stream=(out/(label+'.log')).open('w');streams.append(stream)
        children.append(subprocess.Popen(cmd,stdout=stream,stderr=stream,start_new_session=True,cwd=root))
    deadline=time.monotonic()+22
    while time.monotonic()<deadline:
        rclpy.spin_once(probe,timeout_sec=0.02)
        rclpy.spin_once(viewer,timeout_sec=0.01)
        viewer.render_pending()
        if window._scene is not None and window._scene.grasps and not saved:
            window.save(out/'live-ros-open3d');saved=True
        if any(p.poll() is not None for p in children):raise RuntimeError('perception process exited')
    report={'counts':counts,'max_candidate_count':max(candidate_counts,default=0),'nonempty_candidate_messages':sum(c>0 for c in candidate_counts),'candidate_age_range_sec':[min(ages),max(ages)] if ages else None,'open3d_saved':saved}
    if 'info' in latest:
        info=latest['info'];report['camera_info']={'width':info.width,'height':info.height,'k':list(info.k),'frame':info.header.frame_id}
    if 'detections' in latest:report['last_classes']=[d.class_name for d in latest['detections'].detections]
except Exception as exc:
    report={'error':type(exc).__name__+': '+str(exc),'counts':counts}
finally:
    for process in children:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
    for process in children:
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    for stream in streams:stream.close()
    viewer.destroy_node();probe.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    (out/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('RESULTS:',out,flush=True)
    print(json.dumps(report,ensure_ascii=False),flush=True)
raise SystemExit(0 if saved and 'error' not in report and 0 < report.get('max_candidate_count',0) <= 5 else 1)
