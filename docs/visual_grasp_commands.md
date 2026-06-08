# reBotArm 视觉抓取启动手册

本文档只保留当前推荐使用的视觉抓取命令。以后只要修改了视觉抓取相关配置、launch 参数、相机参数或 Windows AI 脚本，都要同步更新本文件�?
当前主要目标是平躺物体抓取调试：

```text
Windows Gemini2/YOLO/GraspNet
-> Ubuntu ROS2 真实视觉数据
-> candidate_ik_filter
-> /grasp/filtered_plan
-> 真机抓取或独立仿真抓�?```

## 0. 当前关键配置

视觉相机配置�?
```text
彩色: 1280x720 @ 30fps, MJPG
深度: 1280x800 @ 30fps, 优先 Y16/Y14 采集, D2C/HW align 后用 16 �?depth.png 传输
depth_downsample_filter: 1
Ubuntu ros.show_preview: false
Ubuntu ros.publish_annotated: false
```

当前 D2C 对齐后使�?color intrinsic�?
```text
fx=�� camera_info.json ʵʱ���Ϊ׼
fy=�� camera_info.json ʵʱ���Ϊ׼
cx=�� camera_info.json ʵʱ���Ϊ׼
cy=�� camera_info.json ʵʱ���Ϊ׼
width=1280
height=720
```

GraspNet/平躺物体默认策略�?
```text
GraspNet max_grasps: 10
candidate_pose_policy: preserve_candidate_pose
candidate_max_candidates_per_frame: 20
candidate_max_joint6_delta_rad: 1.5708
candidate_joint6_symmetry_enabled: true
candidate_joint6_symmetry_angle_rad: 3.141592653589793
candidate_workspace_gate_enabled: true
tcp_offset_xyz: [0.0, 0.0, 0.0]
gripper_grasp_enabled:=false  # 仿真测试时关闭真实夹爪动�?```

visual_ready 姿态：

```text
[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]
```

## 1. Windows 启动 YOLO 相机服务

Windows PowerShell 终端 1�?
```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main\softare\reBotArmController_ROS2-main"

.\tools\windows_start_yolo_server.ps1
```

启动后检查：

```powershell
curl http://127.0.0.1:8081/camera_info.json
```

Windows 浏览器查看画面：

```text
http://127.0.0.1:8081/video.mjpg
http://127.0.0.1:8081/annotated.mjpg
http://127.0.0.1:8081/depth.png
http://127.0.0.1:8081/camera_info.json
```

## 2. Windows 启动 GraspNet

Windows PowerShell 终端 2�?
```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main\softare\reBotArmController_ROS2-main"

.\tools\windows_start_graspnet_bridge.ps1
```

检�?GraspNet JSON�?
```powershell
curl http://127.0.0.1:8081/graspnet_candidates.json
```

## 3. Ubuntu 清理旧进�?
如果之前启动过视觉抓取，先清理残留节点。确认机械臂安全静止后执行：

```bash
pkill -f rebotarm_table_collision || true
pkill -f rebotarm_visual_ready || true
pkill -f rebotarm_visual_grasp_markers || true
pkill -f rebotarm_ordinary_grasp_node || true
pkill -f rebotarm_grasp_tcp_frame || true
pkill -f rebotarm_grasp_candidate_ik_filter || true
pkill -f rebotarm_visual_grasp_executor || true
pkill -f rebotarm_motion_execution_node || true
pkill -f rebotarm_sim_trajectory_controller || true
pkill -f GripperVisualJointStateNode || true
pkill -f reBotArmController || true
pkill -f move_group || true

rm -f /dev/shm/fastrtps_port*
```

## 4. 真机保持 visual_ready

这个终端负责让真实机械臂�?`visual_ready` 并保持电机使能。到位后不要�?`Ctrl+C`�?
Ubuntu 终端 A�?
```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup visual_ready_hold.launch.py
```

如果需要手动再次回�?visual_ready�?
```bash
ros2 service call /rebotarm/visual_ready/move std_srvs/srv/Trigger "{}"
```

## 5. 真实视觉 + 仿真抓取 benchmark

本节是“真实视�?+ 独立仿真执行”模式�?
这个模式用于�?RViz 里的仿真机械臂如何抓取。真实机械臂保持在终�?A �?visual_ready，仿真链路使用独�?namespace�?
```text
/rebotarm_sim
```

Ubuntu 终端 B�?
```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup real_perception_sim_execution.launch.py
```

执行一次仿真抓取并�?RViz 里看动作。每轮结束后会回�?visual_ready�?
Ubuntu 终端 C�?
```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarm_vision rebotarm_hybrid_grasp_sim_benchmark \
  --namespace rebotarm_sim \
  --attempts 1 \
  --min-success-rate 0 \
  --plan-timeout-sec 10 \
  --service-timeout-sec 180 \
  --return-ready-after-each \
  --wait-enter
```

连续 20 次仿�?benchmark�?
```bash
ros2 run rebotarm_vision rebotarm_hybrid_grasp_sim_benchmark \
  --namespace rebotarm_sim \
  --attempts 20 \
  --min-success-rate 95 \
  --plan-timeout-sec 10 \
  --service-timeout-sec 180 \
  --return-ready-after-each \
  --wait-enter
```

说明：这�?benchmark 会等待新�?`valid=true` `/grasp/filtered_plan`，然后调�?`/rebotarm_sim/visual_grasp/execute`。它测试的是真实视觉候选、IK、MoveIt 规划和仿真执行，不等于真实夹爪已经物理夹稳�?
## 6. 只看 GraspNet/IK 候选，不执行仿真动�?
如果只想�?`/grasp/filtered_plan` �?RViz marker，不需要仿真机械臂运动�?
```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py
```

查看当前最终计划：

```bash
ros2 topic echo /grasp/filtered_plan --once
```

查看 GraspNet 原始候选：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
```

## 7. 真机实际抓取

确认仿真姿态合理后，再进入真机实际抓取。这个命令会真实控制机械臂和夹爪�?
Ubuntu 终端 A�?
```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=execute \
  start_vision:=true \
  ordinary_depth_quality_enabled:=true \
  start_graspnet_baseline:=true \
  graspnet_source_mode:=network \
  graspnet_candidates_url:=http://192.168.145.1:8081/graspnet_candidates.json \
  graspnet_network_poll_hz:=0.5 \
  grasp_candidates_topic:=/grasp/graspnet_candidates \
  start_candidate_ik_filter:=true \
  candidate_pose_policy:=preserve_candidate_pose \
  candidate_max_candidates_per_frame:=20 \
  candidate_workspace_gate_enabled:=true \
  candidate_max_joint6_delta_rad:=1.5708 \
  candidate_joint6_symmetry_enabled:=true \
  candidate_joint6_symmetry_angle_rad:=3.141592653589793 \
  tcp_offset_xyz:="[0.0, 0.0, 0.0]" \
  executor_input_topic:=/grasp/filtered_plan \
  trajectory_precheck_enabled:=true \
  open_before_approach:=true \
  auto_gripper_width:=true \
  auto_gripper_effort:=true \
  gripper_grasp_enabled:=true \
  gripper_grasp_timeout_sec:=5.0 \
  safe_retreat_enabled:=true \
  safe_retreat_min_lift_z_m:=0.12 \
  lift_z_m:=0.04 \
  moveit_planning_time:=8.0 \
  moveit_num_planning_attempts:=5 \
  base_pregrasp_distance_m:=0.06 \
  safe_home_after_grasp:=false
```

执行一次真实抓取：

```bash
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

手动松开夹爪�?
```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.08, max_effort: 0.25}"
```

## 8. 连续真实抓取稳定性测�?
用于统计真实抓取成功率。每次按 Enter 前，手动摆放物体并确认安全�?
```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark \
  --attempts 20 \
  --service-timeout-sec 180 \
  --return-ready-before-each \
  --wait-enter
```

输出里关注：

```text
success_rate
failed_stage
move_to_pregrasp
approach_grasp
close_gripper
lift
```

## 9. 常用检查命�?
Ubuntu 检�?Windows 服务�?
```bash
curl http://192.168.145.1:8081/health
curl http://192.168.145.1:8081/camera_info.json
curl http://192.168.145.1:8081/graspnet_candidates.json
```

检�?ROS 图像尺寸�?
```bash
ros2 topic echo /camera/color/image_raw --once | grep -E "height|width|encoding|step"
ros2 topic echo /camera/depth/image_raw --once | grep -E "height|width|encoding|step"
```

检查候选和最终计划：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_plan --once
```

检查服务：

```bash
ros2 service list | grep -E "visual_ready|visual_grasp|motion_execution"
```

检查参数：

```bash
ros2 param get /rebotarm_grasp_candidate_ik_filter input_topic
ros2 param get /rebotarm_visual_grasp_executor input_topic
ros2 param get /rebotarm_visual_grasp_executor gripper_grasp_enabled
```

## 10. 维护规则

修改以下任意内容后，必须更新本文档：

```text
Windows YOLO/GraspNet 启动脚本
camera.yaml
visual_grasp_system.launch.py
visual_ready_hold.launch.py
real_perception_sim_execution.launch.py
visual_grasp_perception_preview.launch.py
GraspNet/IK/retreat/gripper/table safety 参数
benchmark 命令参数
```


