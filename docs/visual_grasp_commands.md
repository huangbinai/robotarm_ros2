# reBotArm 视觉抓取使用手册

当前版本用于维护后的视觉抓取链路。启动顺序已经重构为：

```text
hardware / controller / MoveIt bringup
-> visual_ready startup
-> vision nodes
-> candidate_ik_filter
-> visual_grasp_executor
```

`visual_ready` 默认姿态已恢复为 V1.1 初始姿态：

```text
[0.0, 0.0, -0.20, 0.20, 0.0, 0.0]
```

当前默认策略要点：

```text
ordinary_depth_quality_enabled = true
candidate_orientation_yaw_offsets_rad = [0.0]
candidate_grasp_z_offsets_m = [0.0]
candidate_max_variants_per_candidate = 1
candidate_min_grasp_z_m = 0.0
safe_retreat_axis_xyz = [-1.0, 0.0, 0.5]
tcp_offset_xyz = [-0.04, 0.0, 0.0]
```

策略默认值已经从 `visual_grasp_system.launch.py` 拆到 YAML：

```text
src/rebotarm_vision/config/grasp_pose_policy.yaml
src/rebotarm_vision/config/gripper_policy.yaml
src/rebotarm_vision/config/retry_policy.yaml
src/rebotarm_vision/config/retreat_policy.yaml
src/rebotarm_vision/config/visual_servo.yaml
src/rebotarm_vision/config/table_safety.yaml
src/rebotarm_vision/config/graspnet_policy.yaml
src/rebotarm_vision/config/visual_ready.yaml
src/rebotarm_vision/config/flat_graspnet.yaml
```

日常启动只需要保留模式开关和少量覆盖参数。具体策略参数优先改 YAML，不要在终端里堆很长一串。

## 1. 启动前清理

如果之前启动过旧版本，先清理残留节点和 FastDDS 共享内存锁。确认机械臂安全静止后执行：

```bash
pkill -f rebotarm_table_collision || true
pkill -f rebotarm_visual_ready || true
pkill -f rebotarm_visual_grasp_markers || true
pkill -f rebotarm_ordinary_grasp_node || true
pkill -f rebotarm_grasp_tcp_frame || true
pkill -f rebotarm_grasp_candidate_ik_filter || true
pkill -f rebotarm_visual_grasp_executor || true
pkill -f rebotarm_motion_execution_node || true
pkill -f GripperVisualJointStateNode || true
pkill -f reBotArmController || true
pkill -f move_group || true

rm -f /dev/shm/fastrtps_port*
```

## 2. 普通视觉抓取路线

适合当前已经验证较稳定的站立瓶子、规则物体。候选来源是：

```text
YOLO / mask / depth
-> /grasp/candidates
-> candidate_ik_filter
-> /grasp/filtered_plan
-> visual_grasp_executor
```

Ubuntu 终端 A 启动：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  start_vision:=true \
  ordinary_depth_quality_enabled:=true \
  start_candidate_ik_filter:=true \
  executor_input_topic:=/grasp/filtered_plan \
  execution_mode:=execute
```

启动后预期：

```text
机械臂先到 visual_ready
然后启动 rebotarm_vision_node
然后启动 candidate_ik_filter / executor
```

## 3. GraspNet 候选路线

适合测试多候选、平躺物体、复杂姿态物体。候选来源是：

```text
Windows GraspNet baseline
-> http://192.168.145.1:8081/graspnet_candidates.json
-> /grasp/graspnet_candidates
-> candidate_ik_filter
-> /grasp/filtered_plan
```

### 3.1 Windows：一键启动 YOLO + GraspNet

```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main"

.\tools\windows_start_grasp_ai_stack.ps1
```

这个脚本会打开两个 PowerShell 窗口：

```text
windows_start_yolo_server.ps1
windows_start_graspnet_bridge.ps1
```

Windows 端路径、模型、端口、候选 JSON 都已经固定在脚本里。日常测试不需要再手动输入一大串参数。

### 3.2 Windows：分开启动排错

```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main"

.\tools\windows_start_yolo_server.ps1
```

另开一个 Windows 终端：

```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main"

.\tools\windows_start_graspnet_bridge.ps1
```

### 3.3 Ubuntu 终端 A：启动 GraspNet 视觉抓取系统

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  start_vision:=true \
  ordinary_depth_quality_enabled:=true \
  start_graspnet_baseline:=true \
  graspnet_source_mode:=network \
  graspnet_candidates_url:=http://192.168.145.1:8081/graspnet_candidates.json \
  graspnet_network_poll_hz:=0.5 \
  grasp_candidates_topic:=/grasp/graspnet_candidates \
  start_candidate_ik_filter:=true \
  candidate_pose_policy:=preserve_candidate_pose \
  candidate_max_candidates_per_frame:=5 \
  candidate_max_variants_per_candidate:=1 \
  candidate_orientation_yaw_offsets_rad:="[0.0]" \
  candidate_grasp_z_offsets_m:="[0.0]" \
  candidate_workspace_gate_enabled:=true \
  candidate_workspace_min_xyz:="[0.18, -0.35, 0.0]" \
  candidate_workspace_max_xyz:="[0.64, 0.35, 0.45]" \
  candidate_max_grasp_to_object_center_m:=0.15 \
  tcp_offset_xyz:="[0.0, 0.0, 0.0]" \
  executor_input_topic:=/grasp/filtered_plan \
  execution_mode:=execute
```

平躺路线对应的参数参考文件是：

```text
src/rebotarm_vision/config/flat_graspnet.yaml
```

当前为了不影响立着瓶子的稳定路线，`flat_graspnet.yaml` 不作为默认 profile 自动加载；上面的启动命令会显式覆盖平躺需要的关键参数。

GraspNet 路线下，终端 A 可能看到：

```text
candidate IK filter accepted candidate=0 hybrid_geometry_yaw0_z0: score=-0.00
candidate IK filter accepted candidate=1 hybrid_geometry_yaw0_z0: score=-1.00
candidate IK filter accepted candidate=2 hybrid_geometry_yaw0_z0: score=-2.00
candidate IK filter best: best_candidate original_index=0
```

如果使用平躺 GraspNet 参数，日志里的 variant 会变成：

```text
preserve_candidate_pose
```

这里的 `score=-0.00/-1.00/-2.00` 不是成功概率，而是保留输入候选排序后的分数。越靠前的 GraspNet candidate 分数越高。

## 4. 判断当前是否有可抓计划

另开 Ubuntu 终端 B：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /grasp/filtered_plan --once
```

可以抓取的关键字段：

```text
valid: true
source: candidate_ik_filter
```

如果是 GraspNet 路线，额外检查：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 param get /rebotarm_grasp_candidate_ik_filter input_topic
curl http://192.168.145.1:8081/graspnet_candidates.json
```

预期：

```text
input_topic = /grasp/graspnet_candidates
/grasp/graspnet_candidates 里 candidates 不为空
HTTP JSON 里 candidates 不为空
```

## 5. 执行一次抓取

Ubuntu 终端 B：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

成功预期：

```text
success=True
message='visual grasp sequence finished'
```

失败时看终端 A 的阶段日志：

```text
[visual_grasp][run=...][attempt=...][candidate=...][stage=plan]
[visual_grasp][run=...][attempt=...][candidate=...][stage=pregrasp_pose]
[visual_grasp][run=...][attempt=...][candidate=...][stage=grasp_pose]
[visual_grasp][run=...][attempt=...][candidate=...][stage=move_to_pregrasp] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=approach_grasp] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=close_gripper] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=lift] start/ok/fail
```

## 6. 手动控制指令

### 6.1 回到 visual_ready

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service call /rebotarm/visual_ready/move std_srvs/srv/Trigger "{}"
```

如果服务不可用，先检查：

```bash
ros2 service list | grep visual_ready
ros2 node list | grep visual_ready
```

### 6.2 松开夹爪

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.08, max_effort: 0.25}"
```

## 7. 常见现象说明

### 7.1 OpenCV / Qt 字体警告

当前 `vision.launch.py` 已设置：

```bash
QT_QPA_PLATFORM=xcb
QT_QPA_FONTDIR=/usr/share/fonts/truetype/dejavu
```

如果旧进程仍打印字体警告，先清理旧节点并重新启动。旧进程不会自动读取新环境变量。

### 7.2 FastDDS SHM 报错

如果看到：

```text
RTPS_TRANSPORT_SHM Error
Failed init_port fastrtps_port...
```

先执行：

```bash
rm -f /dev/shm/fastrtps_port*
export RMW_FASTRTPS_USE_SHM=0
```

### 7.3 move_group 重名警告

如果看到：

```text
Publisher already registered for node name: 'move_group'
```

检查是否有旧节点残留：

```bash
ros2 node list | grep move_group
ps aux | grep move_group | grep -v grep
```

### 7.4 IK filter dropping frame

如果看到：

```text
candidate IK filter is still processing previous candidates; dropping this frame
```

含义是上一帧候选还在做 IK，新帧先丢弃，避免队列堆积。偶尔出现正常；持续刷屏时可以降低：

```bash
candidate_max_candidates_per_frame:=3
```

### 7.5 轨迹到位误差略超限

如果看到：

```text
trajectory goal not reached within tolerance (max error 0.032 rad > 0.030 rad)
```

含义是机械臂几乎到位，但控制器容差太紧。这个属于轨迹执行层问题，不是视觉候选本身。后续可考虑把到位容差从 `0.030 rad` 放宽到 `0.050 rad`。

## 8. 连续稳定性测试

用于统计连续抓取成功率：

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

每次流程：

```text
1. 程序回 visual_ready
2. 手动松开夹爪
3. 摆放物体
4. 确认安全后按 Enter
5. 执行一次 /rebotarm/visual_grasp/execute
6. 记录 success/fail 和失败阶段
```

建议人工记录：

```text
attempt | result | failed_stage | 偏差方向 | 是否夹住 | 是否抬起 | 备注
1       | success|              |          | yes      | yes      |
2       | fail   | close_gripper| 偏右擦边 | yes      | no       |
```
