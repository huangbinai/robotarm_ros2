# reBotArm 视觉抓取操作流程

当前版本：视觉抓取 V1.1。该版本已完成 Windows GraspNet baseline 候选生成、Ubuntu ROS2 网络拉取、IK 过滤和 `/grasp/filtered_plan` 执行链路，并已完成实机抓取验证。

这份文档记录当前视觉抓取测试流程。V1.2 手写多候选评分暂时暂停；V1.3 开始把 GraspNet baseline 作为更强的候选生成器接入，但仍然保留原 ROS2 执行链路做 IK、MoveIt、TCP 和夹爪安全控制。

## 1. 启动视觉抓取系统

实机测试前确认机械臂周围安全，夹爪区域没有手和线缆。

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  start_vision:=true \
  ordinary_depth_quality_enabled:=true \
  start_candidate_ik_filter:=true \
  candidate_pose_policy:=hybrid_geometry_with_base_axis_fallback \
  candidate_orientation_yaw_offsets_rad:="[0.0, 3.141592653589793]" \
  candidate_grasp_z_offsets_m:="[0.0, 0.03]" \
  candidate_collision_check_enabled:=false \
  executor_input_topic:=/grasp/filtered_plan \
  trajectory_precheck_enabled:=false \
  execution_mode:=execute \
  open_before_approach:=true \
  auto_gripper_width:=true \
  auto_gripper_effort:=true \
  gripper_grasp_enabled:=true \
  gripper_grasp_timeout_sec:=5.0 \
  safe_retreat_enabled:=true \
  safe_home_after_grasp:=false \
  pose_policy:=base_axis
```

## 1.3 GraspNet baseline 候选生成器验证

V1.3 的链路是：

```text
Windows YOLO 选择目标物体
  -> Windows GraspNet baseline 输出 top N 个 6D grasp pose
  -> http://192.168.145.1:8081/graspnet_candidates.json
  -> /grasp/graspnet_candidates
  -> candidate_ik_filter
  -> /grasp/filtered_plan
  -> visual_grasp_executor
```

第一次验证建议只用 `plan_only`，确认 RViz marker、`/grasp/graspnet_candidates`、`/grasp/filtered_plan` 都正常后，再改成 `execute`。

Windows 终端 A：启动 Gemini2 / YOLO / HTTP 服务。

```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main"

D:\anaconda3\envs\orbbec_yolo\python.exe tools\windows_mjpeg_server.py `
  --capture-source orbbec `
  --host 0.0.0.0 `
  --port 8081 `
  --classes "bottle,cup" `
  --allowed-classes "bottle,cup" `
  --graspnet-candidates-path "D:\tmp\graspnet_candidates.json"
```

Windows 终端 B：启动 GraspNet baseline bridge，持续写候选 JSON。

```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main"

D:\anaconda3\envs\graspnet\python.exe tools\windows_graspnet_baseline_bridge.py `
  --server-url http://127.0.0.1:8081 `
  --output-path "D:\tmp\graspnet_candidates.json" `
  --model-root "D:\rebot_ai_models\graspnet-baseline" `
  --checkpoint-path "D:\rebot_ai_models\graspnet-baseline\checkpoints\checkpoint-rs.tar" `
  --backend-module graspnet_baseline_inference `
  --device cuda:0 `
  --max-grasps 5 `
  --poll-hz 0.5
```

注意：`graspnet_baseline_inference` 是你本地 GraspNet baseline 的包装模块，需要提供：

```python
GraspNetBaselineInference(model_root, checkpoint_path, device).infer(
    color_bgr=...,
    depth_mm=...,
    detections=...,
    camera_info=...,
    max_grasps=...
)
```

当前仓库已经提供默认包装模板：

```text
tools\graspnet_baseline_inference.py
```

它会按官方 GraspNet baseline 常见结构导入：

```text
D:\rebot_ai_models\graspnet-baseline\models
D:\rebot_ai_models\graspnet-baseline\dataset
D:\rebot_ai_models\graspnet-baseline\utils
```

所以实际测试前需要确认：

```powershell
Test-Path "D:\rebot_ai_models\graspnet-baseline"
Test-Path "D:\rebot_ai_models\graspnet-baseline\checkpoints\checkpoint-rs.tar"
```

如果你的路径不同，只改 Windows 终端 B 里的 `--model-root` 和 `--checkpoint-path`。

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

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
  candidate_max_candidates_per_frame:=5 \
  candidate_pose_policy:=hybrid_geometry_with_base_axis_fallback \
  candidate_orientation_yaw_offsets_rad:="[0.0]" \
  candidate_grasp_z_offsets_m:="[0.0]" \
  candidate_collision_check_enabled:=false \
  executor_input_topic:=/grasp/filtered_plan \
  trajectory_precheck_enabled:=false \
  execution_mode:=plan_only \
  open_before_approach:=true \
  auto_gripper_width:=true \
  auto_gripper_effort:=true \
  gripper_grasp_enabled:=true \
  gripper_grasp_timeout_sec:=5.0 \
  safe_retreat_enabled:=true \
  safe_home_after_grasp:=false \
  pose_policy:=base_axis
```

另开终端检查：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_plan --once
ros2 param get /rebotarm_grasp_candidate_ik_filter input_topic
```

预期：

```text
input_topic 是 /grasp/graspnet_candidates
/grasp/graspnet_candidates 里 candidates 不为空，source 是 graspnet_baseline
/grasp/filtered_plan valid=true
```

如果 `/grasp/graspnet_candidates` 为空，先在 Ubuntu 检查 Windows HTTP：

```bash
curl http://192.168.145.1:8081/graspnet_candidates.json
```

如果返回 `backend_configured:false`，说明 Windows 端还没有启动/接入 GraspNet baseline 推理脚本。此时不要切 execute。

预期现象：

- 启动后机械臂会先进入 `visual_ready` 准备姿态。
- RViz 中能看到机械臂、物体 marker、抓取相关 marker。
- 终端能看到 `visual_ready startup move complete`。

## 2. 单次抓取

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

失败时看启动终端中的标准日志。每次抓取会打印：

```text
[visual_grasp][run=...][attempt=...][candidate=...][stage=plan]
[visual_grasp][run=...][attempt=...][candidate=...][stage=pregrasp_pose]
[visual_grasp][run=...][attempt=...][candidate=...][stage=grasp_pose]
[visual_grasp][run=...][attempt=...][candidate=...][stage=move_to_pregrasp] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=approach_grasp] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=close_gripper] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=lift] start/ok/fail
```

失败日志会额外打印：

```text
failed_stage
failure_message
pregrasp_pose
grasp_pose
jaw_width
last_gripper_reached_position
contact
closure_distance
```

## 3. 严格 20 次稳定性测试

这个测试用于判断“同一个初始姿态 + 人工重新摆放物体”下，视觉抓取是否稳定。

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

每一次测试的流程是：

```text
1. 程序先调用 /rebotarm/visual_ready/move，让机械臂回到固定准备姿态
2. 你手动松开夹爪，并把瓶子摆到测试位置
3. 确认机械臂周围安全后按 Enter
4. 程序调用 /rebotarm/visual_grasp/execute 执行一次抓取
5. 记录本次 success/fail 和失败阶段
6. 进入下一轮
```

最终输出示例：

```text
total=20 success=17 failed=3 success_rate=85.0%
failed_stage:
  close_gripper: 2
  move_to_pregrasp: 1
```

## 4. 如何判断问题来源

如果失败集中在 `move_to_pregrasp`：

- 优先看 MoveIt 规划、起始姿态、目标是否超出可达范围。
- 检查启动终端是否有 `CheckStartStateCollision` 或 `No motion plan found`。

如果失败集中在 `approach_grasp`：

- 多半是 grasp 点、TCP 偏移或接近方向不稳定。
- 对比 RViz 中 `pregrasp_pose` 和 `grasp_pose` 是否合理。

如果失败集中在 `close_gripper`：

- 重点看 `jaw_width`、`last_gripper_reached_position`、`contact`、`closure_distance`。
- 如果实际夹到了但返回失败，需要继续调接触判断阈值。

如果失败集中在 `lift`：

- 说明闭合后疑似夹住，但 lift 后验证失败。
- 检查夹爪保持力、物体是否滑落、lift 高度是否足够。

## 5. 记录表

建议测试时手动记录：

```text
attempt | result | failed_stage | 偏差方向 | 是否夹住 | 是否抬起 | 备注
1       | success|              |          | yes      | yes      |
2       | fail   | close_gripper| 偏右擦边 | yes      | no       |
```

如果 20 次成功率低于 80%，先不要继续加复杂算法，优先根据失败阶段定位 TCP、手眼 TF、深度稳定性或夹爪接触判断。
