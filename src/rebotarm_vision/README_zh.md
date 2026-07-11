# reBotArm 视觉抓取

当前正式抓取链路为 YOLO + GraspNet：

```text
Gemini 2 RGB-D
-> Windows YOLO 检测与目标选择
-> Windows GraspNet baseline 生成 6D 抓取候选
-> /grasp/graspnet_candidates
-> candidate IK / 姿态 / 安全条件筛选
-> /grasp/filtered_plan
-> visual_grasp_executor
-> MoveIt / 夹爪 / 抬升 / 放置
```

旧 ordinary grasp 已移除，不再提供 `/grasp/plan` 或 `/grasp/candidates` 回退路线。GraspNet 不可用或没有有效候选时，系统应停止在感知/筛选阶段，不得自动生成简化抓取动作。

## Windows 感知服务

在仓库的 `tools` 目录启动整套服务：

```powershell
.\tools\windows_start_grasp_ai_stack.ps1
```

该脚本启动 YOLO 图像服务和 GraspNet bridge。默认 HTTP 服务端口为 `8081`，主要接口包括：

```text
http://<windows-ip>:8081/snapshot.jpg
http://<windows-ip>:8081/video.mjpg
http://<windows-ip>:8081/detections.json
http://<windows-ip>:8081/graspnet_candidates.json
```

## ROS 2 感知预览

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py
```

验证候选和筛选结果：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_plan --once
```

## 完整抓取系统

先用只规划模式验证候选、TF、TCP 和 MoveIt 可达性：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false \
  start_vision:=true \
  start_graspnet_baseline:=true \
  graspnet_source_mode:=network \
  execution_mode:=plan_only
```

真实机械臂执行必须在确认相机、TF、TCP、候选姿态和运动范围安全后显式启用。不要把 `plan_only` 验证当作实机安全验证。

## 核心组件

- `vision_node.py`：接收网络 RGB-D、相机内参和 YOLO 检测结果。
- `graspnet_baseline_node.py`：接收网络 GraspNet 候选并发布 `/grasp/graspnet_candidates`。
- `candidate_ik_filter_node.py`：执行 IK、姿态变体、约束和候选评分。
- `visual_grasp_executor_node.py`：执行预抓取、接近、闭合、抬升、放置和安全撤退状态机。
- `visual_grasp_marker_node.py`：显示筛选后的抓取位姿和 TCP 信息。
- `tcp_calibration_node.py`：辅助维护 TCP 偏移。

## 验证

Windows 侧非硬件测试：

```powershell
python -m pytest tests -q
```

实机稳定性验收使用 `rebotarm_visual_grasp_benchmark --attempts 20 --return-ready-before-each --wait-enter`，按失败阶段统计 20 次抓取结果。

ROS 2 构建和 launch 验证应在 Ubuntu 工作区执行。涉及 `use_hardware:=true`、夹爪闭合或轨迹执行前，必须先完成实机安全检查。
