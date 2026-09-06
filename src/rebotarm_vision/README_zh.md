# reBotArm 视觉抓取

## 功能定位

本包负责 RGB-D 输入、目标检测、GraspNet 6D 候选、候选筛选、抓取计划显示和抓取执行状态机。正式感知链路为 **YOLO + GraspNet**，硬件运动仍通过 MoveIt、运动层和 `rebotarmcontroller` 完成。

正式数据流：

```text
RGB-D + detections
  -> GraspNet candidates
  -> 新鲜度 / 几何 / 工作空间 / 姿态 / IK / 碰撞筛选
  -> filtered_plan
  -> plan_only 显示或受控执行
```

系统不提供“没有 GraspNet 候选时自动生成简化动作”的回退。输入断流、空候选或旧计划都应停止在感知/筛选阶段。

## 输入模式

视觉节点支持两种部署方式：

- Ubuntu 本地：`camera.type=gemini2`，本地 YOLO，GraspNet 使用 `source_mode=local_backend`；
- 网络兼容：`camera.type=network_mjpeg`，从 HTTP 获取图像、检测或 GraspNet 候选。

仓库当前 `camera.yaml` 和 `graspnet_policy.yaml` 的默认值仍是网络兼容模式。切换本地模式时必须显式覆盖配置，并提供本机模型和 GraspNet checkpoint。

ROS 构建解释器与视觉推理解释器可以分离：

```bash
export REBOTARM_VISION_PYTHON=/path/to/vision-venv/bin/python
ros2 launch rebotarm_vision vision.launch.py \
  camera_config:=/path/to/camera.yaml \
  handeye_config:=/path/to/handeye.yaml \
  yolo_model_path:=/path/to/model.engine \
  yolo_device:=cuda:0
```

## 主要组件

| 组件 | 职责 |
| --- | --- |
| `vision_node.py` | Gemini 2 或网络 RGB-D、CameraInfo、YOLO 检测 |
| `graspnet_baseline_node.py` | 本地后端或网络候选输入 |
| `candidate_ik_filter_node.py` | 候选门限、姿态变体、IK、碰撞和评分 |
| `visual_grasp_marker_node.py` | 候选、TCP、接近轴和夹爪模型显示 |
| `visual_grasp_executor_node.py` | 预抓取、接近、闭合、抬升、放置、撤退和恢复 |
| `visual_ready_node.py` | 视觉准备位姿规划和移动 |

## 推荐验证顺序

先启动感知预览：

```bash
ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_plan --once
```

再以只规划模式运行完整管线：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false \
  execution_mode:=plan_only
```

确认图像、深度、内参、手眼、TCP、TF、桌面、工作空间、IK 和碰撞结果后，才能进行低速实机测试。`plan_only` 通过不等于实机安全通过。

完成分阶段检查后，可用 20 次稳定性测试统计失败阶段：

```bash
ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark -- \
  --attempts 20 \
  --return-ready-before-each \
  --wait-enter
```

结果应按 `failed_stage` 分类，不只统计总成功率。

## 安全约束

- 最大候选和夹爪宽度为 `0.085 m`。
- 候选与计划默认最大年龄为 `1.5 s`。
- 执行器只允许一个抓取请求进入状态机。
- 停止请求必须同时停止机械臂和夹爪步骤。
- 无有效计划时执行服务返回失败。

配置细节见[视觉七层参数](../../docs/visual_grasp_seven_layer_params.md)，总体状态见[功能状态矩阵](../../docs/feature_status_zh.md)。
