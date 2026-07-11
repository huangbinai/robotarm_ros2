# YOLO + GraspNet 视觉抓取命令

正式路线：

```text
Gemini 2 RGB-D
-> Windows YOLO 目标选择
-> Windows GraspNet baseline
-> /grasp/graspnet_candidates
-> candidate IK filter
-> /grasp/filtered_plan
-> visual_grasp_executor
```

旧 ordinary grasp 已移除且不会自动回退。

## Windows 感知服务

```powershell
.\tools\windows_start_grasp_ai_stack.ps1
```

检查 `http://127.0.0.1:8081/graspnet_candidates.json` 是否包含有效候选。

## ROS 2 规划验证

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false \
  start_graspnet_baseline:=true \
  graspnet_source_mode:=network \
  candidate_ik_input_topic:=/grasp/graspnet_candidates \
  executor_input_topic:=/grasp/filtered_plan \
  execution_mode:=plan_only
```

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_plan --once
```

## 严格稳定性测试

系统完成非硬件验证和实机安全检查后，在第二个终端运行：

```bash
ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark \
  --attempts 20 \
  --service-timeout-sec 180 \
  --return-ready-before-each \
  --wait-enter
```

benchmark 每轮先调用 `/rebotarm/visual_ready/move`，确认后再调用 `/rebotarm/visual_grasp/execute`。记录 `failed_stage`，按 `move_to_pregrasp`、`approach_grasp`、`close_gripper`、`lift` 分类排查。

仿真链路可使用 `rebotarm_hybrid_grasp_sim_benchmark`，通过 `--plan-timeout-sec` 等待新鲜的 `/grasp/filtered_plan`，并用 `--min-success-rate` 设置验收阈值；需要每轮回到准备位时添加 `--return-ready-after-each`。保持当前安全限制 `candidate_max_joint6_delta_rad:=1.5708` 和 `candidate_joint6_symmetry_enabled:=true`，仿真验收使用 `gripper_grasp_enabled:=false`。

真实执行必须显式启用硬件和 execute 模式。GraspNet 后端不可用、候选为空、TF/TCP 不可信或 RViz 位姿异常时不得执行。
