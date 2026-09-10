# rebotarm_bringup Launch 入口说明

本目录的 launch 文件按“基础能力、用户功能、测试场景”维护。

## 使用前先看这三条

1. 新的基础入口是 `core.launch.py`。
2. `hardware_mode:=real` 才允许连接真实机械臂；仿真和查看模式不会启动真实控制器。
3. 带有 `scenario_` 的入口是快捷场景封装，不应在其中重复维护控制器、MoveIt 或视觉算法逻辑。

## 入口总览

| 文件 | 类别 | 主要作用 | 默认是否连接真机 | 默认是否执行运动 |
| --- | --- | --- | --- | --- |
| `core.launch.py` | 基础 | 控制器、机器人状态、MoveIt、RViz 的统一核心入口 | 否 | 由参数决定 |
| `interactive_system.launch.py` | 兼容 | `core.launch.py` 的旧名称兼容入口 | 否 | 由参数决定 |
| `bringup.launch.py` | 兼容/基础 | 传统整机基础启动：控制器、TF、可选 RViz | 是 | 可能会 |
| `driver_only.launch.py` | 基础 | 只启动 `reBotArmController` | 是 | 控制器可接受命令 |
| `moveit_hardware.launch.py` | 兼容/真机 | 真机 MoveIt 快捷入口，内部调用 `core.launch.py` | 是 | 是 |
| `rviz.launch.py` | 显示 | 只查看 URDF、TF 和关节状态 | 否 | 否 |
| `rviz_ee_drag_sim.launch.py` | 场景/仿真 | MoveIt fake 状态下进行 RViz 末端拖动 | 否 | 仿真执行 |
| `rviz_ee_drag_real.launch.py` | 场景/真机 | 使用真实关节状态进行 RViz 末端拖动 | 是 | 是 |
| `teleop_keyboard.launch.py` | 遥操作 | 键盘遥操作；基础链路由 `core.launch.py` 提供 | 由参数决定 | 由参数决定 |
| `teleop_system.launch.py` | 遥操作组合 | 键盘、示教录制和状态面板组合入口 | 由参数决定 | 由参数决定 |
| `rebotarm_app.launch.py` | 遥操作应用 | 网页工作台，包含 MoveIt、控制器和网页面板 | 是 | 是 |
| `teach_record.launch.py` | 示教 | 只启动示教轨迹录制节点 | 不负责启动 | 否 |
| `teach_replay.launch.py` | 示教 | 回放示教轨迹；默认 `dry_run=true` | 不负责启动 | 默认否 |
| `visual_grasp_perception_preview.launch.py` | 视觉预览 | 视觉感知、GraspNet、候选过滤和 RViz 标记 | 否 | 否 |
| `visual_grasp_system.launch.py` | 视觉执行 | 视觉感知、候选过滤、规划和抓取执行 | 由参数决定 | 由参数决定 |
| `real_perception_sim_execution.launch.py` | 联调场景 | 使用真实视觉候选，在仿真机械臂中执行 | 否 | 仿真执行 |
| `visual_ready_hold.launch.py` | 真机动作 | 移动到视觉准备位并启动准备位服务 | 是 | 是 |

## 推荐命令

### 只查看机器人模型

```bash
ros2 launch rebotarm_bringup rviz.launch.py
```

### MoveIt 仿真

```bash
ros2 launch rebotarm_bringup core.launch.py \
  hardware_mode:=sim \
  use_moveit_preview:=true
```

### 真机 MoveIt

```bash
ros2 launch rebotarm_bringup core.launch.py \
  hardware_mode:=real \
  use_moveit_preview:=true \
  channel:=auto
```

### 键盘遥操作仿真

```bash
ros2 launch rebotarm_bringup teleop_keyboard.launch.py \
  hardware_mode:=sim
```

### 键盘遥操作真机

```bash
ros2 launch rebotarm_bringup teleop_keyboard.launch.py \
  hardware_mode:=real \
  channel:=auto
```

### 视觉预览，不执行抓取

```bash
ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py
```

### 视觉抓取仿真

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  hardware_mode:=sim
```

### 视觉抓取真机

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  hardware_mode:=real \
  channel:=auto
```

### 示教录制和回放

示教入口本身只启动录制器或回放器，控制器、MoveIt 等由外部入口负责。

```bash
ros2 launch rebotarm_bringup teach_record.launch.py
ros2 launch rebotarm_bringup teach_replay.launch.py dry_run:=true
```

真机回放前必须确认机械臂已经处于安全状态，并显式关闭 dry-run：

```bash
ros2 launch rebotarm_bringup teach_replay.launch.py dry_run:=false
```

## 入口之间的关系

```text
core.launch.py
├── teleop_keyboard.launch.py
├── rebotarm_app.launch.py
├── visual_grasp_system.launch.py
├── rviz_ee_drag_sim.launch.py
├── rviz_ee_drag_real.launch.py
└── moveit_hardware.launch.py  # 兼容快捷入口

visual_grasp_perception_preview.launch.py
└── 只负责视觉预览，不包含抓取执行器

real_perception_sim_execution.launch.py
└── visual_grasp_system.launch.py + hardware_mode:=sim
```

## 维护规则

- 控制器、MoveIt、机器人状态发布和基础 RViz 配置只在 `core.launch.py` 中维护。
- 组合入口通过 `IncludeLaunchDescription` 调用核心入口，不复制基础节点定义。
- `hardware_mode:=real` 是连接真机的明确开关。
- 预览入口不得启动运动执行器。
- Teach 回放默认保持 `dry_run=true`。
- 旧名称只用于兼容，不再添加新功能。
