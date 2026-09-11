# rebotarm_bringup

跨包启动与部署资源包，包含 launch、运行/安全/交互参数和 RViz 配置。URDF、mesh 及共用电机参数归属 `rebotarm_description`。

## 推荐入口

构建使用 `bash tools/build_workspace.bash`，以项目 Python 生成所有 ROS 启动脚本；脚本自动检查入口解释器。日常运行仍使用 `source tools/source_ubuntu_env.bash`。

日常真机末端操作只使用下面的入口（不要同时启动另一个控制器或 MuJoCo）：

```bash
cd ~/robotarm_ros2
source tools/source_ubuntu_env.bash
ros2 launch rebotarm_bringup rviz_ee_drag_real.launch.py
```

只使用 RViz 原生 MotionPlanning，不启动网页或自定义 ee_target marker。
启动和 Plan 不使能、不运动；点击 Execute 发送 FollowJointTrajectory，控制器在
校验轨迹、反馈及命令所有权之后按需使能并执行。失败会返回 Action 错误。
该行为仅由真机 RViz 入口开启；其他入口默认仍需显式使能。
到位后保持，停止运动使用 MotionPlanning 的 Stop（不等于失能或急停）。
该 Action 接口也可被其他 ROS 客户端调用；控制器无法区分目标是否来自 RViz。
安全停放或支撑机械臂后再退出终端。
Ctrl+C 不自动回安全位，但会关闭控制器并尝试失能，不会继续保持力矩。

不连接硬件的 RViz 仿真使用同样的 Plan / Execute 操作：

```bash
ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

此入口使用单一 RViz fake trajectory controller 执行轨迹并发布反馈，不是 MuJoCo 物理仿真。

日常启动默认值统一在 `config/rviz_real.yaml`，电机参数在 `rebotarm_description/config/arm.yaml` / `gripper.yaml`，
安全边界仍在 `controller_safety.yaml`。串口自动识别；多设备时在配置中指定固定设备标识。
临时覆盖仍可使用 `channel:=/dev/ttyACM1`。
`core.launch.py` 是组合入口，普通 RViz 操作无需填写它的内部开关。

| 场景 | launch |
| --- | --- |
| 仅控制器 | `driver_only.launch.py` |
| 控制器和模型 | `bringup.launch.py` |
| 基础系统/MoveIt | `core.launch.py` |
| 真机 MoveIt（兼容入口） | `moveit_hardware.launch.py` |
| RViz 仿真/真机拖动 | `rviz_ee_drag_sim.launch.py` / `rviz_ee_drag_real.launch.py` |
| 示教 | `teach_record.launch.py` / `teach_replay.launch.py` |
| 视觉抓取 | `visual_grasp_system.launch.py` |

## 主要配置

本包的 `controller_runtime.yaml`、`controller_safety.yaml`、`interactive_control.yaml` 和
`teleop_control.yaml`；后者统一维护网页/键盘遥操作、示教录制与回放参数。电机参数由 description 包提供，可通过 `arm_config` / `gripper_config` 显式覆盖。

该包只负责组装，不应实现硬件、轨迹、示教或感知算法。当前资源依赖图见[项目架构](../../docs/project_architecture_zh.md)，部署验收见[验收依据](../../docs/acceptance_criteria_zh.md)。

录制只由 `rebotarm_teach` 提供。完整网页入口自动开启一个空闲录制节点，`teleop_system` 同样只开启一个；普通 core、RViz 和 driver_only 默认不启动录制。需要基础入口携带录制服务时显式设置 `start_teach_recorder:=true`，或在只运行 driver/core 时使用独立 `teach_record.launch.py`。不要在已开启录制服务的 app 上再次叠加独立录制节点。

完整的 launch 功能、风险说明、依赖关系和常用命令见
[launch/README.md](launch/README.md)。

## 运行模式

基础入口 `core.launch.py` 使用 `hardware_mode` 区分运行环境：

| 模式 | 含义 |
| --- | --- |
| `none` | 仅模型/状态显示，不连接硬件 |
| `sim` | 仿真或 MoveIt fake joint states |
| `real` | 连接真实机械臂 |
| `auto` | 兼容旧参数，根据 `use_hardware` 判断 |

新命令优先显式指定模式，例如：

```bash
ros2 launch rebotarm_bringup core.launch.py hardware_mode:=sim use_moveit_preview:=true
ros2 launch rebotarm_bringup core.launch.py hardware_mode:=real use_moveit_preview:=true
```

`use_hardware` 暂时保留用于兼容旧脚本；新 launch 包装器应传递 `hardware_mode`。
