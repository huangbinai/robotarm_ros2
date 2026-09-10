# rebotarm_bringup

跨包启动与部署资源包，包含 launch、控制参数、URDF、mesh 和 RViz 配置。

## 推荐入口

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

`arm.yaml`、`gripper.yaml`、`controller_runtime.yaml`、`controller_safety.yaml` 和
`rebotarm_interactive_control/config/teleop_control.yaml`。后者统一维护网页/键盘遥操作、示教录制与回放参数。

该包只负责组装，不应实现硬件、轨迹、示教或感知算法。部署步骤见[部署手册](../../docs/deployment_guide_zh.md)。

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
