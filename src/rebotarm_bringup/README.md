# rebotarm_bringup

跨包启动与部署资源包，包含 launch、控制参数、URDF、mesh 和 RViz 配置。

## 推荐入口

| 场景 | launch |
| --- | --- |
| 仅控制器 | `driver_only.launch.py` |
| 控制器和模型 | `bringup.launch.py` |
| 真机 MoveIt | `moveit_hardware.launch.py` |
| RViz 仿真/真机拖动 | `rviz_ee_drag_sim.launch.py` / `rviz_ee_drag_real.launch.py` |
| 示教 | `teach_record.launch.py` / `teach_replay.launch.py` |
| 视觉抓取 | `visual_grasp_system.launch.py` |

## 主要配置

`arm.yaml`、`gripper.yaml`、`driver_params.yaml`、`controller_safety.yaml`、`mode_transition.yaml` 和 `replay_profiles.yaml`。

该包只负责组装，不应实现硬件、轨迹、示教或感知算法。部署步骤见[部署手册](../../docs/deployment_guide_zh.md)。
