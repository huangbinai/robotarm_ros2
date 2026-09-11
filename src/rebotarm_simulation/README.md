# rebotarm_simulation

MuJoCo、RViz fake controller、Real2Sim 和任务环境包。

## 能力

- MuJoCo ROS 节点、Viewer、CLI 和接触检测；
- `FollowJointTrajectory` 与夹爪仿真接口；
- Reach/Pick、Gymnasium 和批量环境；
- Real2Sim 只读桥和 Sim2Real 对比工具；
- URDF 到 MJCF 辅助转换及模型一致性检查。

URDF 与共用电机参数从 `rebotarm_description` 读取，仿真不再依赖 bringup 或 MoveIt 配置资源。选择仿真后端后，由该后端提供夹爪服务和反馈，Dashboard 只作为客户端。

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
ros2 launch rebotarm_simulation real2sim_bridge.launch.py
```

一个命名空间只能选择一个执行后端。仿真通过不代表实机安全。MuJoCo 专题见 [README_mujoco.md](README_mujoco.md)，当前阶段和判据见[验收依据](../../docs/acceptance_criteria_zh.md)。环境冒烟测试通过不代表 Reach/Pick 任务成功。
