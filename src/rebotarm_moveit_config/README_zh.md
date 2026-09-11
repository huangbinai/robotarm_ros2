# MoveIt 配置与验证

当前配置与包边界以 [README.md](README.md) 和[项目架构](../../docs/project_architecture_zh.md)为准。

本包维护 SRDF、规划器、运动限制和控制器映射，URDF 与 mesh 从 `rebotarm_description` 读取。两个 MoveIt 启动入口使用同一 URDF，不再保存本包内的副本。

当前运行路线是 RViz MotionPlanning → MoveIt → 标准 FollowJointTrajectory 接口。旧 PreviewNode/SDK 双后端规划已退休，不应重新接入。

## 本地运行

已有工作区依赖和构建结果时：

```bash
source tools/source_ubuntu_env.bash
ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

这是单一 fake trajectory controller 的交互仿真。仅启动 `demo.launch.py` 可以检查 MoveIt/模型显示，但不等同于已经具备执行后端。

```bash
ros2 launch rebotarm_moveit_config demo.launch.py use_rviz:=true
```

真机、fake 和 MuJoCo 应分开选择；每个命名空间只允许一个执行后端。launch 参数与用途见[入口说明](../rebotarm_bringup/launch/README.md)。

## 当前验证范围

五项迁移已验证安装后的共享 URDF/mesh 解析、launch 构造和组合依赖。MuJoCo ROS 接口也已有独立证据，但本轮未执行实际 MoveIt 规划/GUI/真机操作。

完整结论见[最新迁移记录](../../docs/acceptance_records/2026-09-11-boundary-migration.md)；后续仍需按[验收依据](../../docs/acceptance_criteria_zh.md)完成 SIM-02、HW 和 OPS 项。
