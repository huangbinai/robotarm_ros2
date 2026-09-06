# rebotarm_moveit_config

reBotArm 的 MoveIt 2 模型与规划配置包。

包含 URDF/SRDF、规划组、末端执行器、IK、OMPL、Pilz、控制器映射、碰撞和 RViz MotionPlanning 配置。该包不放可执行业务逻辑。

仿真演示：

```bash
ros2 launch rebotarm_moveit_config demo.launch.py
```

修改关节范围或 effort 时，同时检查 bringup URDF、控制器安全配置和 MuJoCo 模型。详细中文说明见 [README_zh.md](README_zh.md)。
