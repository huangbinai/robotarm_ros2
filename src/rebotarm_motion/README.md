# rebotarm_motion

与硬件后端无关的运动生成和安全协调层。

## 功能

- MoveIt 规划客户端和 Pose 预览/执行；
- 轨迹时间参数化、起点对齐和消息转换；
- 碰撞预检、运行时跟踪保护和停止协调；
- 实机任务失败恢复和成对轨迹协议；
- 示教样本的通用滤波、重采样工具。

可执行入口为 `PoseExecutionNode`。本包可以调用控制器 ROS 接口和 MoveIt，不得导入 MotorBridge、底层 SDK、Dashboard 或兼容层。未被当前运行入口调用的旧 `pose_preview_solver.py` 及兼容导入已退休；预览/IK 使用 MoveIt，受控位姿执行使用 `PoseExecutionNode`。设计边界见[项目架构](../../docs/project_architecture_zh.md)。
