# rebotarm_teleop

操作员命令适配层，将键盘、网页和夹爪交互转换为 ROS 目标。

## 入口

- `TeleopKeyboardNode`：键盘遥操作。
- `GripperVisualJointStateNode`：将夹爪状态转换为可视关节状态。
- `web_teleop_client.py` / `web_execute.py`：Dashboard 命令适配。

该包可以调用控制器和运动层接口，但不包含网页渲染、示教算法或硬件 SDK。遥操作首次上机应使用低速度、小步长和 `cmd_arbitration=reject`。
