# rebotarmcontroller

真机控制和最后一道执行安全边界。该包独占 MotorBridge 与底层机械臂 SDK，负责连接、显式使能、反馈缓存、模式切换、轨迹执行、夹爪控制和安全关闭。

## 主要入口

- 节点：`ros2 run rebotarmcontroller reBotArmController`
- 推荐启动：`ros2 launch rebotarm_bringup driver_only.launch.py`
- Action：`move_to_pose`、`follow_joint_trajectory`、`gripper/command`
- Service：`enable`、`disable`、`safe_home`、`trajectory_stop`、`set_mode`、`set_zero`、重力补偿和夹爪接口

## 关键模块

- `hardware_manager.py`：硬件生命周期、共享总线、反馈和夹爪。
- `ros_actions.py` / `ros_services.py`：ROS API 与命令仲裁。
- `trajectory_safety.py`：轨迹验证与插值。
- `feedback_sequence.py`：逐电机反馈序号和新鲜度。
- `mode_transition.py`：POS_VEL/MIT 平滑切换。

## 边界

上层包不得直接访问 SDK。控制器不负责网页、视觉、示教文件或 MoveIt 规划。夹爪命令范围为 `0–0.085 m`；硬件运行目标为 VMware/Ubuntu。

参数见[配置参考](../../docs/configuration_reference_zh.md)，接口见[ROS 2 接口参考](../../docs/ros_api_reference_zh.md)。
