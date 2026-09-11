# rebotarmcontroller

真机控制和最后一道执行安全边界。该包独占主 ROS 系统的 MotorBridge 与硬件通信 SDK，负责连接、使能、反馈缓存、控制模式、轨迹执行、夹爪控制和安全关闭。默认需要显式使能；真机 RViz 入口开启 `enable_on_trajectory`，合法轨迹执行时按需使能。

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

## 边界

上层包不得直接访问 SDK。控制器不负责网页、视觉、示教文件或 MoveIt 规划。夹爪命令范围为 `0–0.085 m`；硬件运行目标为 VMware/Ubuntu。

外部 SDK 的定位与临时通道配置由 `sdk_runtime.py` 管理；六轴和夹爪共享总线的方法加锁由 `bus_synchronization.py` 管理。`HardwareManager` 只调用这些适配边界，不再直接修改导入路径或包装 SDK 方法。

本包不再创建录制服务或写示教 JSONL；应用启动由 bringup 组合 `rebotarm_teach/TeachRecorderNode`。电机配置由 launch 从 description 注入，控制器自身仍负责执行与安全边界。

配置归属和关键接口见[项目架构](../../docs/project_architecture_zh.md)，实机退出条件见[验收依据](../../docs/acceptance_criteria_zh.md)。
