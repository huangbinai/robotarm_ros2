# rebotarm_msgs

项目公共 ROS 2 接口定义包。

## 内容

- `msg/`：机械臂状态、电机命令/状态、检测、抓取候选、计划和任务状态。
- `srv/`：模式、校零、夹爪、IK、位姿执行、抓取规划和示教路径。
- `action/`：`MoveToPose`、`MoveRelative` 和 `ExecuteGrasp`。

接口字段修改会影响多个包，应优先保持向后兼容；必须变更时，同步更新调用方、接线测试和[ROS 2 接口参考](../../docs/ros_api_reference_zh.md)。该包只包含接口，不放业务算法。
