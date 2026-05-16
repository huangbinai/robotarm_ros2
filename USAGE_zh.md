# reBotArm ROS2 交互预览使用说明

这份文档记录当前已经验证可用的 `RViz + MoveIt 预览 + Interactive Marker` 使用方式。

## 1. 日常启动

在 Ubuntu 终端进入工作区后执行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=false \
  use_local_rviz:=true
```

参数说明：

- `use_moveit_preview:=true`：预览走 MoveIt IK
- `use_hardware:=false`：当前不接实机
- `use_local_rviz:=true`：启动外层 RViz

## 2. 启动成功标志

终端里重点看这些日志：

- `preview node ready: namespace=/rebotarm, backend=moveit`
- `execution node ready: namespace=/rebotarm`
- `marker server ready: namespace=/rebotarm`
- `Successfully loaded planner 'OMPL'`
- `You can start planning now!`

## 3. 正常节点状态

可用下面命令检查：

```bash
ros2 node list | sort | uniq -c
```

关键节点应至少包含：

- `/marker_server`
- `/preview_node`
- `/execution_node`
- `/rviz2`
- `/robot_state_publisher`
- `/static_transform_publisher`

说明：

- `move_group` 可能显示多个名字，这是 MoveIt 单进程内部多个 ROS node handle 的表现
- 只要 `marker_server / preview_node / execution_node / rviz2` 都是单实例，交互主链通常就是正常的

## 4. RViz 侧验收

启动后在 RViz 中确认：

- RViz 窗口正常弹出
- 机械臂模型能显示
- 能看到 `reBotArm EE Target` 交互 marker
- 鼠标拖动 marker 时有响应

## 5. Topic 检查

```bash
ros2 topic list | grep interactive
```

正常会看到：

- `/rebotarm/interactive_control/ee_target/feedback`
- `/rebotarm/interactive_control/ee_target/update`
- `/rebotarm/interactive_control/pose_target`
- `/rebotarm/interactive_control/preview`
- `/rebotarm/interactive_control/status`

## 6. 可达目标测试

先开一个终端监听预览：

```bash
ros2 topic echo /rebotarm/interactive_control/preview
```

再开另一个终端发送一个可达点：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 0.25, y: 0.00, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

通过标准：

- `/preview` 有更新
- `reachable: true`
- 消息类似 `moveit ik preview ready`

## 7. 不可达目标测试

发送一个明显超出工作空间的目标：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 1.20, y: 0.00, z: 1.20}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

通过标准：

- `/preview` 有更新
- `reachable: false`
- 消息类似 `target pose unreachable...`
- 不能假成功成 `reachable: true`

## 8. 常用状态查看

```bash
ros2 topic echo /rebotarm/interactive_control/status
ros2 topic echo /rebotarm/interactive_control/preview
```

## 9. 出问题时先清理旧进程

如果怀疑旧节点残留，先清理：

```bash
pkill -f interactive_system.launch.py
pkill -f demo.launch.py
pkill -f move_group
pkill -f rviz2
pkill -f MarkerServerNode
pkill -f PreviewNode
pkill -f ExecutionNode
pkill -f joint_state_publisher
pkill -f robot_state_publisher
pkill -f static_transform_publisher
```

然后重新：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

再执行启动命令。

## 10. 当前版本状态

当前已经验证通过的能力：

- RViz 正常启动
- Interactive Marker 正常显示
- MoveIt 预览正常
- 可达/不可达判断正常
- 交互主链为单实例运行
