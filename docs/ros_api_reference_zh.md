# reBotArm ROS 2 接口参考

## 1. 命名规则

默认机械臂命名空间为 `/rebotarm`。如果 launch 中设置 `arm_namespace:=robot_a`，表中 `/rebotarm/...` 应替换为 `/robot_a/...`。感知管线的 `/grasp/...` 主题目前使用全局命名。

## 2. 控制器 Topic

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/rebotarm/joint_states` | `sensor_msgs/msg/JointState` | 发布 | 六轴位置、速度和力矩 |
| `/rebotarm/arm_status` | `rebotarm_msgs/msg/ArmStatus` | 发布 | 模式、使能、控制循环、状态机、逐轴状态码和错误码 |
| `/rebotarm/joints/<joint>/state` | `rebotarm_msgs/msg/JointMotorState` | 发布 | 单电机状态，`joint1` 至 `joint6` |
| `/rebotarm/gripper/state` | `rebotarm_msgs/msg/JointMotorState` | 发布 | 夹爪状态 |
| `/rebotarm/joints/<joint>/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 订阅 | 低层单轴稀疏命令，仅用于受控调试 |
| `/rebotarm/gripper/cmd` | `rebotarm_msgs/msg/JointMotorCmd` | 订阅 | 低层夹爪命令 |

`JointMotorCmd` 通过 `use_pos/use_vel/use_kp/use_kd/use_tau/use_vlim` 指示哪些字段有效。低层 Topic 不应作为日常应用接口。

## 3. 控制器 Service

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/rebotarm/enable` | `std_srvs/srv/Trigger` | 按当前反馈位置使能并保持 |
| `/rebotarm/disable` | `std_srvs/srv/Trigger` | 停止控制循环、失能并验证状态 |
| `/rebotarm/safe_home` | `std_srvs/srv/Trigger` | 受控返回安全位置 |
| `/rebotarm/trajectory_stop` | `std_srvs/srv/Trigger` | 停止当前机械臂轨迹 |
| `/rebotarm/gripper/stop` | `std_srvs/srv/Trigger` | 停止夹爪动作 |
| `/rebotarm/gravity_compensation/start` | `std_srvs/srv/Trigger` | 进入重力补偿 |
| `/rebotarm/gravity_compensation/stop` | `std_srvs/srv/Trigger` | 退出重力补偿并恢复保持 |
| `/rebotarm/set_zero` | `rebotarm_msgs/srv/SetZero` | 显式校零并验证新反馈 |
| `/rebotarm/set_mode` | `rebotarm_msgs/srv/SetMode` | 请求控制模式切换 |
| `/rebotarm/move_to_pose_ik` | `rebotarm_msgs/srv/MoveToPoseIK` | IK 求解/小步目标接口 |
| `/rebotarm/gripper/set` | `rebotarm_msgs/srv/SetGripper` | 夹爪位置与最大力矩命令 |
| `/rebotarm/gripper/grasp` | `rebotarm_msgs/srv/GraspGripper` | 带接触/闭合判定的抓取动作 |

示例：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: 0.05, max_effort: 0.4}"
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger
```

## 4. 控制器 Action

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/rebotarm/move_to_pose` | `rebotarm_msgs/action/MoveToPose` | 末端 Pose 轨迹，持续时间必须为 `0.01–300 s` |
| `/rebotarm/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 标准关节轨迹执行 |
| `/rebotarm/gripper/command` | `control_msgs/action/GripperCommand` | 标准夹爪动作 |

所有运动 Action 都要求硬件处于显式使能且可运动状态，并受命令仲裁器保护。取消 Action 会调用相应停止路径。

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {w: 1.0}}, duration: 2.0}" \
  --feedback
```

## 5. 示教接口

示教节点主要使用以下接口：

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/rebotarm/teleop/teach_record/start` | `std_srvs/srv/Trigger` | 开始录制 |
| `/rebotarm/teleop/teach_record/stop` | `std_srvs/srv/Trigger` | 停止并写入记录 |
| `/rebotarm/teleop/teach_record/set_path` | `rebotarm_msgs/srv/SetTeachRecordPath` | 设置记录文件 |
| `/rebotarm/teleop/recording_status` | `std_msgs/msg/String` | 录制状态 |
| `/rebotarm/teleop/replay_status` | `std_msgs/msg/String` | 回放状态 |

实际名称受 `arm_namespace` 和节点配置影响。启动后可用以下命令确认：

```bash
ros2 service list | grep -E 'record|replay'
ros2 topic list | grep teleop
```

## 6. 视觉抓取接口

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/grasp/graspnet_candidates` | `rebotarm_msgs/msg/GraspCandidateArray` | 原始 GraspNet 候选 |
| `/grasp/filtered_candidates` | `rebotarm_msgs/msg/GraspCandidateArray` | 通过前置门限的候选 |
| `/grasp/filtered_plan` | `rebotarm_msgs/msg/GraspPlan` | 当前可执行/预览计划 |
| `/rebotarm/visual_grasp/execute` | `std_srvs/srv/Trigger` | 执行最新且有效的计划 |
| `/rebotarm/visual_grasp/stop` | `std_srvs/srv/Trigger` | 停止视觉任务、运动和夹爪步骤 |
| `/rebotarm/visual_ready/plan` | `std_srvs/srv/Trigger` | 检查视觉准备位姿 |
| `/rebotarm/visual_ready/move` | `std_srvs/srv/Trigger` | 移动到视觉准备位姿 |

无新鲜计划时 `visual_grasp/execute` 应失败，而不是执行旧缓存。

## 7. 仿真接口

MuJoCo 和 RViz fake controller 提供与真机相似的 JointState、`FollowJointTrajectory`、停止和夹爪接口。使用独立命名空间（如 `/rebotarm_sim`）可以避免与真机冲突，但同一命名空间不得同时存在两个后端。

Real2Sim 桥是只读方向：订阅真机关节、夹爪和状态 Topic，发布映射后的仿真状态及桥接状态，不提供真机命令入口。

## 8. 自定义接口文件

`rebotarm_msgs` 当前定义：

- Action：`ExecuteGrasp`、`MoveRelative`、`MoveToPose`；
- Message：`ArmStatus`、`Detection2DArray`、`ExecutionState`、`GraspCandidateArray`、`GraspPlan`、`JointMotorCmd`、`JointMotorState`、`TaskStatus` 等；
- Service：`ExecutePose`、`GraspGripper`、`MoveToPoseIK`、`PlanGraspForLabel`、`SetGripper`、`SetMode`、`SetTeachRecordPath`、`SetZero`。

接口字段的最终定义以 `src/rebotarm_msgs/` 中的 `.msg/.srv/.action` 文件为准。
