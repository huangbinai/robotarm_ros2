# reBotArm 配置参数参考

## 1. 参数分层

| 配置 | 作用 | 修改原则 |
| --- | --- | --- |
| `rebotarm_bringup/config/arm.yaml` | 电机 ID、控制频率、MIT/POS_VEL 参数 | 只按实机调试结果修改 |
| `gripper.yaml` | 夹爪电机与控制器参数 | 与六轴共享总线设置保持一致 |
| `driver_params.yaml` | 常用 ROS 驱动参数 | 用于日常覆盖 |
| `controller_safety.yaml` | 最后一层轨迹和反馈安全边界 | 不得比规划层更宽松 |
| `mode_transition.yaml` | MIT/POS_VEL 平滑切换 | 修改后必须重做模式验收 |
| `replay_profiles.yaml` | 示教滤波、重定时和回放限制 | 按轨迹质量选择 profile |
| `rebotarm_moveit_config/config/*` | MoveIt 规划与模型限制 | 与 URDF 和真机边界同步 |
| `rebotarm_vision/config/*` | 相机、候选、夹爪、撤退和重试策略 | 先 `plan_only` 验证 |
| `rebotarm_simulation/config/*` | MuJoCo、Real2Sim 和随机化 | 不覆盖真机安全边界 |
| `rebotarm_voice_control/config/*` | 意图、命名位姿和语音安全限制 | 默认从 `dry_run` 开始 |

## 2. 频率与反馈

| 参数 | 默认值 | 单位 | 说明 |
| --- | ---: | --- | --- |
| `arm.yaml: rate` | `100` | Hz | 底层统一控制循环 |
| `joint_state_rate` | `100` | Hz | ROS JointState 发布频率 |
| `hardware_feedback_rate_hz` | `50` | Hz | 共享总线主动反馈刷新频率 |
| `feedback_stale_timeout_sec` | `0.15` | s | 有效缓存的最大年龄 |

控制循环、反馈刷新和 ROS 发布是三个不同频率。提高发布频率不会产生更多真实硬件反馈；缓存样本的序号和时间戳决定它是否是新状态。

## 3. 关节边界

控制器最终边界来自 `controller_safety.yaml`：

| 关节 | 最小位置 | 最大位置 | 最大速度 | effort |
| --- | ---: | ---: | ---: | ---: |
| J1 | `-2.8 rad` | `2.8 rad` | `3.0 rad/s` | `27` |
| J2 | `-3.14 rad` | `0.0 rad` | `3.0 rad/s` | `27` |
| J3 | `-3.14 rad` | `0.0 rad` | `3.0 rad/s` | `27` |
| J4 | `-1.87 rad` | `1.57 rad` | `1.8 rad/s` | `7` |
| J5 | `-1.57 rad` | `1.57 rad` | `1.8 rad/s` | `7` |
| J6 | `-3.14 rad` | `3.14 rad` | `1.8 rad/s` | `7` |

所有关节最大加速度当前为 `5.0 rad/s²`。轨迹起点容差为 `0.10 rad`，终点容差为 `0.03 rad`，结束收敛等待为 `2.0 s`，轨迹安全采样周期为 `0.01 s`。

J2、J3 的硬件反馈读取可接受到 `+0.02 rad`。这是反馈验证容差，不修改表中的命令和规划上限。

## 4. 电机控制参数

`arm.yaml` 当前实机参数：

| 关节组 | 电机 | MIT `kp/kd` | POS_VEL `vel_kp/vel_ki` | POS_VEL `pos_kp/pos_ki` | `vlim` |
| --- | --- | --- | --- | --- | ---: |
| J1–J3 | `4340P` | `120 / 8` | `0.0125 / 0.004` | `150 / 0.5` | `5.0` |
| J4–J6 | `4310` | `18 / 2` | `0.0008 / 0.002` | `50 / 1.0` | `3.0` |
| 夹爪 | `4310` | `8 / 1` | `0.0008 / 0.002` | `50 / 1.0` | `3.0` |

这里的 `vlim` 是电机配置值，控制器轨迹层仍受更严格的 `trajectory_safety.max_velocity_rad_s` 限制。

## 5. 夹爪参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| 命令位置范围 | `0–0.085 m` | 超界直接拒绝 |
| 内部几何满量程 | `0.09 m` | 仅用于角度/距离换算，不是允许命令上限 |
| `gripper_position_torque_cap_nm` | `1.0 N·m` | 位置动作力矩上限 |
| `gripper_position_max_speed_rad_s` | `1.5 rad/s` | 渐变速度上限 |
| `gripper_position_timeout_margin_sec` | `1.5 s` | 动态超时附加余量 |
| `grasp_hold_timeout_sec` | `30 s` | 抓取保持上限 |
| `gripper_contact_torque_min_nm` | `0.0 N·m` | 默认禁用单独力矩接触门限 |

闭合端约 `1 mm` 的反馈容差只用于接收传感反馈。不要把内部 `0.09 m` 几何常量写入上层夹爪命令。

## 6. 模式切换

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `mode_transition.enabled` | `true` | 启用受控切换 |
| `allow_velocity_mode` | `false` | 禁止直接速度模式 |
| `enter.ramp_duration_sec` | `0.35 s` | 进入 MIT 增益渐变 |
| `enter.max_start_velocity_rad_s` | `0.05 rad/s` | 允许进入的起始速度 |
| `exit.damping_duration_sec` | `0.15 s` | 退出阻尼阶段 |
| `exit.blend_duration_sec` | `0.35 s` | 退出混合阶段 |
| `exit.velocity_wait_timeout_sec` | `1.0 s` | 等待速度下降超时 |
| `safety.max_position_jump_rad` | `0.02 rad` | 切换过程最大目标跳变 |
| `safety.feedback_timeout_sec` | `0.10 s` | 切换反馈超时 |
| `safety.transition_timeout_sec` | `2.0 s` | 整体切换超时 |

## 7. 命令和命名空间

| 参数 | 默认值 | 有效值/说明 |
| --- | --- | --- |
| `arm_namespace` | `rebotarm` | 合法 ROS 名称段，可用于多臂隔离 |
| `channel` | YAML 中设备 | launch 非空值覆盖 YAML |
| `cmd_arbitration` | `reject` | `reject` 或 `preempt` |
| `frame_id` | `base_link` | 基座坐标系 |
| `ee_frame_id` | `end_link` | 末端坐标系 |

## 8. 视觉关键边界

完整视觉启动文件参数较多，按层分组管理：输入与模型、候选几何、工作空间、IK/碰撞、夹爪、撤退、重试和放置。当前关键默认值包括：

| 参数 | 默认值 |
| --- | ---: |
| `candidate_min_jaw_width_m` | `0.006 m` |
| `candidate_max_jaw_width_m` | `0.085 m` |
| `candidate_workspace_min_xyz` | `[0.18, -0.35, 0.0] m` |
| `candidate_workspace_max_xyz` | `[0.64, 0.35, 0.45] m` |
| `plan_max_age_sec` | `1.5 s` |
| `candidates_max_age_sec` | `1.5 s` |
| `safe_retreat_min_lift_z_m` | `0.12 m` |
| `max_allowed_grasp_width_m` | `0.085 m` |

这些工作空间值依赖基座、桌面和相机安装，不能未经现场测量直接视为通用实机参数。完整说明见[视觉抓取七层参数](visual_grasp_seven_layer_params.md)。

## 9. 修改检查清单

修改机械参数时至少同步检查：

1. bringup URDF；
2. MoveIt URDF 和 `joint_limits.yaml`；
3. `controller_safety.yaml`；
4. MuJoCo `robot.xml` 与控制参数；
5. 视觉工作空间和撤退策略；
6. 对应单元测试和文档。
