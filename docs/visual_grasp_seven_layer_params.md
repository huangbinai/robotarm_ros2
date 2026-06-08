# 视觉抓取七层参数设计

本文记录当前通用视觉抓取路线的七层参数职责。目标是让系统支持竖放、平放、倾斜等不同摆放姿态，而不是只针对固定瓶子姿态调参。

核心路线：

```text
YOLO mask 锁定目标物体
GraspNet Baseline 输出目标物体附近的 6D grasp candidates
TF 转换到 base_link
候选 IK / workspace / jaw_width / joint6 / 轨迹门控
visual_grasp_executor 执行 pregrasp -> grasp -> close -> lift -> retreat
```

## 第 1 层：Approach / Pregrasp

作用：决定真正抓取前，机械臂先退到哪个 pregrasp 点。

当前参数：

```yaml
base_pregrasp_distance_m: 0.08
candidate_pregrasp_min_z_m: 0.120
pregrasp_base_z_offset_m: 0.05
```

说明：

- `base_pregrasp_distance_m`：pregrasp 距离 grasp 点的退后距离。值越大越安全，但越容易 IK 或规划失败。
- `candidate_pregrasp_min_z_m`：候选 IK 过滤阶段的 pregrasp 最低高度。
- `pregrasp_base_z_offset_m`：旧 base_axis/hybrid 路线下的 pregrasp 额外抬高量。GraspNet 主路线建议保持谨慎使用。

测试“直接到 grasp”时，可以临时设置：

```yaml
base_pregrasp_distance_m: 0.0
candidate_pregrasp_min_z_m: 0.0
pregrasp_base_z_offset_m: 0.0
```

## 第 2 层：Pose Variant / 姿态变体

作用：GraspNet 给出一个姿态后，只生成必要的等价姿态，避免手工乱扩展 yaw 或 z。

当前参数：

```yaml
candidate_orientation_yaw_offsets_rad: [0.0]
candidate_grasp_z_offsets_m: [0.0]
candidate_joint6_symmetry_enabled: true
candidate_joint6_symmetry_angle_rad: 3.141592653589793
candidate_max_joint6_delta_rad: 1.5708
```

说明：

- `candidate_orientation_yaw_offsets_rad: [0.0]`：不再默认扩展固定 yaw。
- `candidate_grasp_z_offsets_m: [0.0]`：不再默认额外加 z 高度候选。
- `candidate_joint6_symmetry_enabled`：允许生成夹爪 180 度等价姿态。
- `candidate_max_joint6_delta_rad`：joint6 相对当前姿态最大允许变化量，`1.5708 rad = 90°`。

## 第 3 层：Candidate Target / 抓取目标生成

作用：把 GraspNet candidate 转成执行链路使用的 `pregrasp_pose` 和 `grasp_pose`。

当前参数：

```yaml
candidate_pose_policy: preserve_candidate_pose
tcp_offset_xyz: [-0.04, 0.0, 0.0]
target_base_offset_xyz: [0.0, 0.0, 0.0]
grasp_base_z_offset_m: 0.0
```

说明：

- `candidate_pose_policy: preserve_candidate_pose`：保留 GraspNet 原始 6D 姿态。
- `tcp_offset_xyz`：夹爪抓取中心相对 `end_link` 的局部坐标偏移。当前表示抓取中心在 `end_link` 的 X 负方向 4cm。
- `target_base_offset_xyz`：最终目标点在 `base_link` 下的整体补偿。当前为零，表示不再额外平移视觉目标点。
- `grasp_base_z_offset_m`：人为改变 grasp z 的旧路线参数，当前保持 `0.0`。

## 第 4 层：Candidate Motion / 运动代价

作用：约束机械臂运动不要出现明显不友好的腕部姿态。

当前参数：

```yaml
candidate_score_joint_distance_weight: 0.15
candidate_score_joint6_weight: 0.35
candidate_max_joint6_delta_rad: 1.5708
```

说明：

- `candidate_score_joint_distance_weight`：关节整体移动越大，评分惩罚越大。
- `candidate_score_joint6_weight`：joint6 转动越大，评分惩罚越大。
- `candidate_max_joint6_delta_rad`：joint6 硬限制，超过限制的候选会被拒绝。

## 第 5 层：Candidate Gate / 候选门控

作用：过滤明显不属于目标物体、机械臂范围外、夹爪宽度不合理的候选。

当前参数：

```yaml
candidate_max_candidates_per_frame: 20
candidate_min_jaw_width_m: 0.006
candidate_max_jaw_width_m: 0.082
candidate_min_grasp_z_m: 0.0
candidate_workspace_gate_enabled: true
candidate_workspace_min_xyz: [0.18, -0.35, 0.0]
candidate_workspace_max_xyz: [0.64, 0.35, 0.45]
candidate_max_grasp_to_object_center_m: 0.15
```

说明：

- `candidate_max_candidates_per_frame`：Ubuntu 候选 IK 过滤器每帧最多处理多少个 GraspNet 候选。当前为 20。
- `candidate_min_jaw_width_m`：夹爪最小有效开口。
- `candidate_max_jaw_width_m`：候选最大允许抓取宽度。当前为 `0.082m`。
- `candidate_min_grasp_z_m`：grasp 点最低 z。当前为 `0.0`，不再用 `0.12m` 卡低高度抓取。
- `candidate_workspace_*`：工作空间门控，避免 GraspNet 飞点进入 IK。
- `candidate_max_grasp_to_object_center_m`：候选点距离目标中心过远时拒绝。

## 第 6 层：TF / 坐标转换

作用：把 GraspNet 输出的相机坐标系 pose 转成机械臂规划需要的 `base_link` pose。

当前参数：

```yaml
target_frame: base_link
candidate_source_frame: camera_depth_frame
tf_lookup_timeout_sec: 0.2
```

说明：

- GraspNet 基于深度/点云，候选姿态应来自 `camera_depth_frame`。
- 手眼 TF 当前使用 `end_link -> camera_depth_frame`。
- `tf_lookup_timeout_sec` 不宜太长，否则候选过滤器会卡顿。

## 第 7 层：Motion Feasibility / 运动可行性

作用：判断候选是否真正能被机械臂走完整套流程。

当前参数：

```yaml
candidate_collision_check_enabled: false
trajectory_precheck_enabled: true
moveit_planning_time: 2.0
moveit_num_planning_attempts: 1
candidate_filter_service_timeout_sec: 5.0
```

说明：

- IK 可达不等于轨迹可执行，所以实机抓取建议保持 `trajectory_precheck_enabled: true`。
- 候选阶段负责快速筛掉明显不可用目标，执行前规划负责确认整条路径能走通。
- `candidate_filter_service_timeout_sec` 是候选 IK/service 等待时间，不是夹爪闭合超时。

## 夹爪策略层

夹爪不属于候选七层中的某一层，但它决定 close 阶段是否成功。

当前参数：

```yaml
open_before_approach: true
auto_gripper_width: true
auto_gripper_effort: true
open_clearance_m: 0.0
close_margin_m: 0.012
close_max_effort: 0.4
gripper_grasp_enabled: true
gripper_grasp_close_force: 0.4
gripper_grasp_timeout_sec: 5.0
gripper_grasp_min_close_time_sec: 0.08
gripper_grasp_velocity_threshold: 0.04
gripper_grasp_min_closure_distance_m: 0.006
```

说明：

- `gripper_grasp_timeout_sec`：接触式闭合最多等待多久。如果这段时间内没有检测到接触，就返回 `grasp close timeout before contact`，close 阶段失败。当前默认是 `5.0s`。
- `gripper_grasp_close_force`：闭合阶段力控，当前为 `0.4`。
- `close_max_effort`：普通夹爪 set position 阶段最大力，当前为 `0.4`。
- `open_clearance_m: 0.0`：GraspNet 算出的 jaw_width 不再额外加开口余量。
- `close_margin_m: 0.012`：闭合目标约为 `jaw_width - 0.012m`。

## 当前 GraspNet 主路线汇总

```yaml
candidate_pose_policy: preserve_candidate_pose
candidate_max_candidates_per_frame: 20

tcp_offset_xyz: [-0.04, 0.0, 0.0]
target_base_offset_xyz: [0.0, 0.0, 0.0]

candidate_orientation_yaw_offsets_rad: [0.0]
candidate_grasp_z_offsets_m: [0.0]
candidate_joint6_symmetry_enabled: true
candidate_joint6_symmetry_angle_rad: 3.141592653589793
candidate_max_joint6_delta_rad: 1.5708

candidate_min_jaw_width_m: 0.006
candidate_max_jaw_width_m: 0.082
candidate_min_grasp_z_m: 0.0

candidate_workspace_gate_enabled: true
candidate_workspace_min_xyz: [0.18, -0.35, 0.0]
candidate_workspace_max_xyz: [0.64, 0.35, 0.45]

trajectory_precheck_enabled: true
gripper_grasp_timeout_sec: 5.0
```
