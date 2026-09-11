# 2026-09-11 五项包边界迁移与验证

结论：本次要求解决的五项架构问题已完成代码迁移和自动验证。当前为 13 个 ROS 包，严格架构检查通过，依赖/SDK 例外为零。新 build/install 构建通过，最终回归 48 项通过，MuJoCo 无头、ROS 接口及合成 Real2Sim 检查通过。本记录不构成实机或整个 S1/S2 阶段验收。

## 1. 变更及验证范围

| 问题 | 最终实现 | 验证 |
| --- | --- | --- |
| 控制器与 teach 重复录制 | 删除控制器内置录制器；所有录制服务/JSONL 写入归属 TeachRecorderNode；消息按 header.stamp 去重 | 应用组合仅一个录制节点、普通 RViz 无录制器、兼容开关、重复/倒退时间戳及录制状态门槛 |
| Dashboard 生成仿真反馈 | 删除 Dashboard 的夹爪反馈发布器；仿真夹爪通过后端 SetGripper 服务，真机保留 GripperCommand Action | 无后端拒绝、等待响应、后端成功/失败、Future 异常及原真机 Action 路径 |
| 共享资源形成依赖环 | 新增 description 资源包；迁移唯一 URDF/mesh 和共用电机参数；交互配置移到 bringup；补齐资源依赖 | 新目录构建、实际安装资源解析、mesh URI、模型关节限位、所有包装入口构造及严格图检查 |
| motion 绑定旧 SDK | 删除当前无运行调用者的 PosePreviewSolver 与兼容 shim；规划继续由 MoveIt 提供 | SDK 例外清零；MoveIt 使用共享 URDF 的 launch 构造成功 |
| 直接依赖声明不全 | 补齐直接 ROS、launch 和已映射 Python 依赖；检查器增加外部依赖和服务/反馈归属规则 | 16 项检查器回归、严格门禁、新目录构建 |

## 2. 路径和接口迁移

| 旧位置/行为 | 新位置/行为 |
| --- | --- |
| `rebotarm_bringup/description/` | `rebotarm_description/description/` |
| bringup `config/arm.yaml`、`gripper.yaml` | description `config/arm.yaml`、`gripper.yaml` |
| moveit_config `config/rebotarm.urdf` 副本 | 删除；MoveIt 直接读取 description 的 `description/urdf/reBot-DevArm_fixend.urdf` |
| interactive_control `config/*.yaml` | bringup `config/*.yaml` |
| 控制器 `teach_record_*` ROS 参数 | 由 core 的同名 launch 参数传给独立 TeachRecorderNode；控制器本身不再消费 |
| controller 内置录制服务 | 服务名称不变，由 teach 提供；driver_only/RViz 默认不携带录制 |
| `rebotarm_motion.pose_preview_solver`、旧兼容路径 | 退休；外部脚本需迁移到 MoveIt 规划/IK 接口 |
| 无仿真后端时网页生成成功结果/反馈 | 返回 unavailable，正式反馈只订阅执行后端 |

普通 core 默认 `start_teach_recorder=false`。完整 app 设为 true；teleop_system 自己启动录制节点并向子入口显式传 false，避免继承开关造成重复。独立 teach_record 只用于尚未开启录制服务的 driver/core 场景。多个手工启动或跨机器部署仍须检查 ROS 图，不应叠加相同命名空间的组合入口。

URDF 对比确认旧 bringup 与 MoveIt 副本只在头部注释不同；迁移保留物理数据，仅更新资源归属与 URI。MuJoCo 的生成 MJCF/场景/资产仍保留用于独立运行。

退休代码与旧路径可通过 Git 历史恢复；恢复前需评估是否重新引入本次已关闭的边界问题。语音包维持删除状态。

## 3. 环境与构建

- 日期：2026-09-11，Asia/Shanghai。
- Git 基线：`0dd373ad05a13b944cfb2ba368206c1badf5950a`，dirty 工作区；包含此前已有修改和本次迁移。最终发布需要提交并重新固定版本。
- Python 3.12.3，ROS 2 Jazzy，本机 `.venv-ros`，MuJoCo 3.13.0。
- 新构建根：`.codex_tmp/architecture-build.PdKGGh/`；没有复用旧 build/install 的项目输出，但使用现有 ROS/venv 依赖环境。

实际构建命令：

```bash
source tools/source_ubuntu_env.bash
colcon --log-base .codex_tmp/architecture-build.PdKGGh/log build \
  --base-paths src \
  --build-base .codex_tmp/architecture-build.PdKGGh/build \
  --install-base .codex_tmp/architecture-build.PdKGGh/install \
  --cmake-args -DPython3_EXECUTABLE=/home/huangbin/robotarm_ros2/.venv-ros/bin/python
```

结果：`13 packages finished [20.8s]`，退出码 0。没有使用 `--symlink-install`，验证对象为新目录实际安装的资源和 Python 包。兼容录制参数补齐后单独重建 bringup 成功，并再次通过最终回归。

构建时已有本地 ROS overlay 缺少 local_setup 的警告；该环境由现有环境脚本手工加入路径。此结果不能替代全新 Ubuntu 依赖环境的复现。

日常 `build-venv` 增量构建发现旧 MoveIt URDF 链接悬空。MoveIt、bringup、interactive_control 三个包的旧 build/install 子目录已可恢复地归档到 `.codex_tmp/migration-cache.AjwvF3/` 后重建，其他包缓存未清理。资源移动后若旧构建目录保留已删除文件，不能仅靠重新运行增量构建排除旧路径。

归档后日常目录重建结果：`13 packages finished [7.90s]`，退出码 0。只 source `tools/source_ubuntu_env.bash` 的新进程中已确认全部 13 个包来自 `install-venv`，共享 URDF/电机参数可读取，旧 MoveIt URDF、SDK 预览模块、控制器录制模块和语音包均不可用。日常启动无需 source 临时验收 overlay。

## 4. 自动回归与架构门禁

```bash
python3 tools/check_architecture.py --strict-deployment
source tools/source_ubuntu_env.bash
source .codex_tmp/architecture-build.PdKGGh/install/local_setup.bash
python -m pytest -q tests \
  --junitxml=docs/acceptance_records/2026-09-11-boundary-migration-tests.xml
```

严格门禁：`13 packages, 291 Python files`，`boundaries=PASS; deployment=PASS`，退出码 0。

最终回归：**48 passed in 1.41s**，退出码 0。原始 JUnit 结果见 [boundary-migration-tests.xml](2026-09-11-boundary-migration-tests.xml)。其中 16 项为检查器回归、17 项为本次迁移行为/安装/组合检查、15 项为现有 RViz 假硬件执行回归。

启动组合测试仅构造/解析 launch，不启动控制器或 GUI。夹爪客户端使用内存 Future 与真实 ROS 消息类型；断连案例是 Future 异常，不等同于完成所有真实网络超时测试。录制验证使用内存写入探针。

CI 已切换为严格门禁，本次未提交或触发远端 CI。

最终文档链接检查覆盖 27 篇当前 Markdown，缺失本地目标为 0；迁移过程中已被移除的旧命令/历史文档没有恢复。源码、测试和当前维护文档的差异格式检查通过。

## 5. 无头与 ROS 仿真验证

以下均在新安装目录 overlay 中执行：

```bash
python -m rebotarm_simulation.mujoco_acceptance --skip-renderer
python -m rebotarm_simulation.real2sim_acceptance --mode mirror
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 \
  python -m rebotarm_simulation.mujoco_ros_acceptance --timeout 15
```

| 检查 | 实际结果 | 限制 |
| --- | --- | --- |
| MuJoCo 无头套件 | 4 子项均 ok；8 关节/8 执行器；接触 263 步、最大方块接触力约 2.255 N、最大单步跳变约 0.000503 m | Renderer 跳过；Reach/Pick 冒烟成功数均为 0，不代表任务成功 |
| Real2Sim mirror | 200 个合成样本，最大关节误差 0 rad，状态有限，ok=true | 未连接真实设备，不含 physics 或真实丢帧验收 |
| MuJoCo ROS 接口 | 夹爪服务成功，轨迹 Action 接受且成功，时钟推进、关节消息结构通过，ok=true | 进程内 MuJoCo 后端；没有实际 MoveIt 规划或真机 |

ROS 接口输出摘要：

```json
{
  "ok": true,
  "joint_state_count": 52,
  "clock_count": 52,
  "clock_progress": true,
  "joint_schema_ok": true,
  "gripper_service_success": true,
  "gripper_reached_position_m": 0.05,
  "trajectory_action_accepted": true,
  "trajectory_action_success": true,
  "trajectory_error_code": 0,
  "final_max_joint_error_rad": 0.015228914009311256,
  "final_clock_sec": 1.768
}
```

该测试在独立 ROS domain 173 内创建 MuJoCo 与探针，未启动硬件控制器。ROS 对 LOCALHOST_ONLY 给出了弃用警告，但仍按本机发现范围执行。

## 6. 剩余阶段工作

五项要求对应的架构迁移已关闭。剩余 ARCH-04 后端接口/夹爪默认范围差异、ARCH-05 完整安全回归、实际 MoveIt/GUI、全新系统依赖复现、实机和视觉任务验收仍在阶段表中维护。静态依赖通过和本次仿真通过均不自动升级整个阶段。
