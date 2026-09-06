# 下游真机与安全修复迁移记录

## 基线与策略

- 下游来源：`QingYuan-Chen/robot_Arm`，审查基线 `aa1dcc52c7e7046f8040c2b9becd767df4d07265`。
- 当前迁移分支：`migrate/downstream-hardware-safety`。
- 两条代码线没有可安全直接合并的共同演进历史，因此按功能切片移植，不整文件覆盖。
- 保留本仓库已有的命令仲裁、轨迹严格校验与连续插值、夹爪空闭排除、视觉计划 TTL。
- 不执行任何自动真机测试。完成离线测试后，仍须按本文顺序分阶段上机。

## 已迁移

1. MotorBridge 新反馈身份
   - 审核补丁：`patches/motorbridge/0001-add-feedback-sequence-api.patch`。
   - 安装工具：`tools/setup_motorbridge_fresh_feedback.py`。
   - 每个电机反馈必须在请求之后出现 uint64 sequence 前进；数值变化不能替代 sequence。
   - sequence 为零、重复、反向或超过半范围的跳变均不能证明新帧。
   - `reBotArm_control_py` 的反馈缺失与回零超时修复另存为审核补丁；可用
     `python tools/apply_rebotarm_control_safety_patch.py` 只读确认已导入的固定版本 SDK。

2. 连接与使能生命周期
   - `connect()` 只连接、发现夹爪、检查新鲜失能反馈，最终进入 `CONNECTED_DISABLED`。
   - `enable()` 先取得新鲜当前位置并预载保持目标，再切 POS_VEL、使能、验证状态，最终进入 `ENABLED_HOLD`。
   - 使能失败执行失能回滚；回滚无法验证时报告 `ENABLE_ROLLBACK_FAILED`。

3. 共享总线
   - 六轴和夹爪按 Controller 分组，在同一事务内请求与轮询。
   - 机械臂、反馈和夹爪命令由一个硬件循环串行执行；ROS 发布定时器只读已验证缓存。
   - 反馈超过阈值或通信失败时，硬件循环请求保护失能；只有随后新 sequence
     明确报告全部 `status_code=0`，状态才从 `DISABLING` 转成 `CONNECTED_DISABLED`。
   - 反馈请求按电机维护 baseline、响应窗口和 deadline；允许总线响应延迟到下一调度批次，
     但超过窗口或 sequence 未前进仍按该电机独立报错，不用其他电机的新帧代替。
   - 紧急失能返回成功前同样要求失能命令之后的新 sequence 确认；无法确认时保留
     `DISABLING` 和原 `_enabled` 状态，并报告 `EMERGENCY_DISABLE_UNVERIFIED`，不虚报已失能。
   - 硬件循环只把已确认的反馈/通信错误升级为保护失能；普通回调编程异常会继续抛出，
     避免被错误归类为硬件故障。

4. 夹爪
   - 位置目标采用速度受限渐变，超时由行程/速度加余量动态计算。
   - 位置模式默认总力矩上限 1.0 N·m，到位或取消后发送中性命令并停止持续加载。
   - 抓取保持默认 30 秒，最长允许配置为 120 秒，到期中性释放。
   - 接触判断只累计不同 sequence 的连续样本，并保留空夹爪闭合到底排除。
   - 默认不强制固定扭矩门限，因为当前夹爪没有独立力传感器；可在标定后配置。
   - 允许约 1 mm 的闭合端正角度反馈容差，但只用于反馈接受；更大的正角度返回无效值，不伪装成正常闭合。
   - `set_zero(gripper)` 只允许在 `CONNECTED_DISABLED` 状态调用；校零后必须在 0.5 秒内取得连续三帧不同 sequence 的失能、近零反馈才成功。
   - 校零写入或验证失败会持续作为 `GRIPPER_FEEDBACK` 错误发布，并阻止启用和夹爪位置解释；只有下一次显式校零验证成功才清除。
   - Gripper Action 在命令错误或反馈状态不是 `status_code=1` 时立即中止；动态超时后
     显式停止夹爪，防止 Action 已失败而底层仍继续运动。

5. 失败恢复与独立工具
   - `rebotarm_motion.real_failure_recovery` 提供“健康保持 / 严重故障保护失能 / 可选返回起点后失能”的复用策略。
   - 视觉执行器订阅 `/arm_status` 并实际接入该策略。默认 `failure_recovery_mode=hold`；可选 `return_then_disable` 会先通过 MoveIt 碰撞预检，再以 0.04 速度缩放返回本轮起始末端位姿。
   - 返回失败但状态仍健康时保留使能保持，等待人工处理。
   - 自动返回起点前必须同时确认运动执行层和底层轨迹均已停止；任一停止请求未确认时
     禁止返回。若状态已是严重故障，即使停止确认失败仍继续请求保护失能。
   - 若已经确认夹爪接触，自动任务失败只停止机械臂运动，不提前解除有限时夹爪保持；人工停止仍会停止夹爪。
   - `star_arm_102_rebot_b601_follow/` 独立导入，不加入 ROS launch，也不会自动打开串口或使能电机。

## 实机参数

| 参数 | 类默认 | 实机 launch | 约束 |
|---|---:|---:|---|
| SDK `rate` | 100 Hz | 100 Hz | 机械臂主控制循环，采用下游实机配置；标准 launch 与 SDK 备用配置保持一致 |
| `joint_state_rate` | 100 Hz | 100 Hz | 状态发布频率；启动时限制为有限值 1–500 Hz，不等于总线轮询频率 |
| `hardware_feedback_rate_hz` | 50 Hz | 50 Hz | 允许 20–100 Hz |
| `feedback_stale_timeout_sec` | 0.15 s | 0.15 s | 允许 0.05–2.0 s |
| J2/J3 反馈接受上限 | 0.02 rad | 0.02 rad | 仅允许编码器量化/回差端点反馈，不扩大当前命令上限 |
| J4–J6 模型 effort | 7 N·m | 7 N·m | MoveIt、bringup URDF 和 MuJoCo 执行限制保持一致 |
| `gripper_position_torque_cap_nm` | 1.0 | 1.0 | 允许 0.05–1.5 N·m |
| `gripper_position_max_speed_rad_s` | 0.5 | 1.5 | 类默认保守，实机值来自下游调试 |
| `gripper_position_timeout_margin_sec` | 1.5 s | 1.5 s | 动态超时附加量 |
| `grasp_hold_timeout_sec` | 30 s | 30 s | 最大 120 s |
| `gripper_contact_torque_min_nm` | 0 | 0 | 默认禁用固定扭矩门限 |
| `failure_recovery_mode` | `hold` | `hold` | `hold` 或 `return_then_disable` |
| `failure_recovery_status_timeout_sec` | 1.0 s | 1.0 s | 恢复决策只接受请求后的新状态 |
| `failure_recovery_return_velocity_scaling` | 0.04 | 0.04 | 仅用于经 MoveIt 预检的失败返回 |
| `teach_record_rate_hz` | 150 Hz | 150 Hz | 启动时限制为有限值 1–500 Hz |

夹爪反馈映射仍使用 90 mm 全量程，当前实机命令、视觉候选和抓取策略的开口限制
统一为 85 mm。1 mm 闭合容差不改变映射零点，也不扩大命令范围。

控制器现在会在连接硬件前拒绝空/非法 `arm_namespace` 和未知的 `cmd_arbitration`，
不再静默回退。直接 `MoveToPose` Action 会拒绝 NaN/Inf 位姿、零四元数，以及不在
0.01–300 秒范围内的时长。以上校验不改变表中的下游实机调试值。

## 明确保留的差异

- 安全回零继续调用随附 SDK 的全零 `ArmEndPos.safe_home()`；没有迁入下游非零 safe-home 姿态。
- 受控返回必须由调用方通过已配置的轨迹安全边界和碰撞预检后再执行；恢复模块本身不绕过 MoveIt 或底层轨迹校验。
- 原生 MotorBridge 必须先应用仓库内补丁。未提供 `get_state_with_sequence()` 时控制器会拒绝连接，而不是退回不可靠的缓存判断。

## 上机顺序

1. 在 VMware/Ubuntu 中运行 `python3 tools/setup_motorbridge_fresh_feedback.py --build-only`，
   审核产物后执行 `--install-user` 和 `--check-installed`。Windows 主机不要求安装
   MotorBridge。
2. 在 VMware 中运行 `python3 -m pytest -q tests/test_motorbridge_transport_safety.py`。
   该测试只使用 `/dev/pts/*` 虚拟串口，不连接真机；用于证明错误 CAN ID、Motor ID、
   DLC 不会推进反馈 sequence，并证明校零必须等待请求后的新鲜失能反馈。
3. 运行 `python3 tools/apply_rebotarm_control_safety_patch.py` 确认随附 SDK。
4. 不装负载，只执行连接；确认状态为 `CONNECTED_DISABLED`，七个电机状态均为 0。
5. 托住机械臂并准备急停，再通过现有控制界面显式使能；确认当前位置保持没有跳变。
6. 分别验证断开一只电机反馈、重复帧、总线超时是否触发错误和保护失能。
7. 夹爪先执行 0–10–0 mm 小行程，再逐步扩大；检查速度渐变、动态超时和到位中性释放。
8. 最后才验证抓取、保持超时、任务失败保持和受控返回。主从跟随工具另行按其 README 进行端口身份检查和显式 `--confirm-live-motion` 授权。

## 离线验证

- 主仓库离线测试不连接串口、不使能电机。MotorBridge POSIX 传输用例在 Windows
  跳过，必须在 VMware/Ubuntu 的补丁版 MotorBridge 环境中补跑；具体测试数量以当次
  `pytest` 输出为准。
- 独立主从工具测试使用其 `Python_SDK` 作为 `PYTHONPATH`，同样只运行离线测试；
  2026-09-06 结果为 `143 passed`。
- `git diff --check` 只允许 Windows 工作树的 LF/CRLF 转换警告，不允许空白错误。
