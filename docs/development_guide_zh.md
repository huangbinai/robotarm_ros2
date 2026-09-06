# reBotArm 开发维护指南

## 1. 开发原则

1. 真机安全边界必须在 `rebotarmcontroller` 再检查一次，不能只依赖 UI 或规划层。
2. 原始反馈和外部输入均视为不可信数据，先验证有限值、范围、序号和新鲜度。
3. 新功能按职责放入已有包，不通过复制代码绕开依赖边界。
4. 实机能力先提供 dry-run、plan-only 或仿真路径，再开放真实执行。
5. 失败必须可观察：返回明确结果、更新状态并保留可诊断错误码。

## 2. 代码归属

| 修改内容 | 应放入 |
| --- | --- |
| SDK、串口、电机模式、反馈、使能、急停 | `rebotarmcontroller` |
| 轨迹生成、重定时、规划适配、碰撞和跟踪保护 | `rebotarm_motion` |
| 示教文件、录制和回放业务 | `rebotarm_teach` |
| 键盘、网页命令适配和可视关节桥 | `rebotarm_teleop` |
| HTTP、SSE、页面和状态聚合 | `rebotarm_dashboard` |
| URDF/SRDF、规划组、MoveIt 参数 | `rebotarm_moveit_config` |
| 相机、检测、候选、抓取策略 | `rebotarm_vision` |
| 手眼、TCP、TF 和残差工具 | `rebotarm_calibration` |
| MuJoCo、Real2Sim、Gym 环境 | `rebotarm_simulation` |
| 文本/语音意图和任务路由 | `rebotarm_voice_control` |
| launch 与跨包配置组装 | `rebotarm_bringup` |
| 旧路径转发 | `rebotarm_interactive_control` |

完整依赖规则见[系统架构](architecture.md)。

## 3. 新功能最低交付内容

- 实现代码和清晰的公共接口；
- 参数默认值、类型、单位、有效范围和异常行为；
- 纯逻辑单元测试；
- ROS 接线或 launch 静态测试；
- 对应包 README 和专题文档更新；
- 如果触及真机，提供仿真/只规划验证和实机验收步骤；
- 如果改动公共接口，记录兼容策略。

## 4. 参数规则

- 距离统一使用米，角度使用弧度，时间使用秒，频率使用 Hz，力矩使用 N·m。
- 参数进入核心逻辑前检查类型、有限值、范围及数组长度。
- 同一物理限制只设一个权威来源，其他模型通过测试检查同步。
- 反馈容差不能反向扩大运动目标范围。
- 路径参数不写开发者个人绝对路径；模型缺失时应允许源码构建，并在运行时明确报错。

## 5. 并发与生命周期

- 机械臂和夹爪命令必须取得对应资源 lease。
- 取消回调要触发实际停止，不能只修改 Action 状态。
- 退出循环后重新检查取消、抢占和硬件状态，再宣布成功。
- shutdown 的失能和断开分别验证，任何未知物理状态都不能虚报成功。
- 共享总线访问使用统一锁和调度器，禁止各节点自行并发轮询 SDK。

## 6. 测试

Windows 或 Ubuntu 的基础检查：

```bash
python -m pytest -q
python -m compileall src -q
git diff --check
```

修改包边界时运行：

```bash
python -m pytest tests/test_package_layering.py -q
```

修改硬件安全时优先运行：

```bash
python -m pytest \
  tests/test_controller_safety_boundaries.py \
  tests/test_hardware_feedback_lifecycle.py \
  tests/test_feedback_sequence.py \
  tests/test_mode_transition.py -q
```

修改视觉链路时优先运行：

```bash
python -m pytest \
  tests/test_candidate_gate_policy.py \
  tests/test_visual_grasp_wiring.py \
  tests/test_visual_failure_recovery.py -q
```

## 7. 文档规范

每个 ROS 包的 README 至少包含：职责、主要入口、配置、依赖边界、验证和状态。专题文档文件名使用小写英文加 `_zh.md`，链接使用仓库相对路径。

代码与文档发生冲突时先确认源码事实，再同步修改文档；不要在 README 中复制大量易过期的完整参数表，应链接到配置参考。

## 8. 提交前检查

- `git status --short` 中没有模型缓存、日志、构建产物或私密配置；
- 没有个人绝对路径、IP、密钥和串口硬编码；
- 新增文件已被包安装规则包含，或明确只属于源码文档；
- 变更没有破坏兼容层和包依赖方向；
- 测试结果、未验证项和实机风险写入交付说明。
