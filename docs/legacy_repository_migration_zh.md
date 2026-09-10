# 旧仓库历史迁移记录

## 1. 范围与权威来源

本记录用于追溯从 Seeed 派生工作区迁移到独立 ROS 2 仓库的过程。当前唯一维护入口为：

```text
https://github.com/huangbinai/robotarm_ros2
```

旧仓库中的命令、目录和算法路线不再自动视为当前事实。当前行为以本仓库源码、配置、测试和 `docs/` 为准。

目标部署架构是 Ubuntu 24.04 + ROS 2 Jazzy 上的一体化运行环境，包括机械臂控制、MoveIt、Gemini 2、YOLO、GraspNet 和仿真。Windows 视觉脚本仅在 Ubuntu 相机与推理链完成验收前作为兼容路径保留。

## 2. 旧 Git 历史备份

旧工作区分支 `codex/rebotarm-vision-stage1` 比其 `origin/main` 多 16 个本地提交。完整历史已保存为：

```text
D:\rebot\archives\reBot-DevArm-legacy-20260910.bundle
```

校验信息：

```text
SHA-256: e2dc8b36ceaa45e5f2554f8b378b854f25c57e4e0aa3aae0880bcfb57666fb91
tip:     b6b4de617df6895d5facf4cef8f22d03dcfb34b2
base:    53f5d5f212b67d7fafd41deca6c68d7c7f52f5c0
```

Windows 恢复示例：

```powershell
git clone "D:\rebot\archives\reBot-DevArm-legacy-20260910.bundle" legacy-reBot-DevArm
```

原始未提交文档另存于：

```text
D:\rebot\archives\reBot-DevArm-legacy-20260910\docs
```

## 3. 已归档的 16 个提交

| 提交 | 日期 | 内容 |
| --- | --- | --- |
| `8dc6e59` | 2026-06-09 | 实现早期 GraspNet 稳定夹取功能并加入相关工具、文档和硬件资料 |
| `b6f45e4` | 2026-07-11 | 定义 GraspNet-only 迁移设计 |
| `bab813a` | 2026-07-11 | 编写 GraspNet-only 实施计划 |
| `9d2b65e` | 2026-07-11 | 更新 GraspNet 操作与测试指南 |
| `d44090e` | 2026-07-11 | 新增 GraspNet 验收指南 |
| `73bf45d` | 2026-07-11 | 增加构建和实时状态预览步骤 |
| `3e59533` | 2026-07-11 | 简化真机 GraspNet 测试指南 |
| `9b0f426` | 2026-07-11 | 恢复视觉抓取操作手册 |
| `c1b4846` | 2026-07-11 | 设计抓取后返回视觉准备位 |
| `54d98bf` | 2026-07-11 | 记录零抓取前补偿默认值 |
| `4250015` | 2026-07-11 | 删除废弃的抓取前补偿参数 |
| `daafdc5` | 2026-07-12 | 设计电机模式无扰切换 |
| `3f23ff1` | 2026-07-12 | 设计 MuJoCo 仿真底座 |
| `a53a4d9` | 2026-07-12 | 编写 MuJoCo 仿真实施计划 |
| `c7179ac` | 2026-07-12 | 编写电机模式无扰切换实施计划 |
| `b6b4de6` | 2026-07-12 | 翻译无扰切换实施计划 |

这些提交用于历史追溯，不直接 cherry-pick 到当前仓库。早期提交同时包含旧目录布局、硬件资料变更和已淘汰路线，直接合并会覆盖当前重构结果。

## 4. 保留的有效决策

- 正式视觉候选由 GraspNet 产生；不得恢复无有效候选时生成简化抓取动作的旧回退。
- 候选、计划和反馈必须校验时间戳、新鲜度、有限值、范围和当前机器人状态。
- 无明确实机授权时只执行测试、仿真、RViz、dry-run 或 `plan_only`。
- 测试通过不等于真机安全验收；真实串口、反馈时延、失能、急停和故障恢复必须在 Ubuntu/实机验证。
- TCP、手眼外参、相机内参、TF 方向和模型路径必须使用现场设备重新验收。
- 第三方源码通过 `rebotarm_dependencies.repos` 锁定，项目定制通过 `patches/` 保存；不得迁入 Windows 构建缓存和二进制安装目录。
- GraspNet 权重按 `models/MANIFEST.sha256` 校验，不能仅凭文件名判断版本。

## 5. 未迁入的旧内容

- ordinary-grasp 正式运行路线及其备用回退；
- 指向旧下载目录、旧 Ubuntu 工作区或个人绝对路径的命令；
- 已被当前包分层替代的旧节点、topic 和 launch 入口；
- Windows `.dll`、本地 `build/`、`install/`、缓存和 SDK 安装目录；
- 未经当前源码和配置验证的历史测试数量或实机结论。

## 6. Ubuntu 迁移完成门槛

只有以下项目全部完成，才能认为迁移结束并停用 Windows 兼容视觉链：

1. 在 Ubuntu 克隆本仓库并运行 `tools/bootstrap_ubuntu_dependencies.sh`。
2. 安装 ROS 依赖，完成 `colcon build --symlink-install`。
3. 安装 Linux `pyorbbecsdk`，验证 Gemini 2 彩色、深度、内参和时间戳。
4. 校验 YOLO 与 GraspNet 权重，并验证 Ubuntu GPU 推理。
5. 完成无硬件测试、MoveIt/RViz 和 MuJoCo 验收。
6. 完成只连接不使能、反馈新鲜度、可靠失能和急停验收。
7. 从单轴小幅动作逐步完成轨迹、夹爪、示教和视觉 `plan_only`。
8. 最后以低速、小批次方式完成真实抓取与失败恢复统计。

在这些门槛完成前，旧目录和 Windows 兼容路径只能视为待退役资源，不能视为已经安全删除或停止维护。
