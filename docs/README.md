# reBotArm 文档中心

本目录是项目当前维护入口。维护基线为 2026-09-11；五项边界迁移后包含 13 个 ROS 包，新增共享描述包，语音继续排除在当前范围之外。

## 长期维护先看这里

最新测试：[功能问题修复与复测](acceptance_records/2026-09-11-functional-fixes.md)。启动解释器、六轴录制/回放、停止终态和 live Open3D 已修复并复测；[第一轮记录](acceptance_records/2026-09-11-functional-regression.md)保留当时失败证据。

1. [项目架构与依赖边界](project_architecture_zh.md)：包职责、代码/资源/运行时依赖、配置归属、架构债务和迁移顺序。
2. [当前阶段与交付路线](project_stage_zh.md)：当前处于 S1 工程基线收敛阶段，区分实现、局部验证与实机验收。
3. [验收依据与执行方法](acceptance_criteria_zh.md)：各阶段退出条件、量化判据、检查命令和重验规则。
4. [最新边界迁移与验证记录](acceptance_records/2026-09-11-boundary-migration.md)及[验收记录模板](acceptance_records/TEMPLATE.md)：每次结论与版本、配置、证据绑定。[较早基线](acceptance_records/2026-09-11-baseline.md)保留迁移前状态。
5. [本机 ROS 视觉迁移验证](acceptance_records/2026-09-11-local-ros-perception.md)：73 项软件回归与真实推理部署缺项分别记录。
6. [实际 GPU 环境部署与验收](acceptance_records/2026-09-11-perception-runtime.md)：依赖/原生扩展已就位，真实模型离线样例与 ROS 链通过；相机未连接。
7. [Open3D 迁移及实拍验证](acceptance_records/2026-09-11-open3d-viewer.md)：Windows 工具已删除，Gemini 2 已采集，彩色点云/抓取候选图已生成。

架构自动检查：`python3 tools/check_architecture.py --strict-deployment`；规则见 [architecture_rules.json](../tools/architecture_rules.json)。当前历史依赖例外已清零，CI 使用严格门禁；静态通过不等于实机验收通过。

## 使用者文档

1. [功能说明书](functional_specification_zh.md)：系统能力、数据流、操作流程和失败行为。
2. [启动入口](../src/rebotarm_bringup/launch/README.md)：各场景 launch 的用途、默认模式和常用命令。
3. [配置与关键接口归属](project_architecture_zh.md)：当前配置源、接口提供方及后端差异。
4. [功能状态与证据矩阵](project_stage_zh.md)：当前实现、局部验证、兼容层和待验证事项。
5. [Ubuntu 本机 ROS 视觉路线](local_ros_vision_zh.md)：相机、YOLO、GraspNet 的本地节点通信与依赖/标定要求。

## 开发者文档

- [架构维护规则](project_architecture_zh.md)：新增功能归属、依赖门禁、配置迁移和兼容策略。
- [变更与重验规则](acceptance_criteria_zh.md)：按影响范围验证，保存证据后更新阶段。

## 包级说明

- [真机控制器](../src/rebotarmcontroller/README.md)
- [启动与配置](../src/rebotarm_bringup/README.md)
- [公共 ROS 接口](../src/rebotarm_msgs/README.md)
- [共享模型与电机参数](../src/rebotarm_description/README.md)
- [运动层](../src/rebotarm_motion/README.md)
- [示教](../src/rebotarm_teach/README.md)
- [遥操作](../src/rebotarm_teleop/README.md)
- [Dashboard](../src/rebotarm_dashboard/README.md)
- [MoveIt 配置](../src/rebotarm_moveit_config/README.md)
- [视觉](../src/rebotarm_vision/README.md)
- [标定](../src/rebotarm_calibration/README.md)
- [仿真](../src/rebotarm_simulation/README.md)
- [旧交互控制兼容层](../src/rebotarm_interactive_control/README.md)
- [独立主从跟随工具](../star_arm_102_rebot_b601_follow/README.md)

## 专题手册

- [功能命令](rebotarm_feature_commands.md)
- [视觉抓取说明](../src/rebotarm_vision/README_zh.md)
- [MuJoCo、Real2Sim 与任务环境](../src/rebotarm_simulation/README_mujoco.md)

旧版本的架构、部署、参数和专题文档在当前工作区中已有删除，不再列为有效入口；需要历史背景时通过 Git 检索。旧命令页与当前行为冲突时，以当前源码、launch 和本次维护基线为准。

跨模块状态、决策、代码审查和未决问题统一保存在本目录。发生冲突时，以源码、当前配置和测试证据为准；旧仓库文档只用于历史追溯。

## 文档维护规则

- 参数必须注明单位、默认值、配置来源和适用边界。
- 未经过实机验证的功能必须明确标为“待实机验收”或“实验性”。
- 删除或改名 ROS 接口时，同步修改接口参考和相关包 README。
- 新增包时至少补充职责、入口、依赖方向、配置和测试方法。
- 文档命令必须能够复制执行，示例中的危险动作应带安全提示。
