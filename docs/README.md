# reBotArm 文档中心

本目录是项目的正式文档入口。根 README 用于快速导航，设计细节、参数和验收记录放在这里维护。

## 使用者文档

1. [功能说明书](functional_specification_zh.md)：系统能力、数据流、操作流程和失败行为。
2. [部署与运行手册](deployment_guide_zh.md)：VMware/Ubuntu 环境、构建、首次运行和排障。
3. [配置参数参考](configuration_reference_zh.md)：控制频率、关节限位、夹爪、反馈和视觉参数。
4. [ROS 2 接口参考](ros_api_reference_zh.md)：主要 Topic、Service、Action 和命名空间。
5. [功能状态与验收矩阵](feature_status_zh.md)：已实现、实验性、兼容层和待验证事项。

## 开发者文档

- [系统架构](architecture.md)：包职责、依赖方向和代码归属规则。
- [开发维护指南](development_guide_zh.md)：新增功能、测试、文档和提交要求。
- [下游硬件安全迁移记录](downstream_hardware_safety_migration.md)：迁移范围和决策记录。
- [模式切换测试](mode_transition_test.md)：POS_VEL 与 MIT 切换验证。

## 包级说明

- [真机控制器](../src/rebotarmcontroller/README.md)
- [启动与配置](../src/rebotarm_bringup/README.md)
- [公共 ROS 接口](../src/rebotarm_msgs/README.md)
- [运动层](../src/rebotarm_motion/README.md)
- [示教](../src/rebotarm_teach/README.md)
- [遥操作](../src/rebotarm_teleop/README.md)
- [Dashboard](../src/rebotarm_dashboard/README.md)
- [MoveIt 配置](../src/rebotarm_moveit_config/README.md)
- [视觉](../src/rebotarm_vision/README.md)
- [标定](../src/rebotarm_calibration/README.md)
- [仿真](../src/rebotarm_simulation/README.md)
- [语音控制](../src/rebotarm_voice_control/README.md)
- [旧交互控制兼容层](../src/rebotarm_interactive_control/README.md)
- [独立主从跟随工具](../star_arm_102_rebot_b601_follow/README.md)

## 专题手册

- [常用命令](rebotarm_common_commands.md)
- [功能命令](rebotarm_feature_commands.md)
- [视觉抓取命令](visual_grasp_commands.md)
- [视觉抓取七层参数](visual_grasp_seven_layer_params.md)
- [MuJoCo 验收](mujoco_acceptance_zh.md)
- [MuJoCo Pick](mujoco_pick_zh.md)
- [Real2Sim 桥接](real2sim_bridge_zh.md)
- [Sim2Real 工作流](sim2real_workflow_zh.md)
- [语音控制阶段说明](voice_control_stage4_zh.md)

`docs/superpowers/` 保存历史设计与实施计划。它们用于追踪决策，不代表当前运行接口；发生冲突时，以源码、当前配置和上述正式手册为准。

## 文档维护规则

- 参数必须注明单位、默认值、配置来源和适用边界。
- 未经过实机验证的功能必须明确标为“待实机验收”或“实验性”。
- 删除或改名 ROS 接口时，同步修改接口参考和相关包 README。
- 新增包时至少补充职责、入口、依赖方向、配置和测试方法。
- 文档命令必须能够复制执行，示例中的危险动作应带安全提示。
