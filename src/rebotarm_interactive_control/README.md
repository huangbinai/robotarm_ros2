# rebotarm_interactive_control

旧版交互控制导入路径和可执行入口的兼容包。

历史模块通过薄包装转发到 `rebotarm_motion`、`rebotarm_teach`、`rebotarm_teleop` 和 `rebotarm_dashboard`。新代码应直接依赖这些正式包，不得在本包继续添加轨迹、示教、网页或遥操作实现。

推荐使用 `rebotarm_bringup` 中的当前 launch；RViz 末端拖动采用 MoveIt MotionPlanning 工作流。兼容层的保留和移除应配套旧导入测试及迁移说明。

职责边界见[系统架构](../../docs/architecture.md)。
