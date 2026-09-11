# rebotarm_interactive_control

旧版交互控制导入路径和可执行入口的兼容包。

历史模块通过薄包装转发到 `rebotarm_motion`、`rebotarm_teach`、`rebotarm_teleop` 和 `rebotarm_dashboard`。新代码应直接依赖这些正式包，不得在本包继续添加轨迹、示教、网页或遥操作实现。

推荐使用 `rebotarm_bringup` 中的当前 launch；RViz 末端拖动采用 MoveIt MotionPlanning 工作流。兼容层的保留和移除应配套旧导入测试及迁移说明。

交互配置已迁至 `rebotarm_bringup/config/`，正式包不再依赖本兼容包。旧 SDK `pose_preview_solver` 导入路径已退休，其余兼容转发继续保留。职责边界与迁移结果见[项目架构](../../docs/project_architecture_zh.md)。
