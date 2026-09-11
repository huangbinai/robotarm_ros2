# 视觉抓取使用说明

正式技术路线已迁移为 **Ubuntu 本机相机 → ROS YOLO 节点 → ROS GraspNet 节点 → 候选过滤 → 受控抓取**。

本页不再维护另一份 Windows 网络配置或启动命令。完整职责、Topic、同步/时间戳、设备内参、模型路径、依赖检查、ArUco 标定和启动示例统一维护在[本机 ROS 视觉路线](../../docs/local_ros_vision_zh.md)。

模型推理与机器人执行分别验收。合成回归、真实 GPU 模型离线 RGB-D/ROS 链、Gemini 2 USB 3 色深采集及 Open3D 实拍预览已验证；实际标定精度及实机抓取仍未验收。

相关文档：[项目架构](../../docs/project_architecture_zh.md)、[阶段状态](../../docs/project_stage_zh.md)、[验收依据](../../docs/acceptance_criteria_zh.md)。
