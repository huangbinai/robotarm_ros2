# rebotarm_vision

Ubuntu 本机 RGB-D、YOLO 分割、GraspNet 6D 候选、筛选和抓取执行包。

正式链路使用三个 ROS 节点：

- `rebotarm_vision_node`：Gemini 2 完整 RGB-D，彩色对齐到深度并去畸变，发布 Image/CameraInfo。
- `rebotarm_yolo_node`：订阅图像，发布保留采集时间戳的 Detection2DArray 和分割 polygon。
- `rebotarm_graspnet_baseline_node`：精确同步 RGB-D/内参/检测，整场景 GraspNet 推理，再用 YOLO 筛选，发布 GraspCandidateArray。

正式入口不再使用 Windows JSON/HTTP；GraspNet 推理封装安装在包内。候选、计划和 TF 保留采集时间；输入陈旧、断流或失败时清空/拒绝。

旧 Windows 启动/网络适配文件已删除。Open3D 彩色点云和夹爪候选窗口由可选只读节点 `rebotarm_graspnet_viewer` 提供：`ros2 launch rebotarm_vision graspnet_viewer.launch.py`。感知组合入口也支持 `show_open3d:=true`；窗口按 `S` 保存 PNG/PLY。

启动、依赖、模型路径、标定和验收限制见[Ubuntu 本机 ROS 视觉路线](../../docs/local_ros_vision_zh.md)。本机 GPU 和 Gemini 2 USB 3 采集已通过验证，已生成实拍点云/候选预览；实际标定精度及机械臂抓取仍待验收。
