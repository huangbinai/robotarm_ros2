# 2026-09-11 Windows 工具清理与 Ubuntu Open3D 迁移

结论：Windows 视觉启动/HTTP 中转代码已删除；彩色点云与抓取候选窗口迁为独立 ROS 只读节点。已用真实 Gemini 2 帧、YOLO26/GraspNet 权重和 Open3D 渲染生成预览。没有执行机械臂运动。

## 删除范围与保留能力

删除 10 个旧文件：

- `tools/windows_start_grasp_ai_stack.ps1`
- `tools/windows_start_graspnet_bridge.ps1`
- `tools/windows_start_yolo_server.ps1`
- `tools/windows_graspnet_baseline_bridge.py`
- `tools/windows_mjpeg_server.py`
- `tools/graspnet_baseline_inference.py`（旧转发入口）
- vision 包的 `camera/network_mjpeg_driver.py`
- vision 包的 `detector/network_detection_client.py`
- vision 包的 `converters/network_detection_msgs.py`
- vision 包的 `network_graspnet_client.py`

另移除包内 Windows DLL 搜索、JSON 候选转消息和已无调用的旧 ROI 预处理帮助类。检查确认这些路径没有当前运行消费者。旧代码可通过 Git 历史恢复；不在源码中继续保留第二套视觉链。

本机共用的 `tools/yolo26s-seg.pt`、GraspNet 权重/源码、包内 `graspnet_inference.py`、CUDA 依赖和已完成的 ROS 感知/执行接口继续保留。

## Open3D 功能

- 新增 `graspnet_visualization.py` 和 `graspnet_viewer_node.py`。
- 精确同步同一采集时间的 Image、depth、CameraInfo 与 GraspCandidateArray，重建彩色点云后叠加候选姿态/开度。
- 最高分候选绿色，其余橙红色；候选夹爪手指尺寸为显示示意，不代替真实 URDF/碰撞模型。
- 空候选不绘制替代夹爪，过期输入清空，新的空结果不能重新显示排队的旧候选。
- 鼠标交互，`R` 重置视角，`S` 保存 PNG 与 PLY。
- 只订阅 ROS 消息，没有相机、模型推理、Service/Action 运动客户端。窗口关闭不影响其他推理进程。
- 独立启动入口 `rebotarm_vision/graspnet_viewer.launch.py`，视觉组合入口提供 `show_open3d` 开关，默认 false。

## 验证

当前 13 个包、294 个 Python 文件，严格架构检查通过。全仓回归 **86 passed in 6.85s**；JUnit 见 [open3d-viewer-tests.xml](2026-09-11-open3d-viewer-tests.xml)。新增回归覆盖点云投影/颜色/下采样、错误候选、空结果、时间同步、过期清空、只读边界及可选 launch 接线。

设备：Orbbec Gemini 2，固件 1.4.92，USB 3（5 Gbps）；`70-rebotarm-gemini2.rules` 已配置，当前用户具有设备写权限。之前的短时采集已取得 10 组完整色深帧，注册输出 1280×800，设备内参为：

```text
fx=610.46435546875
fy=610.46435546875
cx=643.04833984375
cy=403.52886962890625
```

本次实拍使用正式相机驱动、真实 YOLO26/GraspNet 权重及相同 ROS 候选消息/可视化几何代码。记录结果：YOLO 检测到 `chair`，原始抓取候选 3 个，显示点数 30,000。Open3D 在当前 DISPLAY 创建图形上下文并导出 PNG/PLY，已目视检查输出。

本机输出（调试产物未纳入 Git）：

```text
.codex_tmp/graspnet-previews/gemini2-yolo.png
.codex_tmp/graspnet-previews/gemini2-graspnet.png
.codex_tmp/graspnet-previews/gemini2-graspnet.ply
```

该图证明实拍点云、候选转换和渲染链可用。当前识别对象是椅子，不能据此认定适合机械臂抓取；候选未经过 MoveIt 可达性/执行碰撞筛选。没有改变手眼/TCP 参数，没有机械臂运动或实际夹取结论。ROS viewer 的同步/失效行为由消息测试验证，本轮截图通过直接调用相同可视化代码生成，不冒充长时间 live GUI 验收。

工作区仍包含未提交变更；发布时需要固定最终提交和环境。原有历史验收记录保留当时结果，不回写为当前状态。
