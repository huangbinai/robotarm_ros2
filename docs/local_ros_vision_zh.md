# Ubuntu 本机 ROS 视觉抓取路线

2026-09-11 起，正式视觉链在 Ubuntu 上运行，不再经 Windows HTTP/JSON 中转。相机、YOLO 和 GraspNet 是 `rebotarm_vision` 中三个独立 ROS 节点；运动和抓取执行仍使用既有 motion/MoveIt/控制器边界。

## 数据链与职责

```text
Gemini 2 USB
  → rebotarm_vision_node：完整 RGB-D、彩色对齐到深度、去畸变、设备内参
  → ROS Image + CameraInfo
  → rebotarm_yolo_node：YOLO 检测/分割，保留采集时间戳
  → ROS Detection2DArray
  → rebotarm_graspnet_baseline_node：同帧同步，整场景点云推理，YOLO 投影筛选
  → ROS GraspCandidateArray
  → candidate_ik_filter：采集时刻 TF、姿态/工作空间/IK/碰撞检查
  → ROS GraspPlan
  → plan_only 或运动层的受控抓取执行
```

| Topic | 类型 | 提供方 | 约定 |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 相机节点 | BGR8；已经对齐到深度相机并去畸变，保留原 topic 名以兼容消费者 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 相机节点 | mono16，单位 mm；设备原始值先乘 SDK depth scale |
| `/camera/depth/camera_info`、`/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机节点 | 同一注册后像素网格和标定 K；图像已去畸变，因此 D 为 0 |
| `/grasp/detections` | `rebotarm_msgs/Detection2DArray` | YOLO 节点 | bbox/类别/置信度/可用的 mask polygon；header 来自输入图像 |
| `/grasp/graspnet_candidates` | `rebotarm_msgs/GraspCandidateArray` | GraspNet 节点 | 6D 候选；数组和每项 header 均保留输入图像的时间/坐标系 |
| `/grasp/filtered_plan` | `rebotarm_msgs/GraspPlan` | 候选过滤节点 | 机器人基坐标系中的计划；仍保留采集时间，不重置为规划结束时间 |

所有感知图像和候选采用 `camera_depth_frame` 的光学坐标约定（x 向右、y 向下、z 向前）。驱动选择 **C2D：彩色对齐到深度**，所以继续使用现有手眼配置的深度相机坐标系，而不是把彩色相机坐标直接改名。实际内参、外参、TCP 和对齐效果仍须在当前设备验证。

相机节点在获取/处理帧之前生成同一 ROS 时间戳，让采集/注册的耗时计入年龄；设备时间戳用于检查色深偏差和重复/倒序帧。缺少完整色深帧、对齐失败、尺寸不符或真实内参缺失时不发布；没有缓存图像或猜测内参的回退。

GraspNet 使用四路精确时间同步：color、depth、CameraInfo、detections。检测框不会与另一时刻的深度配对。同步队列有界，推理使用最近一组输入；默认 `max_frame_age_sec=1.5`，推理前后都检查。空检测、推理失败或输入断流会发布空候选。过滤节点也检查源帧年龄，TF 使用采集时间，执行器在接收计划和准备执行时检查源帧年龄。

YOLO 用于选择目标及筛选 GraspNet 结果，**不改成 YOLO 规则生成抓取姿态**。GraspNet 保留完整场景点云作为网络与接触碰撞检查输入，再用最高置信度目标的 mask/bbox 投影过滤结果；没有有效候选时不生成替代动作。

## 配置与本地资产

| 位置 | 内容 |
| --- | --- |
| vision `config/camera.yaml` | 相机、注册坐标系、YOLO、采样/推理频率 |
| vision `config/graspnet_policy.yaml` | ROS 话题、同步队列、输入年龄、深度范围、GraspNet 配置 |
| vision `config/handeye.yaml` | `end_link → camera_depth_frame` 的现场手眼参数 |
| `third_party/graspnet-baseline` | GraspNet 源码及其 pointnet2/knn 编译扩展 |
| `models/graspnet/checkpoint-rs.tar` | GraspNet 权重，按 `models/MANIFEST.sha256` 校验 |
| `tools/yolo26s-seg.pt` | 本工作区的首选 YOLO 分割权重 |

YOLO 权重查找顺序：显式 `yolo_model_path` → 当前工作区 `tools/yolo26s-seg.pt` → 已安装视觉包的 `models/yolo11n-seg.pt`。运行启动不下载模型。独立部署应显式指定权重，避免把 YOLO11 的包内回退误认为 YOLO26。

GraspNet 默认在 `REBOTARM_WORKSPACE` 下查找源码和权重；环境脚本会设置该变量。也可以通过 `graspnet_model_root` 和 `graspnet_checkpoint_path` 指定本地路径。推理封装属于已安装的 `rebotarm_vision.graspnet_inference`，ROS 节点不依赖 `tools/` 在 Python import 路径中。

默认输出最多 **5 个**通过筛选的 GraspNet 候选，Open3D 默认也最多显示 5 个；有效结果不足 5 个时按实际数量输出，不补造候选。完整/感知预览入口可用 `graspnet_max_grasps` 覆盖。

与 Git 历史中 Windows 启动脚本的默认配置相比，点云参数并非全部相同：

| 参数 | 原 Windows 默认 | 当前 Ubuntu 默认 |
| --- | --- | --- |
| 有效深度范围（沿相机光学 Z 轴） | 0.05–1.50 m | 0.15–1.20 m，推理与查看器一致 |
| GraspNet 网络采样点数 | 20,000 | 20,000 |
| Open3D 显示点数上限 | 8,000（Windows 包装脚本覆盖值） | 30,000 |
| 碰撞检查 voxel_size | 0.01 m | 0.01 m |
| collision_thresh | 0.01 | 0.01 |
| 屏幕点大小 | 4.0 | 2.0 |

此处比较仓库默认值，不能追溯旧 Windows 终端曾使用的临时覆盖。当前深度按设备 SDK 的 scale 转为 mm，再由真实 CameraInfo 投影；手眼和 TCP 配置没有因候选数量调整而修改。

相机、YOLO、GraspNet 和标定节点应使用能看到 ROS 的同一 Python 环境。`vision_python_executable` / `REBOTARM_VISION_PYTHON` 对相机、YOLO 和 GraspNet 都生效；不是只给其中一个节点换解释器。

工作区构建统一使用 `bash tools/build_workspace.bash`，该脚本由 `.venv-ros/bin/python` 运行 colcon，生成同一解释器的 ROS console-script，并自动检查 shebang。仅激活 venv 后调用系统 `colcon` 不足以保证这一点。可用 `python tools/check_python_entrypoints.py` 单独检查；不要手动修改 install 中的脚本。

## 依赖检查与运行

先完成当前工作区构建并 source：

```bash
source tools/source_ubuntu_env.bash
python tools/check_local_perception.py
```

检查器验证模块、原生扩展加载、文件、权重哈希和 CUDA。加 `--probe-camera` 会枚举 Orbbec 设备，但不采集图像或操作机械臂。结果为 false 时先解决报告中的缺项。

当前 `.venv-ros` 已安装 PyTorch 2.5.1+cu121、torchvision 0.20.1+cu121、Ultralytics 8.4.147、Open3D 0.19.0、相机 SDK `pyorbbecsdk2==2.0.18`（Python 仍导入 `pyorbbecsdk`），以及已编译的 PointNet2、KNN、Grasp NMS。NumPy 保持 1.26.4；ROS 环境实际加载的 OpenCV 是系统 4.6.0，pip 安装项 `opencv-python==4.11.0.86` 不等于当前 import 的版本。

GraspNet 源码使用 `rebotarm_dependencies.repos` 的固定提交并应用 `patches/`，权重已校验。相机 SDK 使用与源码版本号对应的官方 2.0.18 Python 3.12 预编译 wheel；本轮没有从该源码提交编译 SDK，因此不能把 wheel 和源码提交视为字节级相同产物。

RTX 4060 上真实 YOLO/GraspNet 离线样例推理及 ROS 候选通信已通过，详见[部署验收记录](acceptance_records/2026-09-11-perception-runtime.md)。随后 Gemini 2 已识别并以 USB 3（5 Gbps）连接，权限规则生效，完整色深采集/设备内参读取及实拍 Open3D 预览已通过。场景对齐精度、现场标定和实机抓取仍须单独验收。

### CUDA 安装位置与重建

CUDA 12.1 编译组件安装在项目 `third_party/cuda-12.1`，GCC 12 在 `third_party/gcc-12`；Python 推理依赖在 `.venv-ros`。没有更换系统驱动、没有写入 `/usr/local/cuda`。`nvidia-smi` 的 CUDA 字段表示驱动支持能力，不表示 nvcc 工具链位置。

只在编译扩展时加载工具链：

```bash
source tools/source_ubuntu_env.bash
source tools/source_graspnet_build_env.bash
nvcc --version
bash tools/build_graspnet_extensions.bash
```

当前编译目标是 RTX 4060 的 `sm_89`，并限制单编译任务以控制内存。换 GPU、PyTorch、Python 或 CUDA 后须按新设备重新编译，不能直接复用本机 wheel。

重装 Python 依赖的版本基线：

```bash
source tools/source_ubuntu_env.bash
python -m pip install torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cu121
python -m pip install Cython==3.0.12
python -m pip install --no-build-isolation \
  -c src/rebotarm_vision/constraints-ubuntu.txt \
  -r src/rebotarm_vision/requirements-local.txt
```

`constraints-ubuntu.txt` 固定已验证的直接依赖，不是完整传递依赖锁文件。编译工具链的来源和校验值见部署记录；这些命令假设 ROS、项目虚拟环境和固定版本第三方源码已经就位。GraspNetAPI 的第二个补丁将离线评估依赖拆为可选 extras，避免旧 NumPy 1.23.4 限制破坏 Python 3.12/ROS 环境。

不接相机也能复核实际模型与 ROS 链：

```bash
source tools/source_ubuntu_env.bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python tools/verify_perception_models.py
python tools/verify_perception_ros.py --domain 182
python tools/check_local_perception.py --probe-camera
```

前两条使用 GraspNet 官方录制的 RGB-D 样例和真实 GPU 模型；候选数量可能随点云采样而变，运行成功不等于抓取成功。第三条检查当前相机枚举；已发现设备也不代表权限/帧采集检查自动通过。

只启动本机相机、YOLO 及坐标配置：

```bash
ros2 launch rebotarm_vision vision.launch.py
```

启动感知、GraspNet 与标记预览，不启动机械臂控制器：

```bash
ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py \
  graspnet_model_root:="$PWD/third_party/graspnet-baseline" \
  graspnet_checkpoint_path:="$PWD/models/graspnet/checkpoint-rs.tar" \
  yolo_model_path:="$PWD/tools/yolo26s-seg.pt"
```

独立感知入口没有完整机器人状态/MoveIt，不能仅凭候选显示判定可执行计划。真实相机加仿真执行使用下面的包装入口，固定硬件关闭：

```bash
ros2 launch rebotarm_bringup real_perception_sim_execution.launch.py \
  graspnet_model_root:="$PWD/third_party/graspnet-baseline" \
  graspnet_checkpoint_path:="$PWD/models/graspnet/checkpoint-rs.tar"
```

不要同时启动上述多个入口，它们会重复占用 USB 相机及发布相同话题。真实机器人执行继续按[验收依据](acceptance_criteria_zh.md)逐项完成，切换感知链不代表实机抓取已验收。

ArUco/TCP 标定的 `reference_mode=aruco` 同样订阅本机 `/camera/color/image_raw` 和 `/camera/color/camera_info`，使用采集时刻 TF；旧 `aruco.snapshot_url` 和写死内参已移除。标定节点等待操作者输入时仍持续接收 ROS 数据。

## 旧链路退役与验收

正式节点不接受 `source_mode=network` / `camera.type=network_mjpeg`；`source_mode=local_backend` 仅作为本机 ROS 模式旧名称兼容。正式 launch/config 不再含 HTTP 地址、轮询频率或 Windows IP。

旧 Windows 启动脚本、HTTP 图像/候选服务、网络适配类和 tools 下的推理兼容转发文件已删除。维护实现仅在 ROS 包中；旧实现可从 Git 历史恢复。YOLO26 权重 `tools/yolo26s-seg.pt` 仍是本机节点使用的资产，继续保留。以后分布式部署通过 ROS 话题连接，不恢复 JSON/HTTP 中转。

## Open3D 彩色点云与抓取候选窗口

`rebotarm_graspnet_viewer` 是独立只读 ROS 节点。GraspNet 完成一次推理时，把该次实际输入的 color、depth、CameraInfo 以可靠 QoS 发布到 `/grasp/preview/color`、`/grasp/preview/depth`、`/grasp/preview/camera_info`；查看器将它们与同时间戳的 GraspCandidateArray 精确配对，不再事后从高速原始相机流寻找可能已丢失的帧。ROS 消息接收线程与主线程 Open3D 渲染分开，避免窗口刷新挤占订阅处理。

这些预览话题保留原始时间戳，只按推理频率发布，队列有界；没有放宽 1.5 s 年龄限制。查看器不打开相机、不加载另一份模型、不发布运动命令，关闭窗口不影响推理节点。

已经运行本机感知节点时，只开启窗口：

```bash
source tools/source_ubuntu_env.bash
ros2 launch rebotarm_vision graspnet_viewer.launch.py
```

从零启动感知与 Open3D，关闭不需要的 RViz/IK 预览：

```bash
source tools/source_ubuntu_env.bash
ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py \
  show_open3d:=true use_local_rviz:=false start_candidate_ik_filter:=false
```

完整视觉或真实感知/仿真执行入口也支持 `show_open3d:=true`，默认 false。不要重复启动两个感知组合去争用同一个相机。

窗口显示彩色场景点云和前 5 个原始抓取候选：绿色为显示集合中最高分候选，其他为橙红色，最佳候选带 XYZ 坐标轴。鼠标旋转/平移/缩放，`R` 在下一帧重置视角，`S` 将当前视角 PNG 和点云 PLY 保存到启动目录下 `.codex_tmp/graspnet-previews/`。

候选位姿和开度来自推理结果；可视夹爪的手指长度/厚度仅是显示示意，不是实际机械臂碰撞模型。窗口显示的是 GraspNet 原始候选，不代表已通过工作空间、IK、MoveIt 碰撞或真实抓取验收。无有效候选时不绘制假夹爪，图像/候选过期后清空窗口，不能把旧帧叠加到新场景。

配置在 `config/graspnet_viewer.yaml`，可调整 `top_n`、`max_points`、`point_size`、深度范围、同步队列和年龄限制。默认只显示采集后 1.5 s 内的同帧结果。实拍验证和截图路径见[Open3D 迁移记录](acceptance_records/2026-09-11-open3d-viewer.md)。

软件回归覆盖：同帧同步/错帧拒绝、真实内参要求、设备 depth scale、图像 stride/endian、mask 传输、整场景推理/投影筛选、候选源时间、超时/断流清空、采集时刻 TF、无硬件启动组合，以及合成三节点真实 ROS 通信。实际验收仍需相机对齐与标定、已编译扩展的 GPU 推理、延迟分布、MoveIt 可达性和真实抓取统计。

SDK 实现依据使用仓库锁定版本的 [Orbbec 同步与 C2D/D2C 示例](https://github.com/orbbec/pyorbbecsdk/blob/727ba3b269fba8161fe045839af972a37c52242c/examples/sync_align.py)及 [StreamProfile 内参接口](https://github.com/orbbec/pyorbbecsdk/blob/727ba3b269fba8161fe045839af972a37c52242c/src/pyorbbecsdk/stream_profile.cpp)。
