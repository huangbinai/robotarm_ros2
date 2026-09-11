# 2026-09-11 Ubuntu 本机 ROS 视觉路线迁移

结论：正式感知链已从 Windows HTTP/JSON 改为 Ubuntu 本机相机、YOLO、GraspNet 三节点 ROS 通信；代码、启动、安装资源、专项及既有回归通过。实际相机/GPU 推理环境缺少依赖与 GraspNet 资产，因此没有把软件迁移结果记为真实视觉抓取验收。

## 变更范围

| 边界 | 最终行为 |
| --- | --- |
| 相机采集 | 本机 Gemini 2；完整帧集、设备时间偏差/重复检查、depth scale、彩色对齐到深度、基于设备内参去畸变；缺帧/缺内参拒绝 |
| YOLO | 独立订阅 ROS 图像；发布原时间戳的检测与 segmentation polygon |
| GraspNet | 四路精确时间同步；包内推理封装使用 RGB-D API；整场景推理后用 YOLO mask/bbox 投影筛选 |
| 时间与坐标 | 图像/内参/检测/候选保留源时间和深度光学 frame；推理前后检查年龄；过滤器使用采集时刻 TF，计划保留源时间；执行前检查源计划年龄 |
| 失效行为 | 空检测、过期、格式错误、推理异常或断流产生空候选；不继续执行旧缓存 |
| 启动默认值 | camera.type=gemini2、source_mode=ros；三个感知节点统一解释器配置；正式 config/launch 无 HTTP 地址/轮询参数 |
| ArUco/TCP | 订阅相同本机 ROS 图像/CameraInfo，使用采集时刻 TF；等待终端输入时继续接收数据 |

旧 Windows 工具没有删除，只作为历史工具保留；正式节点不加载其网络客户端。

## 验证环境

- 2026-09-11，Asia/Shanghai；工作区 `/home/huangbin/robotarm_ros2`。
- Git HEAD 为 `0dd373ad05a13b944cfb2ba368206c1badf5950a`，工作区 dirty，包含之前架构迁移与本次修改；单独 HEAD 不能复现当前结果。
- Python 3.12.3、ROS 2 Jazzy、message_filters 4.11.17、NumPy 1.26.4。
- 本机 NVIDIA RTX 4060 Laptop GPU，驱动 535.288.01；这仅说明硬件/驱动可被识别。
- 未连接相机或机械臂；ROS 通信测试使用合成相机及推理替身，运行于独立 domain 181、本机发现范围。

## 自动测试

```bash
source tools/source_ubuntu_env.bash
ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python -m pytest -q tests \
  --junitxml=docs/acceptance_records/2026-09-11-local-ros-perception-tests.xml
python3 tools/check_architecture.py --strict-deployment
```

结果：**73 passed in 5.89s**，pytest 退出 0；其中本次视觉专项 25 项，原有架构/执行回归 48 项。JUnit 原始结果见 [local-ros-perception-tests.xml](2026-09-11-local-ros-perception-tests.xml)。

专项包含：同帧匹配、错帧/frame/尺寸拒绝、缺失内参、未去畸变输入、过期/未来时间、stride/endian、设备 depth scale、色深缺帧不复用缓存、设备内参去畸变、mask 传输、全场景点云/投影过滤、封装 RGB-D API、真实 ROS 三节点合成链与断流、延迟输出拒绝、空/无效输入清空、采集时刻 TF、计划 header 隔离、ArUco ROS 输入，以及无真机的感知/仿真启动组合。

严格检查结果：`13 packages, 295 Python files`，`boundaries=PASS; deployment=PASS`，退出 0。增加的模块属于现有 vision 包，没有新增包或引入硬件 SDK 越界依赖。

## 构建和安装

本次改变的 `rebotarm_vision` 和 `rebotarm_bringup` 日常增量构建成功。另在新目录 `.codex_tmp/local-perception-build.iydzrs/` 对这两个包执行普通复制安装，结果 `2 packages finished [2.09s]`，退出 0。

```bash
colcon --log-base .codex_tmp/local-perception-build.iydzrs/log build \
  --base-paths src --packages-select rebotarm_vision rebotarm_bringup \
  --build-base .codex_tmp/local-perception-build.iydzrs/build \
  --install-base .codex_tmp/local-perception-build.iydzrs/install
```

其他 ROS 包使用当前 `install-venv` 作为 underlay，此轮不宣称全新 OS/全部依赖安装已验证。新资源包含 YOLO 包内回退权重、requirements-local.txt、三个感知节点及包内 GraspNet 封装。

## 尚未通过的真实部署检查

`python tools/check_local_perception.py` 退出 **1**，明确列出：

- Python 缺少 `pyorbbecsdk`、`ultralytics`、`torch`、`open3d`、`graspnetAPI`。
- `third_party/graspnet-baseline` 与 `models/graspnet/checkpoint-rs.tar` 不存在。
- 本地 YOLO26 权重存在，SHA-256 与 manifest 相符：`3da1d83e31caec96f9300eb4064f4f62882c133c7c264d63dfe61a7c197837a4`。

实际 GPU 模型初始化/推理、GraspNet 扩展编译、相机同步/注册质量、现场内参与手眼/TCP、真实端到端延迟、MoveIt/实机抓取仍为 NOT_RUN。合成 ROS 通信成功不能替代这些项目。

运行配置、路径与下一步部署方法统一见[本机 ROS 视觉路线](../local_ros_vision_zh.md)。
