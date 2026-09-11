# 2026-09-11 本机视觉环境部署与真实 GPU 模型验收

结论：本机 Python/CUDA 视觉运行环境和 GraspNet 资产已经安装。真实 YOLO26/GraspNet 权重在 RTX 4060 上完成离线 RGB-D 推理，经 ROS 话题输出候选并验证断流清空。相机 SDK 枚举到 **0 台 Orbbec 设备**；Gemini 2 实采、现场标定和机械臂抓取仍未验收。

## 安装位置与版本

所有 Python 包安装在项目 `.venv-ros`，编译器放在 `third_party`。没有修改系统 NVIDIA 驱动或安装系统级 CUDA。当前驱动仍为 535.288.01，GPU 为 NVIDIA GeForce RTX 4060 Laptop GPU（8 GB）。

| 组件 | 版本/位置 | 验证 |
| --- | --- | --- |
| Python | 3.12.3，`.venv-ros` | ROS Jazzy 和视觉模块均可导入 |
| PyTorch / torchvision | 2.5.1+cu121 / 0.20.1+cu121 | CUDA 可用，真实模型和矩阵运算通过 |
| CUDA 编译组件 | 12.1.105，`third_party/cuda-12.1` | `nvcc --version`；扩展编译成功 |
| GCC / G++ | 12.4.0，`third_party/gcc-12` | 从 Ubuntu deb 解包到项目，不替换系统 GCC 13 |
| NumPy | 1.26.4 | 保持 ROS 二进制模块兼容 |
| Ultralytics | 8.4.147 | YOLO26 分割权重真实 GPU 推理通过 |
| Open3D | 0.19.0 | GraspNet 点云碰撞检查可用 |
| Orbbec SDK | 官方 `pyorbbecsdk2==2.0.18` Python 3.12 wheel | Python 模块/API 可用，设备枚举数 0 |
| PointNet2 / KNN | 本机编译的 `pointnet2._ext` / `knn_pytorch.knn_pytorch` | CUDA 实际算子和模型通过，目标 sm_89 |
| GraspNetAPI / Grasp NMS | 1.2.11（本地补丁）/ 1.0.2（本机编译） | 导入、NMS 和推理通过 |
| Cython / pybind11 / ninja | 3.0.12 / 2.13.6 / 1.13.2 | 扩展构建依赖 |

`opencv-python==4.11.0.86` 作为 pip 依赖安装，但当前 ROS 环境优先加载系统 `cv2 4.6.0`。已修正 ArUco 兼容路径，使用对应版本的检测 API 和通用 `solvePnP`；实际 OpenCV 的合成 marker 几何/错误 marker 测试通过。

直接版本约束保存在 [constraints-ubuntu.txt](../../src/rebotarm_vision/constraints-ubuntu.txt)，环境加载/扩展重建命令见[本机视觉路线](../local_ros_vision_zh.md)。此表是本次环境记录，不代表所有传递依赖已完全锁定。

## 源码、权重和下载身份

- GraspNet baseline：`280c215129f759ed8649cb4e89fc5dfee55f4f80`。GitHub Git 连接失败后，使用该提交的 tree API 和 raw blobs 获取全部 57 个文件，逐文件校验 Git blob SHA-1，再应用项目补丁。
- GraspNetAPI：`bd6783c3effdebd895abfba8b96dc22a42ec3b5a`，从该提交的 codeload 归档提取。
- 已应用 `patches/graspnet-baseline/0001-use-fixed-width-knn-indices.patch`，以及 GraspNetAPI 的 0001、0002 补丁。0002 将离线评估依赖移为可选 extras，并兼容 Python 3.12 / NumPy 1.26；未安装完整离线评估栈。
- SDK 使用与锁定源码版本号对应的官方 2.0.18 wheel，本次没有从源码提交编译 SDK，不把两者视作字节级相同产物。
- `third_party` 中源码目录保存 `.upstream-revision`；下载缓存保存在 `.codex_tmp/vision-downloads`，不作为项目源码提交。

| 资产 | SHA-256 |
| --- | --- |
| GraspNet checkpoint-rs.tar | `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868` |
| YOLO26s-seg.pt | `3da1d83e31caec96f9300eb4064f4f62882c133c7c264d63dfe61a7c197837a4` |
| Orbbec 2.0.18 cp312 Linux wheel | `3e90d2ff98948bce682374d3597da6aa293ba0cf7107d193a8515333279ffc2d` |
| GraspNetAPI 固定提交归档 | `985171a433fae42e293f87339d3ec39817662af66806034b81b5e5f611cf5e96` |
| Grasp NMS 1.0.2 源码归档 | `30b8709483e69b9f6ad12e5e3701e5fe77882dc4446f3ea6b89dece822f93b8e` |

CUDA 来源为 [NVIDIA 12.1.1 redistributable manifest](https://developer.download.nvidia.com/compute/cuda/redist/redistrib_12.1.1.json)，下列压缩包解到 `third_party/cuda-12.1`，`--strip-components=1`；各包哈希已与官方 manifest 核对：

| 组件 | manifest 中的版本 | SHA-256 |
| --- | --- | --- |
| cuda_nvcc | 12.1.105 | `0b85f7eee17788abbd170b0b493c74ce2e9fd5a9604461b99c2c378165e1083b` |
| cuda_cudart | 12.1.105 | `6096ec878c8c443258d39c6e9cf2decef127f8aa8da594fdc5a336d047ab6bd9` |
| cuda_cccl | 12.1.109 | `b84ef3ec3dc1b4891267be25846f0c3ed7f9fa84154d59eba805402b86991baa` |

GCC 工具链由 `apt-get download gcc-12 g++-12 cpp-12 gcc-12-base libgcc-12-dev libstdc++-12-dev` 获取，版本 `12.4.0-2ubuntu1~24.04.1`，以 `dpkg-deb -x` 解入项目。完整 CUDA SDK 的其他库/头文件由 PyTorch 的 NVIDIA wheel 提供；`source_graspnet_build_env.bash` 只在编译时添加它们。

## 真实模型推理

执行：

```bash
source tools/source_ubuntu_env.bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python tools/verify_perception_models.py
```

输入为固定 GraspNet 提交中的 `doc/example_data` 录制 RGB-D、meta 内参，使用真实 YOLO26 和 GraspNet checkpoint。没有相机或机械臂连接。

| 轮次 | YOLO 检测数 / 目标 | 候选数 | YOLO 时间 | GraspNet 时间 |
| --- | --- | --- | --- | --- |
| 首轮 | 2 / banana | 3 | 2.075 s | 0.796 s |
| 预热后第 1 轮 | 2 / banana | 0 | 0.0106 s | 0.629 s |
| 预热后第 2 轮 | 2 / banana | 1 | 0.0094 s | 0.569 s |

结果 `ok=true`，数值有限，退出 0。候选数随随机点云采样和筛选而变化；空候选是合法的拒绝结果。本试验只验证实际模型执行，不是抓取成功率或持续负载性能。

原生算子另做 GPU/CPU 比对：KNN 在 CPU 与 CUDA 上均找到预期最近点（1-based index=1）；PointNet2 的 CUDA farthest-point sampling 返回预期 `[0,3]`。`python -m pip check` 输出 `No broken requirements found`。

## 真实模型 ROS 通信

```bash
source tools/source_ubuntu_env.bash
python tools/verify_perception_ros.py --domain 182 --timeout 45
```

工具在空闲的本机 domain 中启动独立 YOLO/GraspNet 进程，以录制样例发布 ROS Image/CameraInfo；没有执行器或运动客户端。

```json
{
  "ok": true,
  "domain": 182,
  "source": "upstream recorded RGB-D; real YOLO and GraspNet GPU models",
  "nonempty_candidate_messages": 2,
  "candidate_counts": [1, 1],
  "source_stamps_preserved": true,
  "candidates_cleared_after_input_stop": true,
  "camera_connected": false,
  "robot_connected": false
}
```

退出 0。节点原始日志位于 `.codex_tmp/perception-ros-ml6yg_ma/`，由工具自动创建；预热及停流期间出现源帧过期拒绝记录，符合 1.5 s 新鲜度边界。测试进程已关闭。

## 相机与剩余限制

`python tools/check_local_perception.py` 通过：依赖、原生扩展、权重哈希、CUDA 均可用。

`python tools/check_local_perception.py --probe-camera` 返回 1，唯一错误为未发现 Orbbec 相机，`camera_count=0`。SDK 的 Frame 时间戳、stream profile、C2D/内参相关 API 存在，但尚未用真实 Gemini 2 帧验证。系统 USB 列表同样没有 Orbbec 设备。

下一步需要将 Gemini 2 接到 Ubuntu 主机，再验证完整色深帧、C2D/去畸变、实际 CameraInfo、TF/手眼/TCP 与真实场景的模型延迟。当前没有授权或执行机械臂运动，本次也没有实机抓取结论。

最终软件回归为 **75 passed in 6.20s**，原始结果保存为 [perception-runtime-tests.xml](2026-09-11-perception-runtime-tests.xml)。严格架构检查仍为 `13 packages, 295 Python files; boundaries=PASS; deployment=PASS`。视觉包日常重建成功，30 篇当前 Markdown 的本地链接检查通过。

本次仍是 dirty 工作区，基线 HEAD `0dd373ad05a13b944cfb2ba368206c1badf5950a`，发布前须提交并固定最终版本；不能用单独 HEAD 复现所有未提交修改。
