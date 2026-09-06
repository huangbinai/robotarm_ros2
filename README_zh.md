# reBotArm ROS 2

reBotArm B601 的 ROS 2 Jazzy 工作空间，覆盖真机控制、MoveIt 规划、示教回放、网页与键盘遥操作、视觉抓取、标定、MuJoCo 仿真和任务级语音控制。

> 当前真机运行目标是 VMware/Ubuntu 24.04。Windows 工作区用于代码维护和离线测试，不应直接占用 MotorBridge 串口。

## 快速导航

| 想做什么 | 文档 |
| --- | --- |
| 了解全部功能及数据流 | [功能说明书](docs/functional_specification_zh.md) |
| 安装、构建并首次启动 | [部署与运行手册](docs/deployment_guide_zh.md) |
| 查询参数、安全边界和单位 | [配置参数参考](docs/configuration_reference_zh.md) |
| 查询 Topic、Service、Action | [ROS 2 接口参考](docs/ros_api_reference_zh.md) |
| 判断功能是否可用于实机 | [功能状态与验收矩阵](docs/feature_status_zh.md) |
| 参与开发、测试或添加模块 | [开发维护指南](docs/development_guide_zh.md) |
| 浏览全部文档 | [文档中心](docs/README.md) |

## 核心运行参数

| 项目 | 当前值 | 说明 |
| --- | ---: | --- |
| 底层控制循环 | `100 Hz` | 来自 `arm.yaml` 的 `rate` |
| 硬件反馈刷新 | `50 Hz` | 六轴和夹爪共用总线的反馈调度频率 |
| JointState 发布 | `100 Hz` | ROS 状态发布频率，可配置 |
| 反馈过期阈值 | `0.15 s` | 超时反馈不能继续作为有效状态使用 |
| 夹爪命令范围 | `0–0.085 m` | 越界命令直接拒绝，不做静默裁剪 |
| J1–J3 / J4–J6 effort | `27 / 7` | URDF、MoveIt 和 MuJoCo 应保持一致 |

J2、J3 的硬件反馈可接受上边界为 `+0.02 rad`，它只用于容忍零点附近的反馈偏差，不扩大规划或运动命令范围。

## 包结构

| 包 | 职责 |
| --- | --- |
| `rebotarm_msgs` | 自定义消息、服务和动作 |
| `rebotarmcontroller` | MotorBridge/SDK、真机生命周期、反馈、安全边界和 ROS 控制接口 |
| `rebotarm_motion` | 轨迹生成、重定时、碰撞预检和执行协调 |
| `rebotarm_teach` | 示教录制、轨迹准备和回放 |
| `rebotarm_teleop` | 键盘、网页和夹爪遥操作适配 |
| `rebotarm_dashboard` | 网页界面、状态聚合、HTTP/SSE 接口 |
| `rebotarm_moveit_config` | MoveIt 模型、规划组和约束配置 |
| `rebotarm_vision` | RGB-D、YOLO/GraspNet、候选筛选和视觉抓取执行 |
| `rebotarm_calibration` | 手眼标定、TCP 标定和几何工具 |
| `rebotarm_simulation` | MuJoCo、Real2Sim、任务环境及仿真 ROS 接口 |
| `rebotarm_voice_control` | 文本/语音意图、任务路由和 dry-run/sim/real 执行模式 |
| `rebotarm_bringup` | launch、硬件配置、URDF 和 RViz 资源 |
| `rebotarm_interactive_control` | 旧导入路径兼容层，不再承载新实现 |

独立的 Star Arm 102-LD → reBot B601 主从跟随工具位于 `star_arm_102_rebot_b601_follow/`，不接入主 ROS 启动链。

## 最小构建与启动

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

只启动真机控制器：

```bash
ros2 launch rebotarm_bringup driver_only.launch.py channel:=/dev/ttyACM0
```

启动后机械臂不会把“连接”视为“已使能”。检查状态后显式使能：

```bash
ros2 topic echo /rebotarm/arm_status --once
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

停止时优先执行受控回零和失能：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

## 安全提示

- 第一次运行新配置时先用仿真或 `plan_only`，再进入实机。
- 真机周围必须保留急停空间，首次运动使用小位移和低速。
- 不要让两个节点同时打开同一 MotorBridge 串口。
- 不要把反馈容差、规划关节限位和运动目标范围混为一套参数。
- 感知或规划失败不等于硬件故障；健康控制器应保持力矩或受控恢复，不能直接让机械臂掉电下落。

## 验证状态

当前 Windows 全量离线回归、Python 编译和差异格式检查均通过；其中 POSIX 虚拟串口、真实 ROS 2 运行时和显式性能基准会按平台跳过。离线结果不能替代 VMware/Ubuntu 下的 ROS 2 构建、虚拟串口和实机验收，详见[功能状态与验收矩阵](docs/feature_status_zh.md)。
