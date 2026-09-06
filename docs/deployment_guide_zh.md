# reBotArm 部署与运行手册

## 1. 支持环境

| 层级 | 建议环境 | 用途 |
| --- | --- | --- |
| 真机控制 | VMware/Ubuntu 24.04、ROS 2 Jazzy、系统 Python 3.12 | MotorBridge、控制器、MoveIt 和真机应用 |
| 视觉推理 | Ubuntu 本机独立 Python 环境，或兼容的网络服务 | Gemini 2、YOLO、TensorRT、GraspNet |
| 开发测试 | Windows 或 Ubuntu | 静态检查、纯 Python 单元测试 |
| 仿真 | Ubuntu 优先 | MuJoCo、RViz、MoveIt 联调 |

ROS 构建 Python 与视觉/GraspNet/MuJoCo 的运行解释器应分开。不要把 ROS 2 系统依赖全部装入一个 Conda 环境。

## 2. VMware 设备准备

1. 将 USB 串口设备连接到 VMware 中的 Ubuntu，而不是 Windows 主机。
2. 检查设备节点：

   ```bash
   ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
   ```

3. 将当前用户加入串口组，重新登录后生效：

   ```bash
   sudo usermod -a -G dialout "$USER"
   ```

4. 确认没有其他程序占用端口：

   ```bash
   lsof /dev/ttyACM0
   ```

同一时刻只能由一个控制器进程打开机械臂总线。主从跟随工具和 ROS 控制器不能同时连接相同端口。

## 3. 获取依赖

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r requirements-runtime.txt
mkdir -p third_party
vcs import third_party < rebotarm_dependencies.repos
```

如果底层 SDK 已按其他方式安装，可跳过重复导入，但必须确认当前 Python 实际加载的是预期版本：

```bash
python3 -c "import motorbridge; print(motorbridge.__file__)"
python3 -c "import reBotArm_control_py; print(reBotArm_control_py.__file__)"
```

导入后按审核补丁准备 SDK：

```bash
python3 tools/apply_rebotarm_control_safety_patch.py --apply
```

## 4. 构建

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

检查包：

```bash
ros2 pkg list | grep rebotarm
ros2 pkg executables rebotarmcontroller
```

## 5. 无硬件检查

在连接实机前完成：

```bash
python3 -m pytest -q
python3 -m compileall src -q
```

然后启动 MoveIt/RViz 或 MuJoCo 仿真验证模型方向、关节限位和规划：

```bash
ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

或：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

## 6. 首次真机启动

首次启动建议只运行控制器：

```bash
ros2 launch rebotarm_bringup driver_only.launch.py \
  channel:=/dev/ttyACM0 \
  cmd_arbitration:=reject
```

另开终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source ~/seeed/rebotarm_ros2/install/setup.bash
ros2 topic echo /rebotarm/arm_status --once
ros2 topic hz /rebotarm/joint_states
```

状态正常后显式使能：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

先发送很小的关节或位姿运动，不要直接执行完整视觉抓取或历史示教文件。结束时：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

如果回零失败但控制器仍健康，应保持当前位置，不要立即切断力矩。

## 7. 常用启动入口

| 场景 | 命令 |
| --- | --- |
| 仅真机控制器 | `ros2 launch rebotarm_bringup driver_only.launch.py` |
| 控制器、模型与可选 RViz | `ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true` |
| 真机 MoveIt | `ros2 launch rebotarm_bringup moveit_hardware.launch.py` |
| RViz 拖动仿真 | `ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py` |
| RViz 拖动真机 | `ros2 launch rebotarm_bringup rviz_ee_drag_real.launch.py` |
| 示教录制 | `ros2 launch rebotarm_bringup teach_record.launch.py` |
| 示教回放检查 | `ros2 launch rebotarm_bringup teach_replay.launch.py dry_run:=true` |
| 视觉只规划 | `ros2 launch rebotarm_bringup visual_grasp_system.launch.py execution_mode:=plan_only use_hardware:=false` |
| MuJoCo ROS 后端 | `ros2 launch rebotarm_simulation mujoco_sim.launch.py` |
| Real2Sim 只读桥 | `ros2 launch rebotarm_simulation real2sim_bridge.launch.py` |
| 语音 dry-run | `ros2 launch rebotarm_voice_control voice_control.launch.py` |

## 8. 视觉解释器

`vision.launch.py` 支持通过环境变量指定视觉 Python：

```bash
export REBOTARM_VISION_PYTHON=/path/to/vision-venv/bin/python
ros2 launch rebotarm_vision vision.launch.py \
  camera_config:=/path/to/camera.yaml \
  handeye_config:=/path/to/handeye.yaml \
  yolo_model_path:=/path/to/model.engine
```

模型路径默认留空，缺少本机模型时源码仍可构建；运行推理前必须显式提供可用模型。

## 9. 常见问题

### 串口不存在

确认 USB 已切换到虚拟机，并用 `channel:=实际设备` 覆盖默认 `/dev/ttyACM0`。

### 串口权限不足

检查 `groups` 是否包含 `dialout`。临时使用 `sudo` 启动整个 ROS 工作区会产生文件权限和环境不一致问题，不建议作为常规方案。

### 反馈过期

检查总线负载、MotorBridge 版本、反馈 ID、控制器是否重复启动，以及 `hardware_feedback_rate_hz` 是否被错误调高。不要通过无限增大 `feedback_stale_timeout_sec` 掩盖通信故障。

### RViz 有模型但不能执行

确认当前命名空间内存在唯一的 `FollowJointTrajectory` 服务端，并检查是仿真后端还是真机后端。仅启动 `robot_state_publisher` 不提供执行能力。

### 视觉没有候选

依次检查 RGB、深度、CameraInfo、检测结果、GraspNet 候选、TF、IK 和碰撞筛选。不要启用旧计划回退。

## 10. 当前验收边界

Windows 已完成离线测试，但 VMware/Ubuntu 仍需执行：

- `colcon build` 和包入口检查；
- MotorBridge 构建/导入和 POSIX 串口验证；
- 连接但不使能、显式使能、反馈新鲜度和安全失能验证；
- 小幅轨迹、夹爪校零、夹爪边界和取消操作验证；
- MoveIt、示教和视觉链路的分阶段实机验收。

本仓库不包含 P0 Gate B/C 验收工具。
