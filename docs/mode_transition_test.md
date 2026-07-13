# reBotArm 电机模式平滑切换测试手册

## 当前实现

- 正式控制路径只允许 `POS_VEL ↔ MIT`。
- 进入重力补偿时，对重力力矩、`KP`、`KD` 使用 smoothstep 斜坡。
- 退出重力补偿时，依次执行阻尼减速、MIT 保持混合、POS_VEL 稳定保持。
- VEL 默认关闭；底层 SDK 接口仍保留，但 ROS 正式功能不使用。
- 转换期间拒绝新的轨迹、位置控制和单关节低层命令。
- 反馈无效、转换失败或超时时回退到当前位置保持；无法安全保持时立即失能。
- 停机和 disable 会先尝试平滑退出，但无论退出是否成功，最终都会执行电机失能。

参数位于 `src/rebotarm_bringup/config/mode_transition.yaml`。首次实机测试不要直接修改默认增益；先记录现象，再一次只调整一个参数。

## 非实机验证

以下命令不会连接或使能机械臂：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select \
  rebotarm_msgs rebotarmcontroller rebotarm_bringup

source install/setup.bash
python3 -m pytest \
  tests/test_mode_transition_policy.py \
  tests/test_mode_transition.py \
  tests/test_gravity_compensation_core.py -q
```

预期：构建成功，测试零失败，不会出现串口连接或电机使能日志。

## 实机单次空载测试

> 风险：以下命令会使能真实机械臂。移除末端负载，清空工作空间，保持急停可触达并托住机械臂，再执行。

终端 1：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup driver_only.launch.py
```

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotarm_ros2/install/setup.bash

ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger
```

进入后轻扶机械臂确认重力补偿稳定，再执行：

```bash
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

预期状态顺序：

```text
IDLE
→ ENTERING_GRAVITY_COMP
→ GRAVITY_COMP
→ EXIT_DAMPING
→ EXIT_BLENDING
→ POS_VEL_SETTLING
→ POS_VEL_HOLD / IDLE
```

合格标准：无明显咔嗒声、高频抖动或下沉；位置跳变小于 `0.02 rad`；锁定前速度小于 `0.05 rad/s`；单次转换在 `2 s` 内完成；无电机错误。

## 异常处理

- 出现持续振荡、突然运动或电机报错：立即急停或调用 `/rebotarm/disable`，不要继续重复测试。
- 返回 `feedback velocity ... exceeds ...`：机械臂仍在运动，稳定后再试。
- 返回 `position changed beyond transition limit`：位置变化超过安全阈值，先检查外力、负载和反馈。
- 返回 `mode transition in progress`：前一次转换尚未结束，不要并发发送命令。
- `set_mode vel` 或低层 `mode=2` 返回 `VEL mode is disabled` 是默认预期行为。

