# MuJoCo 已知瓶位接触试验

本试验只使用规范 MuJoCo 桌面瓶子场景。它读取仿真瓶位，生成预抓取与接近
目标，经 MoveIt IK、状态有效性与规划服务检查，再把两段轨迹交给唯一的
MuJoCo `FollowJointTrajectory` 服务端。随后闭合仿真夹爪，观察左右手指与
瓶子的同帧接触、接触力、穿透深度和瓶子位移。

终端 1（有图形桌面，能直接看到机械臂、夹爪和瓶子运动）：

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export REBOTARM_MUJOCO_PYTHON="$PWD/third_party/rebotarm_mujoco_venv/bin/python"
ros2 launch rebotarm_simulation mujoco_rviz_viewer.launch.py
```

如在无显示器的 SSH/服务器上运行，将最后一行换成
`ros2 launch rebotarm_simulation mujoco_headless.launch.py`。这个入口只输出
终端日志，不会打开 MuJoCo 或 RViz 窗口。

终端 2，等待终端 1 出现 `You can start planning now!` 后运行：

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service list | grep /rebotarm/sim/grasp_state
ros2 run rebotarm_motion rebotarm_mujoco_bottle_trial --timeout 30
```

先等终端 1 出现 `You can start planning now!`，并确认 `grep` 输出服务名。
如果没有输出，先检查终端 1 的 MuJoCo 节点是否仍在运行，以及两个终端的
`ROS_DOMAIN_ID` 是否一致；单独执行 trial 命令不会启动仿真服务。

两个终端需使用同一 `ROS_DOMAIN_ID`；单套系统无需专门设置。只运行这一套
MuJoCo launch，不同时启动真机控制器或另一个仿真 Action 服务端。一次试验后
瓶子可能位移；需要从规范初始位重复测试时，退出终端 1 并重新启动 launch。

报告里的 `ok=true` 表示试验流程完成；`arm_phases` 中的两段动作需各自为
`action_status=4`、`action_error_code=0`。`bilateral_finger_contact=true`
要求左右手指在同一观测帧接触瓶子。`grasp_result` 区分
`no_bilateral_contact`、`bilateral_contact_without_lift` 和
`contact_and_lift`。最后一种要求瓶子相对初始高度至少上升 20 mm；本试验
目前不安排抬升轨迹，因此通常只会达到接触阶段。接触力和穿透均为仿真指标，
不代表真实夹持力或真机抓取成功。

目标位姿只覆盖规范瓶位附近的固定姿态。超出已验证工作台范围时程序会在
动作前拒绝；规划、IK 或 Action 失败时报告失败阶段并请求仿真轨迹停止。
