# reBotArm MuJoCo 仿真底座

实机状态驱动 MuJoCo 的只读 Real2Sim Bridge、mirror/physics 模式和 Viewer 使用方法见
[`../../docs/real2sim_bridge_zh.md`](../../docs/real2sim_bridge_zh.md)。

Pick 精细抓取环境、成功/失败判定和批量验收见
[`../../docs/mujoco_pick_zh.md`](../../docs/mujoco_pick_zh.md)。

Sim2Real/Real2Sim 的仿真侧随机化、JSONL 记录、确定性回放、轨迹比较和批量安全
检查见 [`../../docs/sim2real_workflow_zh.md`](../../docs/sim2real_workflow_zh.md)。该流程
不连接实机。

本目录提供可独立使用的 MuJoCo 物理仿真核心、桌面 Viewer 和 ROS 2
适配层。已验证的目标环境是 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。
本阶段尚未开始强化学习训练；已提供轻量 Gym 风格 Reach API 和 headless 批量
rollout 入口，供后续云端训练/本地推理验证复用。当前环境不依赖 Gymnasium，
奖励函数只覆盖 Reach 验证任务，不代表 Pick/精细抓取训练已经完成。

## 安装与构建

在 Ubuntu VM 的工作区根目录执行。先 source ROS，确保虚拟环境能看到 Jazzy
的 `rclpy`；`--system-site-packages` 是必需的。colcon 当前需要兼容版本的
setuptools，因此限定为 `setuptools>=68,<80`。

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
python3 -m venv .venv-mujoco-ros --system-site-packages
source .venv-mujoco-ros/bin/activate
python -m pip install --upgrade pip 'setuptools>=68,<80'
python -m pip install -r src/rebotarm_simulation/requirements-mujoco.txt
.venv-mujoco-ros/bin/python -m colcon build --symlink-install --packages-select rebotarm_simulation
source install/setup.bash
```

每个新终端都按 `source /opt/ros/jazzy/setup.bash`、激活 venv、再
`source install/setup.bash` 的顺序初始化。

从源码调试时应显式使用 `.venv-mujoco-ros/bin/python -m ...`；colcon 安装后
的 `rebotarm_mujoco_*` console script 会在构建时写入解释器 shebang。因此若
更换或重建 venv，必须重新 colcon build，不能假定旧脚本会自动改用当前 Python。
以下是从工作区源码直接运行的完整等价命令，不依赖已安装的 console script：

```bash
MUJOCO_GL=egl PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_health --renderer-timeout 30
PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_cli --headless --duration 5
PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_viewer --duration 30
```

## 健康检查与无界面运行

EGL 渲染检查在隔离子进程中运行，并有超时保护，避免驱动挂起拖死主进程：

```bash
MUJOCO_GL=egl rebotarm_mujoco_health --renderer-timeout 30
```

预期 JSON 关键字段如下；版本和路径允许不同，但关节/执行器均应为 8，
`ok`、模型加载和有限步进应为 true。无 EGL 设备时物理健康仍可为 true，
同时 `renderer_available` 为 false 并给出 `renderer_error`。

```json
{"ok": true, "model_loaded": true, "physics_step_finite": true,
 "joint_count": 8, "expected_joint_count": 8,
 "actuator_count": 8, "expected_actuator_count": 8,
 "headless": true, "renderer_available": true}
```

不依赖窗口的定时运行：

```bash
rebotarm_mujoco_cli --headless --duration 5
```

输出同时含 `"requested_duration"` 和 `"achieved_duration"`；后者会按物理
时间步向上取整。交互 CLI 还支持 `state`、`joint`、`joints`、`jog`、
`gripper`、`step`、`contacts`、`reset`、`pause`、`resume` 和 `quit`。

无界面 Reach 随机策略批量 rollout：

```bash
rebotarm_mujoco_batch --episodes 3 --steps 100 --seed 0
```

该命令输出 JSON 摘要，包含每回合步数、累计奖励、末端到目标距离和成功标志；
它只验证 API/物理层可无界面运行，不代表已经完成强化学习训练。

夹爪接触稳定性检查：

```bash
rebotarm_mujoco_contact_check
```

该命令在无界面 MuJoCo 中移动到固定接触验收姿态，打开夹爪、放置 `test_cube`、
闭合夹爪并输出 JSON。`ok=true` 表示仿真中出现持续的手指-方块接触，方块位姿
保持有限，单步跳变、接触力和最低高度都在阈值内；它用于排查抖动、穿模或接触力
爆炸，不连接实机。

基础总验收入口会串行运行 health、headless Reach batch 和接触检查：

```bash
rebotarm_mujoco_acceptance --skip-renderer
```

在已经 source ROS 2 与 `install/setup.bash` 的 Ubuntu VM 中，可追加 ROS 2 接口验收：

```bash
rebotarm_mujoco_acceptance --skip-renderer --include-ros --timeout 30
```

MoveIt 联动验收需要先按下文启动 MuJoCo 后端和 MoveIt，再单独运行
`rebotarm_mujoco_moveit_acceptance --timeout 30`，或对总验收追加 `--include-moveit`。

## 桌面 Viewer

在 Ubuntu 图形桌面的终端运行，而不是普通无显示 SSH 会话：

```bash
rebotarm_mujoco_viewer --duration 30
```

按 `1`–`6` 选择关节，按住 `J/K` 连续正/反向移动当前关节，按住 `C/O`
连续闭合/打开夹爪，`G` 进入重力补偿，`H` 保持当前位置，`P` 进入 POS_VEL
位置目标控制，空格暂停，`.` 单步，`R` 回零位，`T` 回 home 姿态，`Q` 退出。
终端状态栏会显示当前模式、选中关节、实际关节角、关节速度、目标关节角、
夹爪实际/目标宽度、当前接触数量和最大接触力。MuJoCo 右侧
`control` 面板显示的是底层 actuator 的 torque/force，不是关节位置；日常控制请用
键盘 jog 或 ROS/API 发送目标。若从 SSH 启动，需正确配置 X11 转发，且
`DISPLAY` 与 `XAUTHORITY` 必须指向当前桌面会话；否则请改用 headless CLI。

Viewer 运行时也可以直接在启动它的终端输入目标命令，命令会排队到仿真主线程执行，
不会从键盘回调线程直接改 MuJoCo 状态：

```text
joints 0.0 -0.8 -1.0 0.3 0.0 0.0
joint joint2 -0.6
jog joint3 -0.1
gripper 0.05
mode hold
mode pos_vel
home
reset
state
contacts
```

其中 `joints` 一次设置六个关节目标角，`joint` 设置单个关节目标角，`jog` 是以当前
实际关节角为基准加一个增量，`gripper` 设置夹爪目标开口宽度。若只想看 Viewer 而不
读取终端命令，可加 `--no-command-input`。

模型使用同一套原始 STL 作为 visual 与 collision。为避免两个完全重合的网格在
Viewer 中互相闪烁或遮挡，collision 网格保留参与碰撞，但显示为半透明调试层；
正常观察以 visual 网格为准。场景只 include `models/rebotarm/robot.xml`，没有再叠加
旧的手写机械臂模型。

## ROS 2 适配层

只启动 MuJoCo 后端：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

节点提供五个 reBotArm 接口并额外发布仿真时钟：

- Action：`/rebotarm/follow_joint_trajectory`
- Topic：`/rebotarm/joint_states`
- Service：`/rebotarm/gripper/set`
- Topic：`/rebotarm/gripper/state`
- Service：`/rebotarm/trajectory_stop`
- Topic：`/clock`

MuJoCo 物理步进定时器固定使用 steady clock，不依赖节点的 ROS 时钟；消费仿真状态的
MoveIt、RViz 和状态发布节点使用 `use_sim_time:=true` 跟随 `/clock`。这样 `/clock` 的
发布者不会反过来等待自己尚未发布的仿真时间。

另开已 source 的终端发送一个六关节目标：

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.1, -0.2, -0.2, 0.2, 0.0, 0.0], time_from_start: {sec: 3}}]}}" --feedback

ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: 0.05, max_effort: 10.0}"

ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

也可以在 Ubuntu VM 中直接跑一次无实机 ROS 2 接口验收。它在同一进程里启动
MuJoCo ROS 2 节点和探针节点，检查 `/rebotarm/joint_states`、`/clock`、
`/rebotarm/gripper/set` 和 `/rebotarm/follow_joint_trajectory`：

```bash
rebotarm_mujoco_ros_acceptance --timeout 15
```

输出 JSON 中 `ok=true` 才表示 topic、service 和 action 均通过；该命令不启动
`rebotarmcontroller`，也不使用 `use_hardware:=true`。

MoveIt 的控制器映射保持为 `/rebotarm/follow_joint_trajectory`。联调时所有
bringup 必须显式使用 `use_hardware:=false`，且只保留一个该 Action 的服务端。

### MuJoCo 与 MoveIt 的安全双终端接线

终端 1 先启动唯一的 MuJoCo 轨迹服务端：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

终端 2 启动 MoveIt 预览，并明确关闭硬件、fake joint state publisher 和
passive joint state publisher：

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py use_moveit_preview:=true use_hardware:=false use_moveit_fake_joint_states:=false start_passive_joint_state_publisher:=false use_sim_time:=true
```

如需本地 RViz，可在第二条命令末尾追加 `use_local_rviz:=true`。关闭 fake 和
passive 发布器是为了让 `/rebotarm/joint_states` 只有 MuJoCo 这一份权威物理状态，
避免多个 joint state publisher 竞争。MoveIt 控制器仍映射到
`/rebotarm/follow_joint_trajectory`，由终端 1 的 MuJoCo Action 服务端执行。

两端启动后，可在第三个终端运行 MoveIt-MuJoCo 联动验收：

```bash
rebotarm_mujoco_moveit_acceptance --timeout 30
```

该命令等待 `/plan_kinematic_path`、`/rebotarm/follow_joint_trajectory`、
`/rebotarm/joint_states` 和 `/clock`，向 MoveIt 请求一条六关节规划，再把 MoveIt
返回的 `JointTrajectory` 发给 MuJoCo action server 执行。`ok=true` 表示规划、
执行和最终关节误差均通过；它不启动 RViz、不连接实机，但验证的就是 RViz 规划执行
会走的同一条 MoveIt planning service 和 MuJoCo trajectory action 链路。

## Python API（面向后续训练/推理复用）

核心类不依赖 ROS 2：

```python
from rebotarm_simulation.mujoco_sim import RebotArmMujoco

with RebotArmMujoco() as sim:
    sim.reset(seed=7)
    scene = sim.randomize_scene(seed=7)
    sim.set_joint_position_targets([0.1, -0.2, -0.2, 0.2, 0.0, 0.0])
    sim.set_gripper_width(0.05)
    sim.set_object_pose("test_cube", [0.45, 0.0, 0.44], [0.0, 0.0, 0.0, 1.0])
    saved = sim.save_state()
    state = sim.step(10)
    sim.restore_state(saved)
    contacts = sim.get_contacts()
```

`get_state()` 返回关节位置/速度、执行器力、末端位姿、夹爪宽度、物体位姿
和仿真时间。公开的末端与物体四元数顺序都是 XYZW；MuJoCo 内部 WXYZ 已在
API 边界转换。`save_state()`/`restore_state()` 只允许同一模型实例，
`set_object_pose()` 只接受带 free joint 的物体，四元数也使用 XYZW。
`randomize_scene()` 会按给定 seed 随机 `test_cube` 位置和 Reach 目标点，
返回 `RandomizedScene(cube_pose, reach_target_position, seed)`，用于后续
Reach/Pick 场景随机化。

轻量 Gym 风格 Reach 环境不依赖 `gymnasium`：

```python
from rebotarm_simulation.mujoco_env import RebotArmReachEnv

with RebotArmReachEnv() as env:
    obs, info = env.reset(seed=7)
    obs, reward, terminated, truncated, info = env.step([0.0] * 7)
    obs, reward, done, info = env.step_done([0.0] * 7)
```

`obs` 包含 `joint_positions`、`joint_velocities`、`gripper_width`、
`ee_position`、`target_position`、`cube_pose` 和 `max_contact_force`。
`step(action)` 接受 6 维关节增量或 7 维关节+夹爪增量，内部裁剪到 `[-1, 1]`。
默认 `step()` 使用 Gymnasium 的 `terminated/truncated` 形状，同时在 `info` 里提供
`done`；旧 Gym 风格代码可直接调用 `step_done()` 获取 `(obs, reward, done, info)`。

## Sim2Real / Real2Sim 基础层

当前先实现不连接实机的基础设施：随机化、统一轨迹日志、仿真回放和误差比较。
默认 profile 不改变现有 MuJoCo 行为；`training_profile` 只在显式传入时启用。

```python
from pathlib import Path

from rebotarm_simulation.mujoco_env import RebotArmReachEnv
from rebotarm_simulation.sim2real import (
    RandomizationConfig,
    TrajectoryRecorder,
)

config = RandomizationConfig.from_yaml(
    Path("src/rebotarm_simulation/config/sim2real_randomization.yaml"),
    profile="training_profile",
)
sample = config.sample(seed=7)
recorder = TrajectoryRecorder(episode_id="reach-0007", source="sim")

with RebotArmReachEnv() as env:
    env.reset(seed=7, randomization=sample)
    for step_index in range(100):
        action = [0.0] * 7
        obs, reward, terminated, truncated, info = env.step(action)
        recorder.append(
            env.sample_from_last_step(
                action,
                episode_id="reach-0007",
                step_index=step_index,
            )
        )
        if terminated or truncated:
            break

recorder.to_jsonl("logs/reach-0007.jsonl")
```

相同 seed 会得到相同随机化 sample。`RandomizationSession` 会显式修改并在退出时恢复
MuJoCo 的质量、阻尼、摩擦和 arm torque scale。日志格式不依赖 ROS 2，未来真实机械臂
只需要把数据转换成同一个 `TrajectorySample(source="real")`。`replay_actions()` 和
`compare_trajectories()` 可用于仿真回放和误差报告；当前阶段不宣称已经完成真实系统辨识
或 sim-to-real 精度标定。

后续架构是云服务器运行无界面 MuJoCo 并行训练，本地 Ubuntu VM 做模型验证、
ROS 2 联调和策略推理。现在只提供可复用物理/API 底座和 Reach rollout 验证，
不宣称已经有可用策略或训练收敛结果。

模型在 `end_link` 下提供命名坐标系 `wrist_camera_mount`，作为后续手眼/末端 RGB-D
相机的稳定安装基准。当前阶段只定义安装位，不绑定具体相机型号、内参或渲染传感器。

## 双端同步规则

Windows 仓库是受版本控制的主副本，Ubuntu VM 是构建与运行环境。每次同步前
先备份 VM 上将被覆盖的明确文件；只传输本次文件清单，传后比较 SHA-256 哈希。
禁止使用带 `--delete` 的目录镜像，也不反向同步 `.venv-mujoco-ros`、
`build/`、`install/`、`log/` 或缓存。

## 排障

- **重复节点**：若 Action 提示重复服务端，先用 `ros2 node list` 检查重复节点。
  仅结束 launch 父进程可能留下 `rebotarm_mujoco_node` 子进程；先用
  `pgrep -af rebotarm_mujoco_node` 定向查找候选 PID，再用
  `ps -fp <confirmed-pid>` 核对完整命令。只在确认它是本次仿真的遗留进程后，执行
  `kill -TERM <confirmed-pid>`，再复查节点列表。禁止宽泛 kill，
  也禁止同时运行旧 RViz-only 控制器。
- **SSH 无窗口**：检查 `DISPLAY`、`XAUTHORITY` 和 X11 转发；物理验证优先用
  `--headless`。
- **EGL 超时/崩溃**：健康检查的 renderer 子进程会在 `--renderer-timeout`
  后报告失败；可先用 `--skip-renderer` 单独确认物理模型。
- **轨迹未成功**：检查配置中的 goal tolerance（位置、速度和时间），当前采用
  节点级容差，不接受每个 goal 的覆盖值。
- **规划或接触异常**：MoveIt self-collision 矩阵与 MuJoCo 接触过滤是不同层，
  分别检查 SRDF self-collision 与 MJCF 的 `contype/conaffinity`。

## 无硬件安全边界

本流程不得设置 `use_hardware:=true`，不得启动 `rebotarmcontroller`，不得打开
CAN、串口或真实夹爪，不得发送实机使能、回零或轨迹命令。发现任何硬件节点、
CAN 设备或串口已被占用时，停止联调并确认环境；MuJoCo 测试不需要连接实机。

## 从 URDF 生成本地 MJCF

机器人结构的唯一权威来源是
`src/rebotarm_moveit_config/config/rebotarm.urdf`，MuJoCo 日常运行读取本地文件
`src/rebotarm_simulation/models/rebotarm/robot.xml`。后者是自动生成并随仓库提交的
MJCF，不需要联网加载。当前支持范围固定为 `mujoco>=3.3,<4`。

URDF 或其引用的原始 STL 更新后，在仓库根目录重新生成：

```bash
rebotarm_urdf_to_mjcf --repo-root .
```

只检查仓库中的 MJCF 是否仍与 URDF 一致，不写文件：

```bash
rebotarm_urdf_to_mjcf --repo-root . --check
```

转换使用 MuJoCo 官方 URDF 解析器。原始 STL 同时保留为视觉网格和碰撞网格，不执行
VHACD。执行器、双指联动、传感器、末端 site 和接触过滤由确定性后处理补入。不要手工
修改 `robot.xml`；应修改 URDF 或生成器后重新生成。

Windows 与 Ubuntu VM 同步后，可分别检查受控文件哈希：

```powershell
Get-FileHash src/rebotarm_simulation/models/rebotarm/robot.xml -Algorithm SHA256
```

```bash
sha256sum src/rebotarm_simulation/models/rebotarm/robot.xml
```

生成、检查和运行均不连接实机，不探测 CAN、串口或机械臂控制器；ROS 2 联调继续明确
使用 `use_hardware:=false`。
