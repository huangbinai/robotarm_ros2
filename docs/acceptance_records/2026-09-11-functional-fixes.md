# 2026-09-11 功能问题修复与复测

本记录接续[第一轮功能测试](2026-09-11-functional-regression.md)。本次仍仅运行仿真机械臂和真实 Gemini 2 感知，没有连接或驱动真实机械臂。

## 修复内容

| 问题 | 最终处理 | 复测 |
| --- | --- | --- |
| ROS 入口调用系统 Python，找不到 mujoco/torch | `tools/build_workspace.bash` 由项目 venv 的 Python 运行 colcon；增加 `check_python_entrypoints.py`，构建后校验 shebang | 48 个 Python 入口全部正确；标准 MuJoCo launch 和直接 ros2 run GraspNet 正常启动，不再用 python -m 绕过 |
| 示教录制混入手指关节 | `select_record_joints` 按配置筛选/排序六轴；长度、缺轴、重名及数值校验 | 实际录制 164 样本，只有 joint1～joint6；网页计算起点误差并完成预检查 |
| 规划回调阻塞自己的服务响应 | MoveIt 客户端独立响应回调组；TeachReplay 多线程执行器；同步等待用单调时钟；停止与发送边界加锁 | 网页与独立节点的起点规划/碰撞预检均通过；实际 MuJoCo 回放 done，status=4/error_code=0 |
| 取消响应覆盖终态、停止被报成成功/泛化失败 | Web 会话按执行代次隔离，终态由 Action 决定；明确的服务停止显示 stopped；未确认取消不伪装完成；执行失败仍为 failed | 网页终态 stopped，之后的取消回复不覆盖；新旧目标/晚到回复/待接受时停止均有回归 |
| MuJoCo 目标完成后可能残留 busy | 使用实际准入 token 清理活动目标，不依赖 rclpy 请求包装对象身份相同 | 一轮中连续规划、网页、键盘、停止、回放可执行 |
| Open3D 无法配对实际推理帧 | GraspNet 在推理后可靠发布实际输入 RGB-D/CameraInfo；查看器订阅低频预览话题，ROS 接收与 GUI 渲染分线程 | 真相机 ROS 查看器收到同帧候选，成功保存实时 PNG/PLY |

已有旧八关节文件不自动覆盖。新增转换工具导出到新的六轴文件，并要求重新预检查。本次对第一轮失败的 162 样本记录验证：原文件仍保留八关节，新文件六轴，采样时间和六轴位置完全一致；这只是数据契约转换，不宣称旧轨迹已实机验收。

## 自动回归

```bash
source tools/source_ubuntu_env.bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
python -m pytest -q tests --junitxml=docs/acceptance_records/2026-09-11-functional-fixes-tests.xml
python3 tools/check_architecture.py --strict-deployment
```

结果 **107 passed in 7.60s**，严格架构检查通过。原始 JUnit：[functional-fixes-tests.xml](2026-09-11-functional-fixes-tests.xml)。新增回归包括六轴筛选、非法反馈、Future 响应可并行处理、暂停 ROS 时钟仍能超时、停止竞态与最终状态。

## 仿真端到端复测

可重复运行：`python tools/verify_sim_workflows.py`。工具使用空闲的独立 domain 184，只启动 MuJoCo、MoveIt、teach、dashboard；执行结束清理自己启动的进程。10 项检查全部为 true，退出 0：

| 项目 | 实测 |
| --- | --- |
| 标准入口与 MoveIt 规划/执行 | 37 个规划点，Action 成功，最终最大关节误差 0.011626 rad |
| 六轴录制 | 164 个样本，joint1～joint6 |
| 网页夹爪 | 0.05 m 命令成功，收到后端反馈 |
| 网页关节与键盘小步 | 轨迹成功；合法按键请求 accepted=true |
| 停止终态 | stopped；Action status=6/error_code=-1，后端明确说明 service stop |
| 停止位置 | 稳定后的 joint1 额外变化约 3.02×10⁻⁷ rad；距原轨迹目标仍有 0.08885 rad |
| 网页示教预检查 | accepted=true，MoveIt 起点规划成功，80 个碰撞检查样本全部通过 |
| 独立示教预检查/实际回放 | dry_run 通过，随后 MuJoCo 中回放 done，status=4/error_code=0 |

原始日志和完整 JSON：`.codex_tmp/functional-sim-lic2m0ln/`。控制器的停止与失能不同；本轮证明的是仿真停止，不是实际机械臂制动时间/位移。

随后将 MoveIt 验收探针改为单线程 polling executor，避免退出后残留线程回调访问已销毁节点。完整仿真流程再次运行，10 项检查仍全部通过，未再出现探针 Destroyable 警告；最后结果在 `.codex_tmp/functional-sim-mczb994_/`。该轮规划最终误差 0.008497 rad，实际示教回放成功。

## 真实相机与实时 Open3D

可重复运行：`python tools/verify_live_vision.py`，使用独立 domain 185。测试仅连接相机和推理/查看器，无运动客户端。

本次收到 17 条候选消息，其中 8 条非空；最大候选数量 2（上限仍为 5），非空消息源年龄约 0.730～0.878 s。色深注册输出 1280×800，CameraInfo 正常。ROS 查看器成功保存匹配场景，`open3d_saved=true`；这是实际 ROS 同步后的结果，不是直接调用绘图函数的替代验证。

日志和结果：`.codex_tmp/functional-vision-edgwzwwn/`。
截图：`.codex_tmp/functional-vision-edgwzwwn/live-ros-open3d.png`，对应 PLY 同目录。相机看到 banana/cake 类别，图中候选是原始 GraspNet 输出，不代表可执行或已完成夹取。

## 限制

本次修复关闭第一轮的启动、六轴契约、规划阻塞、停止终态和 Open3D live 配对问题；探针清理警告也在最后一轮对照复测中消除。测试进程已结束，真实机械臂未启动。后续实机反馈、失能、物理停止、手眼精度和任务成功率仍按验收矩阵单独进行，不能由软件/仿真通过推断全部关闭或故障恢复均已验收。

当前工作区仍有未提交修改；历史记录不回写为本次通过。构建完成后以 `source tools/source_ubuntu_env.bash` 运行，无需依赖临时验收安装目录。

最终构建：13 个包成功（7.44 s），构建后 48 个 Python 入口 shebang 全部符合项目解释器；依赖检查无冲突。
