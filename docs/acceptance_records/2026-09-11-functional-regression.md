# 2026-09-11 原有功能第一轮测试

范围：无真实机械臂执行。仿真运动使用 MuJoCo，真实视觉使用 Gemini 2。所有本轮启动的进程已结束，没有使能、回零或驱动真实机械臂。测试期间用户暂停询问后以“恢复”继续。

## 结果

| 功能 | 结果 | 证据与限制 |
| --- | --- | --- |
| 当前回归测试 | PASS | 86 passed in 6.92s；不是已删除历史测试套件的通过数量 |
| 严格架构 / Python 依赖 | PASS | boundaries=PASS; deployment=PASS；pip check 无依赖冲突 |
| MuJoCo 核心与渲染 | PASS | 模型 8 关节/8 执行器、有限物理步、64×64 RGB Renderer、接触子项通过 |
| MuJoCo ROS 接口 | PASS | 夹爪服务、轨迹 Action、时钟、消息结构通过；最终误差 0.00619 rad |
| Reach/Pick | 仅环境冒烟通过 | 本轮任务成功数仍为 0，不作为任务能力证明 |
| Real2Sim mirror | PASS（合成） | 200 样本，最大关节误差 0 rad |
| 标准 MuJoCo launch | FAIL | ros2 入口 shebang 为 /usr/bin/python3，找不到 venv 的 mujoco；用项目 python -m 启动后才继续测试 |
| MoveIt → MuJoCo | PASS（绕过上述启动问题） | 两次规划/执行成功，各生成 37 轨迹点；最终误差分别 0.01163、0.01026 rad |
| 示教录制服务 | 局部 PASS | start/stop 成功，分别录制 164、162 样本；文件带 6 轴和 2 个夹爪关节，后续兼容性有问题 |
| Dashboard 页面/配置/URDF | PASS（HTTP/API） | 页面和 URDF 返回 200；六轴限位配置与模型一致；未进行浏览器逐按钮视觉验收 |
| 网页夹爪 | PASS（仿真） | 0.05 m 请求成功，反馈约 0.05048 m；操作状态 done |
| 网页关节执行 | PASS（仿真） | joint1 小步 0.02 rad、2 s，Action 最终 status=4/error_code=0，网页 done |
| 网页非法目标拒绝 | PASS | joint1=100 rad 返回 HTTP 400，accepted=false，明确 joint limit 拒绝 |
| 网页键盘 | 部分检查 | enable/disable 成功；测试请求漏带 KEYBOARD_TELEOP 确认被正确拒绝，合法按键动作尚未验收；不能归为产品故障 |
| 网页停止 | 待核查 | 发出 8 s 轨迹后约 0.5 s 请求停止，返回 cancel_requested；随后显示 done / already finished before cancel；缺少终止位移及可靠最终状态证据，不计 PASS |
| 网页示教 dry-run | BLOCKED | start_band=unknown / current start error unavailable，accepted=false |
| 独立示教 dry-run | BLOCKED | 起点需对齐；一次 planning request timed out，另一次 service unavailable；未进入可执行回放 |
| 相机 SDK/资产 | PASS | 1 台 Gemini 2，CUDA、扩展与权重哈希可用 |
| 直接 ros2 run GraspNet | FAIL | 入口使用系统 Python，缺少 torch；项目 python -m 路径可用；正式带 Python prefix 的组合入口不等于此直接入口 |
| 真实相机 → YOLO → GraspNet | PASS（候选链） | 调整测试调用解释器后，22 s 窗口收到 13 条候选消息，4 条非空，最大 5 个；非空候选年龄约 0.803–0.945 s |
| Open3D live 同步 | 未通过本轮证据门槛 | 查看器可创建，未形成可保存的匹配候选画面；之前直接渲染截图通过不等于此 live 链通过 |

## 已定位或待诊断的问题

1. ROS 安装入口的 Python 与依赖环境不一致。已核实 MuJoCo 和 GraspNet 入口首行是 `/usr/bin/python3`，对应错误分别为缺失 `mujoco`、`torch`。本轮只用测试脚本中的项目解释器绕过，没有修改产品启动配置或重装依赖。
2. `TeachRecorderNode._write_sample` 按输入消息全部关节写入；MuJoCo 发布 8 个关节，记录也包含手指。Dashboard `_teach_record_info` 只提供六轴当前状态，故无法计算这个文件的完整起点误差。需要统一录制关节集合与回放契约。
3. 独立 TeachReplayNode 的规划调用发生在回调内；MoveItMotionPlanner 通过 sleep 等待 Future，而入口使用单线程 spin_once。这是起点对齐超时的嫌疑点，尚未通过修复对照验证；另一次服务不可用仍需单独核查生命周期/发现情况。
4. 停止请求的返回不等于确认已经停止；本轮网页终态文案不够明确，需补采轨迹取消状态和停止前后运动数据。
5. live Open3D 的消息匹配未满足截图条件。本轮 probe 收到 CameraInfo 281 条、color 150 条、depth 74 条、detections 41 条；这只是该订阅者的接收计数，不能直接认定相机发布频率或设备丢帧率。需隔离负载并核查查看器队列、QoS、处理延迟。
6. 若干进程退出存在 KeyboardInterrupt/Destroyable 警告，已与业务成功结果分开记录，不视为可靠关闭验收通过。

## 原始证据

- [回归 JUnit](2026-09-11-functional-regression-tests.xml)。
- `.codex_tmp/functional-sim-j6qt_ahl/`：原始 MuJoCo launch 失败日志。
- `.codex_tmp/functional-sim-r25038du/`：首轮项目解释器下的 MoveIt、Dashboard、录制/预检结果。
- `.codex_tmp/functional-sim-7d6abvi_/`：补充非法目标、键盘请求拒绝、停止和 dry-run 诊断；`results.json` 保存 HTTP 错误正文。
- `.codex_tmp/functional-vision-nu2qs2od/`：直接 ros2 run GraspNet 解释器失败日志。
- `.codex_tmp/functional-vision-9i2rl3sd/`：真实视觉多进程测试结果及日志。

仿真使用独立 ROS domain 184；视觉使用 185；回归与仿真接口测试分别使用 181、183，本机发现范围。报告中的成功不包含真实机械臂、实机示教/重力补偿、真机夹取、完整 GUI 或全部故障恢复。本轮发现问题未擅自修改产品实现；后续应先解决这些软件问题，再进入实机动作验收。
