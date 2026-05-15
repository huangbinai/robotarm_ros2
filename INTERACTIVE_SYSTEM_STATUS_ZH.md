# 机械臂交互控制系统当前状态说明

## 1. 当前目标

当前系统的核心目标是：

```text
RViz 交互 marker
-> pose_target
-> preview
-> execute
```

也就是用户在 RViz 中拖动机械臂末端目标后，系统先生成目标位姿，再做预览求解，最后再通过执行接口进入仿真或真机执行链。

## 2. 主要 ROS2 包职责

### `rebotarm_bringup`

负责：

- launch 启动入口
- URDF / 机械臂描述
- 正式 RViz 配置

### `rebotarm_interactive_control`

负责：

- interactive marker 交互
- preview 链路
- execution 链路
- debug / smoke 测试工具

### `rebotarmcontroller`

负责：

- 真机控制器的 ROS2 包装层
- joint state 发布
- trajectory / 控制接口
- 对接底层 Python SDK / motorbridge

### `rebotarm_msgs`

负责：

- 自定义 service / action 接口

## 3. 当前三大核心节点

### `MarkerServerNode`

职责：

- 创建正式的 `ee_target` interactive marker
- 接收 RViz feedback
- 发布：

```text
/rebotarm/interactive_control/pose_target
```

它只负责：

- 交互输入入口

它不负责：

- IK
- preview 求解
- execute
- estop

---

### `PreviewNode`

职责：

- 订阅：

```text
/rebotarm/interactive_control/pose_target
```

- 进行 preview 求解
- 发布：

```text
/rebotarm/interactive_control/preview
/rebotarm/interactive_control/status
```

它当前内部仍然复用了：

- `PreviewManager`
- `PosePreviewSolver`

因此它已经在节点层面独立出来了，但内部实现后续还可以继续收紧。

---

### `ExecutionNode`

职责：

- 订阅最近一次 preview 结果
- 提供服务：

```text
/rebotarm/interactive_control/execute_preview
/rebotarm/interactive_control/estop
/rebotarm/interactive_control/reset_estop
/rebotarm/interactive_control/set_mode
```

- 处理 simulation / real 分流
- 将真机执行请求送入 trajectory action 路径

它负责的是：

- 执行控制入口

## 4. 当前正式主链

现在系统正式主链已经整理为：

```text
RViz
-> MarkerServerNode
-> /pose_target
-> PreviewNode
-> /preview + /status
-> ExecutionNode
-> /execute_preview
-> reBotArmController
```

这就是当前工程里最重要的一条主链。

## 5. 当前正式启动方式

### 正式入口

后续默认使用：

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py
```

这个入口会启动：

- `reBotArmController`
- `robot_state_publisher`
- `MarkerServerNode`
- `PreviewNode`
- `ExecutionNode`
- `rviz2`

这是当前正式系统入口。

## 6. 当前调试入口

如果只是单独排查 marker / preview / execution，可使用这些调试入口：

- `interactive_debug.launch.py`
- `marker_server_debug.launch.py`
- `preview_debug.launch.py`
- `interactive_stage5_debug.launch.py`

这些是调试入口，不是正式主入口。

## 7. Legacy / 兼容入口

以下入口目前仍然保留，但已经不推荐作为默认工作流：

- `interactive_basic.launch.py`
- `InteractiveTargetNode`

它们现在主要用于：

- 兼容旧链路
- 防止迁移过程中一下子失去历史入口

后续推荐的正式架构是：

```text
MarkerServerNode -> PreviewNode -> ExecutionNode
```

## 8. 当前已经验证过的能力

### 交互能力

- RViz marker 显示正常
- marker 拖动正常
- `/ee_target/feedback` 有输出

### 预览链路

- `pose_target -> preview` 链路已打通
- `/preview` 有输出
- `/status` 有输出

### 执行链路

- `execute_preview` 服务可调用
- `execution accepted` 已验证

### 架构能力

- 正式链路与调试链路已经分开
- marker / preview / execution 已拆为独立节点
- 正式入口已经建立

## 9. 当前仍属“临时实现”的部分

### `PreviewNode` 内部仍然复用旧 preview 管理逻辑

目前它仍然复用了：

- `PreviewManager`
- `PosePreviewSolver`

这意味着：

- 节点边界已经清晰
- 但内部逻辑还不是最终最优结构

### `ExecutionNode` 目前仍是“最小落地版”

当前已经能工作，但后续还可以进一步改进：

- 更清晰的状态存储
- 更正式的 preview 到 execute 数据交接
- 更明确的 simulation / real 边界

### `InteractiveTargetNode` 仍然存在

它现在已经不应该继续扩功能，后续应继续清理或最终移除。

## 10. 当前最容易踩坑的地方

### 1. launch 混用

不要同时乱起：

- `interactive_basic.launch.py`
- `interactive_system.launch.py`
- 各种 debug launch

否则容易出现：

- namespace 冲突
- marker update 序号异常
- 当前不知道到底是谁在发 topic

### 2. build / install 缓存残留

你已经遇到过：

- `PackageNotFoundError`
- entry point 不更新

如果：

- 改了 `setup.py`
- 新增了 console script
- launch 找不到新入口

优先这样清理：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 3. DDS 共享内存残留

你之前遇到过：

- `RTPS_TRANSPORT_SHM Error`

这通常不是代码逻辑本身的问题，更像是：

- ROS2 / Fast DDS 共享内存残留
- 上一次进程未正常退出

必要时可以：

- 关干净 ROS2 进程
- 或直接重启虚拟机

## 11. 当前工程定位

一句话概括：

```text
这个项目已经不再是一个临时拼起来的原型，而是一个完成了第一轮架构重整、具备继续接 MoveIt2 的 ROS2 机械臂交互控制骨架。
```

这说明你现在最难的“把乱工程收成能持续开发的工程骨架”这一步，已经基本完成了。

## 12. 下一阶段建议

下一阶段建议按这个顺序推进：

### 第一优先级：继续稳定化收尾

建议继续做：

- 进一步清理 `InteractiveTargetNode`
- 收紧 `PreviewNode` / `ExecutionNode` 的内部实现
- 统一状态流

### 第二优先级：开始接 MoveIt2

等稳定化收尾差不多后，再开始：

- MoveIt2 接入 preview 层
- IK 求解
- 可达性判断
- 约束与轨迹规划

也就是说：

```text
现在最合适的下一阶段是：稳定化收尾 + MoveIt2 接入
```
