# Sim2Real / Real2Sim 基础层设计

## 目标

在不连接实机、不改变当前 MuJoCo 控制器行为的前提下，为后续 Sim2Real 和 Real2Sim 建立独立的仿真侧基础层：可复现的动力学/执行器随机化、统一轨迹日志、仿真回放和轨迹误差比较。

第一阶段不包含策略训练、不包含真实硬件通信、不把真实数据直接写入控制器。

## 范围与边界

新增功能放在 `rebotarm_simulation` 包的 `sim2real` 子包中，核心 `RebotArmMujoco` 只增加最小的生命周期钩子或只读访问，不承担日志格式、文件 IO 和比较报告职责。

推荐结构：

```text
src/rebotarm_simulation/rebotarm_simulation/sim2real/
  __init__.py
  schemas.py
  randomization.py
  trajectory_log.py
  replay_compare.py
src/rebotarm_simulation/config/sim2real_randomization.yaml
```

所有入口都默认仿真模式；不得启动硬件节点、打开 CAN/串口或要求 `use_hardware:=true`。

## 随机化设计

`RandomizationConfig` 使用明确的范围和 seed，范围采用 `[min, max]`，每次 reset/episode 生成一份不可变 `RandomizationSample`。第一阶段支持：

- body mass scale
- joint damping scale
- geom friction scale
- arm actuator torque scale
- control latency（离散物理步数）
- action、position、velocity noise 标准差

随机化必须满足：

1. 相同模型、配置和 seed 得到相同 sample；
2. 质量、阻尼、摩擦、torque scale 为正；
3. actuator 最终 ctrl/force range 仍受 MJCF 和现有电机上限约束；
4. reset/close 后不残留上一个 episode 的随机状态；
5. 默认配置为零噪声、scale=1，保持现有行为完全兼容。

随机化只通过显式 `apply_randomization(sample)` / `restore_randomization()` 或等价的 session 对象进入仿真，禁止在每个控制周期偷偷改变模型参数。

## 统一轨迹数据格式

`TrajectorySample` 是与 ROS 2 无关的不可变记录，字段固定为：

```text
schema_version
episode_id
step_index
simulation_time
joint_positions[6]
joint_velocities[6]
joint_targets[6]
actuator_torques[6]
gripper_width
gripper_target_width
end_effector_position[3]
end_effector_orientation_xyzw[4]
action[6 or 7]
max_contact_force
contact_count
source            # sim 或 real
```

记录器提供：

- `append(sample)`：校验维度、有限性和单调时间；
- `to_jsonl(path)` / `from_jsonl(path)`：一行一个 sample，便于流式写入；
- `summary()`：样本数、起止时间、最大速度/力矩/接触力。

真实数据适配器以后只需把设备数据转换为同一 `TrajectorySample`，不应修改比较器或仿真控制器。

## 回放与比较

回放器读取 action 序列，在指定 seed 和随机化 sample 下调用现有 `reset()` / `step()`，生成新的仿真轨迹。第一阶段只保证仿真日志回放，不宣称 sim-to-real 精度。

`compare_trajectories(reference, candidate)` 输出：

- joint position RMSE / max error；
- joint velocity RMSE / max error；
- end-effector position RMSE / max error；
- gripper width RMSE / max error；
- actuator torque RMSE / max error；
- contact force RMSE / max error；
- 时间轴是否单调、关节/动作维度是否一致；
- `ok` 和逐项阈值结果。

比较器不自动修改参数，不自动判定“真实可用”；阈值由调用方显式传入。

## API 草案

```python
from rebotarm_simulation.sim2real.randomization import RandomizationConfig
from rebotarm_simulation.sim2real.trajectory_log import TrajectoryRecorder

config = RandomizationConfig.from_yaml(path)
sample = config.sample(seed=7)

with RebotArmMujoco() as sim:
    session = sim.randomization_session(sample)
    recorder = TrajectoryRecorder(episode_id="reach-0007", source="sim")
    obs, info = env.reset(seed=7)
    for step_index in range(100):
        action = policy(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        recorder.record_from_env(env, action, step_index)
        if terminated or truncated:
            break
    recorder.to_jsonl("logs/reach-0007.jsonl")
```

实际实现可以调整方法名，但必须保留三个边界：随机化、记录、比较相互独立；文件格式不依赖 ROS 2；默认不改变当前仿真行为。

## 验收标准

### 单元测试

- 配置范围、seed、非法参数校验；
- 随机化 sample 可复现且默认 sample 与当前参数等价；
- `TrajectorySample`/JSONL 往返保持数值和字段一致；
- 非有限值、错误维度、非单调时间被拒绝；
- 比较器的 RMSE、最大误差和阈值结果正确。

### 运行测试

- 同一 action + seed 的两次仿真回放结果在数值容差内一致；
- 随机化 episode 仍能通过 joint/actuator 限位检查；
- 现有 MuJoCo、ROS 2、MoveIt、接触和 headless 测试不回归；
- Windows 单元测试和 Ubuntu VM 仿真验收均可运行；
- 全程不连接实机。

## 后续阶段

本设计完成后再编写实现计划。后续 Real2Sim 阶段才增加真实日志采集适配器、时间同步、系统辨识和参数拟合；后续 Sim2Real 阶段再把随机化参数接入训练环境和部署安全门。
