# MuJoCo 多姿态搜索与示教轨迹预演

两项功能只使用仿真和规划服务，不连接真实电机。它们不训练模型，也不会把仿真
排名自动写入真机抓取配置。

## 多姿态搜索

终端 1 只启动 MoveIt 假关节状态规划环境；不要同时启动真机控制器或 MuJoCo ROS
轨迹服务端：

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_moveit_config demo.launch.py use_rviz:=false use_fake_joint_states:=true
```

等出现 `You can start planning now!`，终端 2 运行：

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
third_party/rebotarm_mujoco_venv/bin/python tools/mujoco_grasp_search.py --limit 9
```

如需保留全部数值，追加 `--report /tmp/mujoco_grasp_search.json`。

两个终端的 `ROS_DOMAIN_ID` 必须一致。程序在规范瓶位附近枚举 9 组
末端高度、前后偏移和俯仰角；每组依次检查 MoveIt IK、状态有效性和预抓取、
抓取、抬升三个规划段。通过后为每组新建独立的 MuJoCo 实例，执行三段轨迹，
闭合夹爪并等待稳定，再按双侧接触、最终抬升高度、横向位移和穿透评分。

结果中的 `best_index` 只是**本批相对最高分**；必须同时看到
`best_is_stable_lift=true` 才能说找到仿真稳定抬升候选。
`--detailed` 可以显示每阶段物理指标。当前规范瓶子场景的 9 组实测
`stable_lift_count=0`：闭合和抬升中有短暂双侧接触，但保持阶段均失去接触。
这说明当前候选集或夹爪接触模型仍需改进，不能把最高分当作成功抓取。

## 示教轨迹预演

使用已有示教 JSONL 文件；原文件只读，程序复用示教包的平滑、滤波、重采样与
重定时流水线，再在独立 MuJoCo 模型中按重定时路径播放：

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
third_party/rebotarm_mujoco_venv/bin/python -m rebotarm_teach.mujoco_preview \
  /你的/示教记录.jsonl --viewer
```

无桌面环境时去掉 `--viewer`。报告包含原始/准备后的质量分级、轨迹时长、
最大跟踪误差、接触及瓶子起终位置。红色质量、关节名不匹配、时间戳倒退或
模型限位外起点都会拒绝预演。当前功能只预演手臂六轴；示教记录若没有夹爪
命令，程序不会猜测夹爪开合。MuJoCo 接触并不代替 MoveIt 完整碰撞预检，
预演通过也不构成真机回放授权。
