# rebotarm_teach

示教录制、轨迹准备和回放业务包。

## 入口

- `TeachRecorderNode`：唯一录制实现，按 JointState 消息时间戳去重并写入 JSONL；控制器不再内置录制器。
- `TeachReplayNode`：读取、准备、预检并发送标准关节轨迹。
- `TeachReplayWorkflow`：供节点和 Dashboard 共用的轨迹准备、起点对齐、碰撞预检与轨迹构造流程。
- `TeachReplaySession`：管理 Dashboard 回放 Action 的发送、取消、结果和运行时跟踪状态。
- `teach_record_types.py`：示教记录、质量和回放决策的数据模型。
- `teach_record_repository.py`：JSONL 编解码、读取和准备文件写入。
- launch：`rebotarm_bringup/teach_record.launch.py` 和 `teach_replay.launch.py`。

原始记录不是可直接信任的执行轨迹。真实回放前必须经过有限值、跳变、滤波、重采样、重定时、起点和碰撞检查。先使用 `dry_run:=true`。

本包管理示教文件和工作流；通用轨迹算法属于 `rebotarm_motion`，网页属于 `rebotarm_dashboard`。

录制严格按 `joint_names` 参数选择并排序关节，默认六轴；MuJoCo 输入里的手指关节不会混入六轴轨迹。缺轴、重复名称、长度不一致或非有限反馈拒绝写入。回放使用多线程 executor 和独立服务响应回调组，避免在规划回调等待 Future 时阻塞响应。

旧的八关节文件需导出为新文件后重新做预检查，不覆盖原始数据：

```bash
source tools/source_ubuntu_env.bash
python tools/convert_arm_teach_record.py old_record.jsonl arm6_record.jsonl
```

转换只保留六轴和原时间戳，不代表轨迹已安全；输出文件已存在时拒绝覆盖。
`teach_recording.py` 保留旧导入兼容，并集中质量分析和回放规则，不再实现文件存储。

完整 app/teleop 组合入口会启动一个空闲录制节点；普通驱动/RViz 入口默认不启动。独立 `teach_record.launch.py` 与已有录制服务不能叠加。真机组合要求重力补偿状态，仿真组合关闭这一硬件状态要求；重复/倒退的消息时间戳不会重复写样本。
