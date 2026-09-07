# rebotarm_teach

示教录制、轨迹准备和回放业务包。

## 入口

- `TeachRecorderNode`：按反馈批次去重并写入 JSONL。
- `TeachReplayNode`：读取、准备、预检并发送标准关节轨迹。
- `TeachReplayWorkflow`：供节点和 Dashboard 共用的轨迹准备、起点对齐、碰撞预检与轨迹构造流程。
- `TeachReplaySession`：管理 Dashboard 回放 Action 的发送、取消、结果和运行时跟踪状态。
- `teach_record_types.py`：示教记录、质量和回放决策的数据模型。
- `teach_record_repository.py`：JSONL 编解码、读取和准备文件写入。
- launch：`rebotarm_bringup/teach_record.launch.py` 和 `teach_replay.launch.py`。

原始记录不是可直接信任的执行轨迹。真实回放前必须经过有限值、跳变、滤波、重采样、重定时、起点和碰撞检查。先使用 `dry_run:=true`。

本包管理示教文件和工作流；通用轨迹算法属于 `rebotarm_motion`，网页属于 `rebotarm_dashboard`。
`teach_recording.py` 保留旧导入兼容，并集中质量分析和回放规则，不再实现文件存储。
