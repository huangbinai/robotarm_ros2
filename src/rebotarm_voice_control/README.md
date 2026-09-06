# rebotarm_voice_control

文本、文件语音和实时语音的任务级控制包。

## 执行模式

- `dry_run`：解析、校验和展示计划，不运动；
- `sim`：发送到仿真动作接口；
- `real`：允许调用真机链路，必须显式选择。

主要组件包括 ASR/实时会话客户端、意图解析器、结构化工具 schema、安全守卫、任务规划器和 ROS Action 传输。LLM/ASR 输出均是不可信输入，不能绕过 schema 和物理边界。

推荐入口：`ros2 launch rebotarm_voice_control voice_control.launch.py`。当前功能为实验性，详见[功能状态](../../docs/feature_status_zh.md)。
