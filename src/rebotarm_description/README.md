# rebotarm_description

真机、MoveIt、Dashboard 与仿真共享的资源包，不包含 ROS 节点或 SDK 代码。

- `description/urdf/reBot-DevArm_fixend.urdf`：唯一维护的 URDF 源。
- `description/meshes/`：URDF 使用的 mesh。
- `config/arm.yaml`、`gripper.yaml`：共用电机身份与标称控制参数，真机/仿真由各自适配器读取。

现场串口和配置覆盖由 bringup 的 launch 参数注入；运行与安全策略仍在 bringup 配置中。
MoveIt 保留自身 SRDF/规划参数，MuJoCo 保留生成的 MJCF 和任务场景。
改变物理参数后需要重验规划、仿真和受影响的真机行为，资源共用不代表已完成实机验收。
