# rebotarm_dashboard

网页应用边界，负责页面资源、HTTP 路由、SSE 状态流、状态聚合和 ROS 客户端。

主要入口为 `TeleopStatusPanelNode`。它订阅关节、机械臂、示教和任务状态，并通过受控接口调用运动、夹爪、重力补偿、示教和停止功能。

Dashboard 不生成底层电机命令，不实现轨迹重定时、碰撞检测或示教准备算法。网页预览与真实执行应保持明确区分。

停止接口返回表示请求已发送，不代表动作完成。最终状态取自 Action：canceled 为已取消，后端明确确认服务停止时为 stopped，正常成功才是 done，其他执行错误仍为 failed。晚到的取消回复不能覆盖终态，旧目标的回调不能覆盖新目标状态。

正式关节/夹爪反馈只由执行后端发布，Dashboard 不发布 `/joint_states` 或 `/gripper/state`。真机夹爪仍调用 GripperCommand Action；仿真夹爪通过 SetGripper 服务请求后端，收到响应后才更新操作结果，无后端时返回 unavailable。录制服务由 teach 包提供。

`TeachReplayParameterAdapter` 仅负责把 ROS 参数转换为示教包定义的配置对象；轨迹准备、MoveIt 起点对齐和碰撞预检由 `rebotarm_teach.TeachReplayWorkflow` 统一编排。
