# rebotarm_vision

RGB-D 感知、YOLO/GraspNet 候选、筛选、显示和视觉抓取执行包。

推荐先以 `execution_mode:=plan_only` 验证相机、TF、TCP、工作空间、IK 和碰撞，再开放真机执行。候选和计划必须满足新鲜度要求，无有效输入时不允许执行旧缓存。

视觉 Python 可通过 `REBOTARM_VISION_PYTHON` 与 ROS 构建解释器分离；模型路径在运行时显式提供。

完整中文说明见 [README_zh.md](README_zh.md)，参数见[视觉抓取七层参数](../../docs/visual_grasp_seven_layer_params.md)。
