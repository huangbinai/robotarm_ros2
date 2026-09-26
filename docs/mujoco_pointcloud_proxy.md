# RGB-D 点云到 MuJoCo 碰撞代理

当前先做一个可验证的最小闭环：目标点云输入 `base_link` 坐标，程序用去除少量深度
离群点后的 AABB 生成盒状碰撞代理，并写出一份独立 MuJoCo 场景。规范
`scene.xml` 不会被改写；点云没有覆盖到的背面、质量和摩擦不会被自动猜成真实值。

输入可为 `Nx3` 的 `.npy`、带 `points` 数组的 `.npz` 或 JSON 点列表。示例：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export REBOTARM_MUJOCO_PYTHON="$PWD/third_party/rebotarm_mujoco_venv/bin/python"
ros2 run rebotarm_simulation rebotarm_mujoco_pointcloud_proxy \
  /tmp/bottle_points_base.npy \
  --base-scene src/rebotarm_simulation/models/rebotarm/scene.xml \
  --output-scene /tmp/bottle_proxy_scene.xml \
  --report /tmp/bottle_proxy_report.json
```

检查输出代理：

```bash
MUJOCO_GL=egl "$REBOTARM_MUJOCO_PYTHON" \
  -m rebotarm_simulation.mujoco_health \
  --model /tmp/bottle_proxy_scene.xml --renderer-timeout 30
```

下一步可以让当前 `build_detection_cloud()` 输出同一份 base-frame 点云，再把这个
代理交给 MoveIt/MuJoCo 做碰撞和抓取候选复核。正式应用时再加入桌面分离、圆柱/凸包
拟合、多帧点云融合和质量/摩擦参数标定。
