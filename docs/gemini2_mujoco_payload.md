# Gemini 2 与 Seeed 支架：MuJoCo 装配初版

当前附件仅在 MuJoCo 默认模型启用（`enabled: true`），采用用户确认外观位置的装配。
这不代表实物尺寸或动力学标定通过；支架长度与实物版本仍待进一步核对。
启用后 `models/rebotarm/scene.xml` 使用的生成 `robot.xml` 会包含固定在
`end_link` 下的 Gemini 2 与旧版 Seeed 支架。没有新增自由度、执行器或 ROS TF。
仅修改仿真；MoveIt URDF、真实控制器重力补偿与生产手眼外参不变。
**因此当前 MoveIt 规划模型尚不包含这些附件碰撞体；MuJoCo 接触检查不能替代
规划场景同步，更不能作为真实机器人附件避障验收。**

## 数值与假设

| 项目 | 当前值 | 依据 |
| --- | --- | --- |
| Gemini 2 质量 | 0.098 kg | Orbbec 产品规格 |
| 相机外形 | 官方 Orbbec STL | CAD 导出壳体，包围盒约 30×89.427×25.025 mm |
| 相机碰撞 | 30×90×25 mm 长方体（厚、宽、高） | 规格尺寸近似 |
| 相机质心/惯量 | 包围盒中心、均匀长方体惯量 | 估计，不是厂家辨识 |
| 支架 | Seeed 旧版 STEP 中的一个实体 | STEP 毫米转换为米，保留 CAD 装配坐标 |
| 支架质量 | 单件 0.022 kg | 用户实测打印支架重量22 g |
| 支架质心/惯量 | CAD 实体几何计算，按用户实测质量缩放 | 均匀有效密度近似，不复现真实30%填充内部结构 |
| 新增总质量 | 0.120 kg | 不包含线缆和螺丝 |

当前使用用户确认的旧版 `840971c817a9289f24bf6dfb82d92627026c77de`
（2026-04-21），替换2026-09-18新版，移除新版独立长连接臂。
旧版为一个实体，网格轴向包围盒约95.76×53.51×28.46 mm。
旧版卡座圆弧半径28.5 mm，轴线对齐end X；相机按旧版后板M3孔轴
`(0,-0.2588160164,0.9659266378)` 与座面定位。相机倾角随旧版座面更新。
卡座采用内侧安装面配准：CAD z=-16.9 mm 对齐夹爪 end X=-103.20933 mm，
因此总成平移为[-0.08630933,0.0002079068933,0.0000007587264] m。
CAD安装孔中心x=-11.2079069/10.7920931、y=29.9992413 mm，配准到
夹爪Y=±11、Z=30 mm。之前把z=0槽口贴到安装面，只消除表面缝隙，
未让夹爪凸台嵌入卡槽；该判断撤回。此处是CAD/网格配准，不是实物精度验收。

相机中心不是光学原点，不使用已有 handeye 平移直接充当质心。
原 `wrist_camera_mount` site 保留其兼容语义，不作为新附件的装配基准。

支架视觉使用单件完整 CAD 网格；接触使用每件网格的 MuJoCo 凸包，属于
保守近似，不保证复现夹槽和凹面。相机使用盒形碰撞体。
所有附件显式指定惯量，视觉和碰撞 geom 的 mass 为0，避免重复计重。
原 `end_link` 的0.5 kg保持不变；若后续确认它已经包含相机/支架，需要重新
核算总成质量，当前不能认为已经排除了这一来源不确定性。

## 来源与许可

- 相机：https://github.com/orbbec/OrbbecSDK_ROS2/tree/main/orbbec_description/meshes/gemini2
- 产品规格：https://www.orbbec.com/products/stereo-vision-camera/gemini-2/
- 支架：https://github.com/Seeed-Projects/reBot-DevArm/blob/840971c817a9289f24bf6dfb82d92627026c77de/hardware/reBot_B601_DM/3D_Printed_Parts/D435_Gemini2_Mount.step
- Orbbec 上游 Xacro 的所有link质量总和约26.81g，与98g整机不同，未直接复用。
- `models/rebotarm/assets/gemini2/` 保留STEP源文件、SHA256来源记录和两方许可证；
  相机资源遵循上游Apache-2.0，Seeed硬件资源保留CERN-OHL-W-2.0。
- 支架STL用独立临时环境内的 `cadquery-ocp-novtk==7.8.1.1.post1` 转换，
  `STEPControl_Reader` 读取、`gp_Trsf.SetScale(...,0.001)` 转米，
  `BRepMesh_IncrementalMesh(shape,0.0001)` 网格化。此工具不是运行时依赖。

## 调整和重新生成

编辑 `src/rebotarm_simulation/config/gemini2_payload.yaml` 后重新生成，
不要手改自动生成的 `robot.xml`：

```bash
PYTHONPATH=src/rebotarm_simulation third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.urdf_to_mjcf --repo-root .
PYTHONPATH=src/rebotarm_simulation third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.urdf_to_mjcf --repo-root . --check
```

`enabled: false` 加重新生成可移除附件。之后重建 `rebotarm_simulation`。
运行无需联网或CAD软件。已有MuJoCo进程必须重新加载模型才能显示新附件。

## 验证范围

新增测试检查固定连接、不增加自由度、显式总质量、资源完整性、生成一致性、
规范home姿态无附件接触及100步有限数值。离屏预览确认附件可见。
这些检查不证明全工作空间无碰撞、装配实测准确或真实动力学标定通过。
