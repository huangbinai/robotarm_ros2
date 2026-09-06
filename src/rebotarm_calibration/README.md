# rebotarm_calibration

标定算法和配置的统一归属包。

## 内容

- `handeye_config.py`：手眼配置加载和验证；
- `tcp_calibration.py`：TCP 样本求解和残差分析；
- `geometry.py`：标定所需刚体几何工具。

标定结果由视觉和运动模块通过文件或 TF 消费，不应复制算法。每份结果建议记录设备、日期、坐标系方向、样本数、单位和残差。当前仍需使用现场采集数据完成标定验收。
