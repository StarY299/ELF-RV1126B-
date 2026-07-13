# RV1126B 多源融合吸烟行为与火灾识别智慧物联系统

基于 RV1126B 边缘AI芯片的端侧实时吸烟行为与火灾检测系统，融合 YOLOv8 INT8 RKOPT 视觉检测、LoRa 无线传感器网络与双轴舵机云台追踪。

## 目录结构

```
ai_camera/main.c — 板端主程序（16线程架构入口，单文件展示工程结构）
Servo/           — 独立舵机追踪模块（P+EMA控制器）
model/           — 模型文件（ONNX + INT8 RKNN）
dataset/         — 数据集说明 + 标注文件 + 样例图片
qt_monitor.py    — PC端上位机（PySide6 SCADA界面）
```

## 关键技术

- **INT8 RKOPT 量化**：box/cls 分离输出解决混合张量量化塌缩，NPU 推理 31ms
- **LoRa 15B 定帧协议**：CC AA 头 + XOR BCC + DD 尾三重校验，一主双从机分布式感知
- **视觉-传感器双通道加权融合评分**：28 帧滑动窗口占比判定，温度≥80°C 直达火灾判定
- **P+EMA 双平滑云台控制**：输入/输出双层滤波 + 速度预测惯性滑行 + PWM 硬件断电
- **单进程 16 线程一体化架构**：NPU~33% / CPU~50% / 内存~240MB

## 许可证

MIT License
