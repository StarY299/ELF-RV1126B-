# RV1126B 多源融合吸烟行为与火灾识别智慧物联系统

基于 RV1126B 边缘AI芯片的端侧实时吸烟行为与火灾检测系统，融合 YOLOv8 INT8 RKOPT 视觉检测、LoRa 无线传感器网络与双轴舵机云台追踪。

## 系统架构

- **板端**：rtsp_test 单进程 16 线程，C/C++约 6000 行
- **上位机**：qt_monitor.py，PySide6 约 1700 行
- **模型**：YOLOv8 INT8 RKOPT，11.9MB，NPU 推理 31ms

## 目录结构

```
ai_camera/     — 板端嵌入式主程序（16线程架构）
  main.c        — 系统入口，线程管理
  ai_processor.c — AI推理+画框+融合评分+语音报警
  rknn_infer.c  — RKOPT DFL后处理+NMS+letterbox
  sensor.c      — LoRa 15B定帧协议解析
  servo_thread.c — 三模式云台控制（追踪/手动/扫描）
  tcp_server.c  — TCP多客户端传感器+评分推送
  rtsp_stream.c — GStreamer+MPP硬件编解码推流

Servo/         — 独立舵机追踪模块
  tracker.c     — P+EMA双重平滑控制器
  main.c        — 独立servo_track进程

qt_monitor.py  — PC端SCADA上位机（融合判定+波形+DVR+远程控制）
```

## 关键技术

- INT8 RKOPT 量化：box/cls 分离输出解决混合张量量化塌缩
- LoRa 15B 定帧协议：CC AA 头 + XOR BCC + DD 尾三重校验
- 视觉-传感器双通道加权融合评分（28 帧滑动窗口占比判定）
- P+EMA 双平滑云台控制（输入/输出双层滤波 + 速度预测 + PWM 断电）
- 单进程 16 线程一体化架构（NPU~33%/CPU~50%/内存~240MB）

## 许可证

MIT License
