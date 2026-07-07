#ifndef AI_MSG_H
#define AI_MSG_H

/* AI 坐标消息 (与 ai_camera/ai_fifo.h 的 ai_coord_msg_t 完全一致，52 字节) */
typedef struct {
    /* ---- AI 检测 (36 字节) ---- */
    int   cx, cy;           // 目标中心坐标
    int   w, h;             // 目标宽高
    int   frame_w, frame_h; // 原始分辨率
    float confidence;       // 置信度
    int   has_target;       // 1=有目标, 0=无
    int   class_id;         // 类别: 0=smoking 1=fire 2=smoke

    /* ---- 传感器数据 (16 字节，保留读取但不使用) ---- */
    float tvoc;
    float ch2o;
    int   co2;
    int   sensor_alarm;
} ai_msg_t;

#endif
