#ifndef AI_FIFO_H
#define AI_FIFO_H

#include <stdint.h>

/* ============================================================
 *  ai_fifo — FIFO + TCP 发送 AI 检测坐标 + 传感器数据
 *
 *  协议: 52 字节 (40B AI 坐标 + 16B 传感器)
 *  下游程序读取 /tmp/ai_track.fifo 或 TCP:9999
 * ============================================================ */

#define AI_FIFO_PATH  "/tmp/ai_track.fifo"
#define AI_TCP_PORT   9999

typedef struct {
    /* ---- 来源标识 (4 字节) ---- */
    int     slave_id;        // 0=AI检测, 1=从机1传感器, 2=从机2传感器

    /* ---- AI 检测 (36 字节) ---- */
    int     cx, cy;          // 目标中心坐标
    int     w, h;            // 目标宽高
    int     frame_w, frame_h;// 原始分辨率
    float   confidence;      // 置信度 (最高框)
    int     has_target;      // 1=有目标, 0=无
    int     class_id;        // 类别: 0=no_smoking 1=smoking 2=fire 3=smoke

    /* ---- 传感器数据 (20 字节) ---- */
    float   tvoc;            // TVOC mg/m³
    float   ch2o;            // CH2O mg/m³
    int     co2;             // CO2 PPM
    int     sensor_alarm;    // 0=正常 3=环境指数异常
    float   temperature;     // 温度 °C

    /* ---- 融合评分 (8 字节) ---- */
    float   smoke_score;     // 吸烟评分 (0~10)
    float   fire_score;      // 火灾评分 (0~10)
} ai_coord_msg_t;  // = 68 字节

/* sensor_alarm 值 */
#define SENSOR_OK      0
#define SENSOR_SMOKE   1
#define SENSOR_FIRE    2
#define SENSOR_BAD_AIR 3

#ifdef __cplusplus
extern "C" {
#endif

int  ai_fifo_init(void);
void ai_fifo_send(int cx, int cy, int w, int h, int fw, int fh, float conf, int has_target, int class_id,
                  float smoke_score, float fire_score);
void ai_fifo_set_sensor(float tvoc, float ch2o, int co2, int alarm, float temperature);
void ai_fifo_set_sensor_slave(int slave_id, float tvoc, float ch2o, int co2, float temperature);
void ai_fifo_flush_sensor(void);
void ai_fifo_get_sensor(float *tvoc, float *ch2o, int *co2, int *alarm, float *temperature);
void ai_fifo_get_scores(float *smoke_score, float *fire_score);
int  ai_fifo_tcp_health(void);   /* 0=正常 1=需要重启 */
void ai_fifo_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // AI_FIFO_H
