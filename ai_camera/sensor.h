/**
 * sensor.h — TVOC 传感器模块 (串口 /dev/ttyS5 9600bps)
 *
 * 协议: 9 字节帧
 *   2C E4 | TVOC_H TVOC_L | CH2O_H CH2O_L | CO2_H CO2_L | CHECKSUM
 *
 * 报警阈值:
 *   火警: TVOC > 3.0 且 CH2O > 0.5
 *   烟雾: TVOC > 0.5 或 CH2O > 0.1
 *   空气差: CO2 > 1000
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

typedef struct {
    float   tvoc;          // mg/m³
    float   ch2o;          // mg/m³
    int     co2;           // PPM
    int     alarm;         // 0=正常 1=烟雾 2=火警 3=空气差
    int     frame_count;   // 传感器帧计数
} sensor_data_t;

#ifdef __cplusplus
extern "C" {
#endif

int  sensor_init(void);
void sensor_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_H
