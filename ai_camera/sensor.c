/**
 * sensor.c — LoRa 传感器线程 (STM32 → LoRa → RV1126B)
 *
 * 一主多从模式, 协议: 15字节定帧
 *   Byte 0-1:  Slave ID  (uint16 大端, LoRa主机附加, 0x0001=从机1, 0x0002=从机2)
 *   Byte 2-3:  0xCC 0xAA  帧头
 *   Byte 4-5:  TVOC × 1000 (uint16 大端)
 *   Byte 6-7:  CH2O × 1000 (uint16 大端)
 *   Byte 8-9:  CO2          (uint16 大端)
 *   Byte 10-11: Temperature × 10 (int16 大端, 有符号)
 *   Byte 12:   ALARM (0=正常 1=报警)
 *   Byte 13:   BCC (Byte4~12 的 XOR 校验)
 *   Byte 14:   0xDD  帧尾
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include "uart.h"
#include "tcp_server.h"
#include "sensor.h"

#define TVOC_SMOKE     1.0f
#define CH2O_SMOKE     0.3f
#define CO2_BAD_AIR    1500
#define ALARM_COOLDOWN 10
#define FRAME_SIZE     15     /* 2B地址 + 2B帧头 + 9B数据 + 1B BCC + 1B帧尾 */
#define FRAME_HEAD1    0xCC
#define FRAME_HEAD2    0xAA
#define FRAME_TAIL     0xDD
#define DATA_LEN       9      /* TVOC+CH2O+CO2+Temp+ALARM = 9B */

static int g_running = 0;
static pthread_t g_thread;

/* XOR 校验 (与 STM32 _bcc 一致) */
static unsigned char xor_bcc(unsigned char *data, int len)
{
    unsigned char b = 0;
    for (int i = 0; i < len; i++) b ^= data[i];
    return b;
}

static void *sensor_thread(void *arg)
{
    (void)arg;

    unsigned char buf[256];
    int buf_index = 0;
    int frame_count = 0;
    time_t last_alarm = 0;

    if (uart_open("/dev/ttyS5", 9600) < 0) {
        fprintf(stderr, "[SENSOR] uart open failed\n");
        return NULL;
    }
    printf("[SENSOR] started on /dev/ttyS5 @9600 (LoRa master-slave, 15B)\n");

    while (g_running) {
        if (buf_index >= (int)sizeof(buf)) buf_index = 0;

        int n = uart_recv(buf + buf_index, sizeof(buf) - buf_index);
        if (n > 0) {
            buf_index += n;

            while (buf_index >= FRAME_SIZE) {
                int found = 0;
                /* 搜索帧头 CC AA (跳过 slave_id 前缀 2B, 所以 i>=2) */
                for (int i = 2; i <= buf_index - (FRAME_SIZE - 2); i++) {
                    if (buf[i] == FRAME_HEAD1 && buf[i+1] == FRAME_HEAD2) {
                        int frame_start = i - 2; /* 帧起点(含slave_id) */
                        if (frame_start < 0) continue;
                        if (frame_start + FRAME_SIZE > buf_index) break;

                        /* 校验帧尾 */
                        if (buf[frame_start + 14] != FRAME_TAIL) {
                            /* 帧尾不匹配, 跳过此字节继续搜索 */
                            i += 1;
                            continue;
                        }

                        uint16_t slave_id = ((uint16_t)buf[frame_start]<<8) | buf[frame_start+1];

                        /* XOR 校验: 覆盖数据区 (Byte4 ~ Byte12, 共9B) */
                        unsigned char bcc = xor_bcc(buf + frame_start + 4, DATA_LEN);
                        if (bcc != buf[frame_start + 13]) {
                            /* BCC 不匹配, 跳过 */
                            continue;
                        }

                        /* 解析传感器数据 (大端) */
                        uint16_t tvoc_raw = ((uint16_t)buf[frame_start+4]<<8)  | buf[frame_start+5];
                        uint16_t ch2o_raw = ((uint16_t)buf[frame_start+6]<<8)  | buf[frame_start+7];
                        uint16_t co2_raw  = ((uint16_t)buf[frame_start+8]<<8)  | buf[frame_start+9];
                        int16_t  temp_raw = ((int16_t)buf[frame_start+10]<<8) | (int16_t)buf[frame_start+11];
                        /* Byte 12: ALARM (STM32本地判定, 板端忽略) */

                        float tvoc = tvoc_raw * 0.001f;
                        float ch2o = ch2o_raw * 0.001f;
                        int   co2  = co2_raw;
                        float temp = temp_raw * 0.1f;

                        ai_fifo_set_sensor_slave((int)slave_id, tvoc, ch2o, co2, temp);

                        frame_count++;
                        if (frame_count % 20 == 0) {
                            printf("[SENSOR] #%d S%d TVOC=%.3f CH2O=%.3f CO2=%d temp=%.1f (f=%d)\n",
                                   (int)slave_id, (int)slave_id, tvoc, ch2o, co2, temp, frame_count);
                        }

                        /* 消费已解析帧 */
                        memmove(buf, buf + frame_start + FRAME_SIZE,
                                buf_index - frame_start - FRAME_SIZE);
                        buf_index -= (frame_start + FRAME_SIZE);
                        found = 1;
                        break;
                    }
                }
                if (!found) { buf_index = 0; break; }
            }
        }
        usleep(10000);
    }

    uart_close();
    printf("[SENSOR] stopped\n");
    return NULL;
}

int sensor_init(void)
{
    g_running = 1;
    if (pthread_create(&g_thread, NULL, sensor_thread, NULL) != 0) {
        perror("[SENSOR] pthread_create");
        g_running = 0;
        return -1;
    }
    return 0;
}

void sensor_deinit(void)
{
    g_running = 0;
    pthread_join(g_thread, NULL);
}
