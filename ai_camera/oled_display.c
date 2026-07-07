/**
 * oled_display.c — OLED 双页显示 (128x64)
 *
 * 页面0: 传感器数据 (TVOC/CO2/CH2O/温度)
 * 页面1: 报警状态 (传感器报警/AI检测时自动切换, 保持5s)
 */
#include <stdio.h>
#include <string.h>
#include "oled.h"
#include "oled_display.h"
#include "tcp_server.h"

#define STATUS_HOLD_SEC  5   /* 报警后状态页保持秒数 */

static int  g_show_status = 0;
static int  g_status_timer = 0;

int oled_display_init(void)
{
    OLED_Init();
    OLED_DisPlay_On();
    OLED_NewFrame();
    OLED_PrintASCIIString(0, 0, "ELF-RV1126B", &afont16x8, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 2, "AI Camera", &afont16x8, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 4, "Starting...", &afont8x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    printf("[OLED] Init OK\n");
    return 0;
}

void oled_display_update(int has_target, int class_id, float confidence,
                         float tvoc, int co2, int sensor_alarm,
                         float smoke_score, float fire_score)
{
    char line[32];
    float _, ch2o, temp; int __, ___;
    ai_fifo_get_sensor(&_, &ch2o, &__, &___, &temp);

    /* 触发条件: 评分达到语音播报阈值 */
    int alarm_now = (fire_score >= 5.5f) || (smoke_score >= 6.5f);
    if (alarm_now) {
        g_show_status = 1;
        g_status_timer = STATUS_HOLD_SEC * 2;
    }

    OLED_NewFrame();

    if (g_show_status) {
        /* ---- 状态页: 报警信息 ---- */
        if (sensor_alarm != 0) {
            OLED_PrintASCIIString(0, 0, "ENV ABNORMAL", &afont16x8, OLED_COLOR_REVERSED);
        } else if (has_target && class_id == 2) {
            OLED_PrintASCIIString(0, 0, "!!! FIRE !!!", &afont16x8, OLED_COLOR_REVERSED);
        } else if (has_target && class_id == 1) {
            OLED_PrintASCIIString(0, 0, "!! SMOKING !!", &afont16x8, OLED_COLOR_REVERSED);
        } else {
            OLED_PrintASCIIString(0, 0, "!! WARNING !!", &afont16x8, OLED_COLOR_REVERSED);
        }

        snprintf(line, sizeof(line), "CO2:%d T:%.1fC", co2, temp);
        OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);

        snprintf(line, sizeof(line), "TVOC:%.3f", tvoc);
        OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);

        snprintf(line, sizeof(line), "CH2O:%.3f", ch2o);
        OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);

        if (--g_status_timer <= 0) g_show_status = 0;
    } else {
        /* ---- 传感器页: 环境数据 ---- */
        OLED_PrintASCIIString(0, 0, "SENSOR DATA", &afont16x8, OLED_COLOR_NORMAL);

        snprintf(line, sizeof(line), "TVOC:%.3f", tvoc);
        OLED_PrintASCIIString(0, 16, line, &afont16x8, OLED_COLOR_NORMAL);

        snprintf(line, sizeof(line), "CO2: %d ppm", co2);
        OLED_PrintASCIIString(0, 32, line, &afont16x8, OLED_COLOR_NORMAL);

        snprintf(line, sizeof(line), "CH2O:%.3f T:%.1fC", ch2o, temp);
        OLED_PrintASCIIString(0, 48, line, &afont16x8, OLED_COLOR_NORMAL);
    }

    OLED_ShowFrame();
}

void oled_display_deinit(void)
{
    OLED_NewFrame();
    OLED_ShowFrame();
    OLED_DisPlay_Off();
    printf("[OLED] Deinit\n");
}
