/**
 * oled_display.h — OLED 信息显示模块 (128x64 I2C)
 */
#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

int  oled_display_init(void);
void oled_display_update(int has_target, int class_id, float confidence,
                         float tvoc, int co2, int sensor_alarm,
                         float smoke_score, float fire_score);
void oled_display_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
