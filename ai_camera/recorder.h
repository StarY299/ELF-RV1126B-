/**
 * recorder.h — 板端本地录像 (JPEG直存, SD卡)
 *
 * 每15帧存1帧 (~2fps), 每5分钟自动分片, 自动清理旧文件
 * 路径: /userdata/recordings/YYYYMMDD_HHMMSS.jpg
 */
#ifndef RECORDER_H
#define RECORDER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int  recorder_init(void);
void recorder_feed(const uint8_t *jpeg_data, size_t size);
void recorder_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
