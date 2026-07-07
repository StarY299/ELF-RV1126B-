#ifndef RTSP_SERVICE_H
#define RTSP_SERVICE_H

#include <sys/types.h>

/* ============================================================
 *  rtsp_service — mediamtx RTSP 服务器进程管理
 *
 *  GStreamer 推流管线已迁移至 gst_pipeline.c (进程内 C API)
 *  此模块仅管理 mediamtx 的 fork / 监控 / 终止
 * ============================================================ */

// 获取 mediamtx 子进程 PID（供外部监控存活状态）
pid_t get_mediamtx_pid(void);

// 启动 mediamtx
int start_mediamtx(void);

// 停止 mediamtx
int stop_mediamtx(void);

#endif
