#ifndef GST_PIPELINE_H
#define GST_PIPELINE_H

#include <stdint.h>
#include <stddef.h>

/* ============================================================
 *  gst_pipeline — GStreamer RTSP 推流管线 (替代 shell 脚本)
 *
 *  管线结构:
 *    appsrc → mppjpegdec → videoconvert → mpph264enc
 *           → h264parse → rtph264pay → udpsink (→ mediamtx)
 *
 *  与 shell 脚本版本的区别:
 *    - 数据源从 v4l2src 改为 appsrc, 由 main.c 统一喂帧
 *    - 硬件编解码 (mpp*) 保持不变
 *    - 在 main.c 同一进程内运行, 无需 fork
 * ============================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 GStreamer 并构建管线
 * @param width   输入 JPEG 图像宽度
 * @param height  输入 JPEG 图像高度
 * @param fps     帧率
 * @param bitrate H264 编码码率 (bps), 0 使用默认 4Mbps
 * @return 0 成功, -1 失败
 */
int  gst_pipeline_init(int width, int height, int fps, int bitrate);

/**
 * 推送一帧 MJPEG 数据到管线 (非阻塞, 线程安全)
 * @param data  JPEG 压缩数据
 * @param size  数据长度
 * @param frame_id 帧序号 (用于调试)
 * @return 0 成功, -1 失败/管线未就绪
 */
int  gst_pipeline_push_frame(const uint8_t *data, size_t size, int64_t frame_id);

void gst_pipeline_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // GST_PIPELINE_H
