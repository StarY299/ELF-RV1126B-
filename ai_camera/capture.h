#ifndef CAPTURE_H
#define CAPTURE_H

#include <stdint.h>
#include <stddef.h>

/* ============================================================
 *  capture — V4L2 视频采集模块
 *
 *  从 USB 免驱摄像头直接取 MJPEG 帧，不依赖 OpenCV。
 *
 *  设计:
 *    - 基于 V4L2 ioctl + mmap，4 个缓冲区低延迟轮转
 *    - 输出原始 MJPEG 压缩帧，交给下游:
 *        GStreamer → 硬件 mppjpegdec 解码 → 推流
 *        cv_branch → OpenCV imdecode 软解 → AI 推理
 *    - 采集线程阻塞在 dequeue，不浪费 CPU
 * ============================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 采集帧输出 ---------- */
typedef struct {
    uint8_t *data;          // 帧数据 (MJPEG)
    size_t   size;          // 字节数
    int64_t  timestamp_us;  // 采集时间戳 (单调时钟)
    int64_t  frame_id;      // 帧序号 (单调递增)
} capture_frame_t;

/* ---------- API ---------- */

/**
 * 自动查找 UNIQUESKY USB 摄像头设备节点
 * @param out_device 输出缓冲区
 * @param out_size   缓冲区大小
 * @return 0 成功, -1 未找到
 */
int  capture_find_device(char *out_device, size_t out_size);

/**
 * 打开摄像头并配置采集格式
 * @param device  设备节点, e.g. "/dev/video52"
 * @param width   图像宽度
 * @param height  图像高度
 * @param fps     帧率
 * @return 0 成功, -1 失败
 */
int  capture_init(const char *device, int width, int height, int fps);

/**
 * 取下一帧 (阻塞直到有新帧)
 * @param frame  输出帧, 调用方不应 free(data)
 * @return 0 成功, -1 设备错误, -2 超时 (未实现)
 *
 * 注意: frame->data 指向内部 mmap 缓冲区,
 * 在下次 capture_get_frame 调用后失效。
 * 调用方如有需要应自行拷贝。
 */
int  capture_get_frame(capture_frame_t *frame);

/**
 * 停止采集，释放资源
 */
void capture_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CAPTURE_H
