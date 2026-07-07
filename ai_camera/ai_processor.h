#ifndef CV_BRANCH_H
#define CV_BRANCH_H

#include <stdint.h>
#include <stddef.h>

/* ============================================================
 *  cv_branch — OpenCV AI 数据分支
 *
 *  与 GStreamer 推流管线并行运行：
 *    USB Camera → V4L2 ─┬── GStreamer Pipeline → RTSP 推流 (原有)
 *                        └── cv_branch → OpenCV AI 处理   (新增)
 *
 *  设计要点:
 *    - 独立的处理线程，不阻塞主采集/推流管线
 *    - 帧数据通过零拷贝引用计数传递，避免内存拷贝
 *    - 结果通过回调异步上报
 *    - 当 AI 处理跟不上帧率时自动丢帧 (leaky queue)
 * ============================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 像素格式枚举 ---------- */
typedef enum {
    CV_FMT_BGR888,      // OpenCV 默认 BGR 24-bit
    CV_FMT_RGB888,      // RGB 24-bit
    CV_FMT_NV12,        // YUV NV12 (RV1126 硬件常用)
    CV_FMT_NV21,        // YUV NV21
    CV_FMT_YUYV,        // YUYV 4:2:2
    CV_FMT_JPEG,        // MJPEG 压缩帧 (当前摄像头输出格式)
    CV_FMT_GRAY8,       // 灰度 8-bit
} cv_pixel_format_t;

/* ---------- 帧数据结构 ---------- */
typedef struct {
    uint8_t         *data;          // 帧数据指针
    size_t           size;          // 数据长度 (字节)
    int              width;         // 图像宽度
    int              height;        // 图像高度
    int              stride;        // 行步长 (bytes)，<=0 表示 width * bpp
    cv_pixel_format_t format;       // 像素格式
    int64_t          timestamp_us;  // 采集时间戳 (微秒)
    int64_t          frame_id;      // 帧序号 (单调递增)
} cv_frame_t;

/* ---------- AI 检测结果 ---------- */
typedef struct {
    // ---- 边界框 ----
    int     class_id;       // 类别 ID
    float   confidence;     // 置信度 [0.0, 1.0]
    int     x, y;           // 左上角坐标
    int     w, h;           // 宽高

    // ---- 扩展字段 (预留) ----
    char    label[64];      // 类别标签 (如 "person", "car")
    void   *user_data;      // 用户自定义数据
} cv_detection_t;

typedef struct {
    cv_detection_t *detections;     // 检测结果数组
    int              count;         // 检测数量
    int64_t          frame_id;      // 对应的帧 ID
    int64_t          elapsed_us;    // 单帧处理耗时 (微秒)
} cv_result_t;

/* ---------- 回调类型 ---------- */

// 帧处理完成回调 (在 cv_branch 线程中调用，不要做耗时操作)
typedef void (*cv_on_result_t)(const cv_result_t *result, void *user_data);

/* ---------- 配置 ---------- */
typedef struct {
    int             max_queue_size;     // 最大缓存帧数 (默认 2, 超出的帧丢弃)
    int             processing_width;   // AI 处理分辨率宽 (0=保持原图, 用于降分辨率加速)
    int             processing_height;  // AI 处理分辨率高
    cv_on_result_t  on_result;          // 结果回调
    void           *user_data;          // 回调透传参数
    const char     *model_path;         // 模型文件路径 (如 .rknn 或 .onnx)
} cv_branch_config_t;

/* ---------- API ---------- */

/**
 * 初始化 OpenCV 分支
 * @param cfg  配置参数，NULL 则使用默认值
 * @return 0 成功, -1 失败
 */
int  cv_branch_init(const cv_branch_config_t *cfg);

/**
 * 推送一帧到 OpenCV 处理队列 (非阻塞, 线程安全)
 *
 * 调用方在 push 之后即可释放或复用帧数据 —
 * cv_branch 内部会做一份拷贝或引用计数保护。
 *
 * @param frame  帧数据 (不会被修改)
 * @return 0 成功入队, -1 队列满已丢弃, -2 未初始化
 */
int  cv_branch_push_frame(const cv_frame_t *frame);

/**
 * 获取 ck_branch` 运行状态
 * @return 1 运行中, 0 已停止/未初始化
 */
int  cv_branch_is_running(void);

/**
 * 获取统计信息 (调试用)
 * @param total_in   累计接收帧数
 * @param total_out  累计处理帧数
 * @param total_drop 累计丢弃帧数
 */
void cv_branch_get_stats(int64_t *total_in, int64_t *total_out, int64_t *total_drop);

/**
 * 获取最近一次 AI 处理后的标注 JPEG 帧 (线程安全, 非阻塞)
 *
 * 在 GStreamer 推流前调用: 如果 AI 产出了标注帧就推标注帧, 否则推原始帧.
 * 这样 RTSP 拉流就能看到检测框.
 *
 * @param out_size  输出 JPEG 数据长度 (仅当返回值非 NULL 时有效)
 * @return JPEG 数据指针 (cv_branch 内部管理, 调用方不应 free),
 *         无新帧时返回 NULL
 */
const uint8_t *cv_branch_get_annotated_frame(size_t *out_size);

/**
 * 销毁 OpenCV 分支，释放资源
 */
void cv_branch_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CV_BRANCH_H
