/**
 * cv_branch.c — OpenCV AI 数据分支实现
 *
 * 数据流:
 *   push_frame() → [ring queue (leaky)] → process_thread() → cv::Mat → AI推理 → callback
 *
 * 队列策略: leaky ring buffer
 *   - 写入: 队列满时覆盖最旧的帧 (总是不阻塞)
 *   - 读取: 取最新帧处理
 *   保证: 采集线程永远不会被 AI 处理阻塞, AI 始终处理最新帧
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include "ai_processor.h"
#include "voice_service.h"
#include "tcp_server.h"

/* ---- SD卡日志 ---- */
#define LOG_PATH "/userdata/alarm_log.txt"
#define LOG_MAX_SIZE (1024*1024)  /* 1MB 轮转 */

static void write_detection_log(int has_target, int class_id, float conf,
    float tvoc, float ch2o, int co2, int alarm, float temp,
    float smoke_score, float fire_score)
{
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);
    char buf[512];
    int off = snprintf(buf, sizeof(buf),
        "[%04d-%02d-%02d %02d:%02d:%02d] ",
        t.tm_year+1900, t.tm_mon+1, t.tm_mday,
        t.tm_hour, t.tm_min, t.tm_sec);
    if (has_target) {
        static const char *names[] = {"no_smk","smoking","fire","smoke"};
        const char *nm = (class_id>=0 && class_id<=3) ? names[class_id] : "?";
        off += snprintf(buf+off, sizeof(buf)-off,
            "%s(%.2f) | ", nm, conf);
    } else {
        off += snprintf(buf+off, sizeof(buf)-off, "no_det | ");
    }
    snprintf(buf+off, sizeof(buf)-off,
        "TVOC=%.3f CH2O=%.3f CO2=%d temp=%.1f°C alarm=%d | "
        "smoke=%.1f fire=%.1f\n",
        tvoc, ch2o, co2, temp, alarm, smoke_score, fire_score);

    /* 追加写入, 超过1MB轮转 */
    FILE *fp = fopen(LOG_PATH, "a");
    if (!fp) return;
    fseek(fp, 0, SEEK_END);
    if (ftell(fp) > LOG_MAX_SIZE) {
        fclose(fp);
        fp = fopen(LOG_PATH, "w");
        if (!fp) return;
    }
    fputs(buf, fp);
    fclose(fp);
}

/* ============================================================
 *  内部常量
 * ============================================================ */
#define DEFAULT_QUEUE_SIZE      8
#define MAX_DETECTIONS          64

/* ---- 语音报警 (状态机) ----
 *
 * 状态流转:
 *   IDLE(0) → 4s内检测≥80% → voice1 → STAGE1(1)
 *   STAGE1   → 4s后检测≥80% → voice2 → STAGE2(2)
 *   STAGE2   → 4s后检测≥80% → voice3 → STAGE3(3)
 *   STAGE3   → 每2s检测≥80% → voice3 (重复)
 *   任意阶段 → 15s无检测 → 重置IDLE
 * ---------------------------------------- */
#define VOICE_HISTORY_MAX   128
#define VOICE_STAGE1_SEC      4   // voice1→voice2 间隔 (秒)
#define VOICE_STAGE2_SEC      8   // voice2→voice3 间隔 (秒)
#define VOICE_REPEAT_SEC      2   // voice3 重复间隔 (秒)
#define VOICE_RESET_SEC       5   // 低评分持续此秒数 → 重置 (秒)
#define VOICE_TH1             6.5 // 吸烟评分阈值 (疑似)
#define VOICE_TH2             6.5 // 吸烟评分阈值 (报警)
#define FIRE_TH1              5.5 // 火灾评分阈值 (疑似)
#define FIRE_TH2              6.5 // 火灾评分阈值 (报警)
#define TEMP_FIRE_THRESHOLD   80.0 // 温度超过此值直接判定火灾 (°C)

#define VOICE_PATH "/userdata/my_projects/voicedate"

static struct {
    double  history[VOICE_HISTORY_MAX];
    int     h_idx, h_count;
    double  fire_history[VOICE_HISTORY_MAX];  // 火灾评分历史
    int     fire_h_idx, fire_h_count;
    int     voice_playing;         // 是否正在播放
    int     stage;                 // 0=IDLE 1=voice1已播 2=voice2已播 3=voice3阶段
    time_t  stage_start;           // 当前阶段开始时间
    time_t  last_detect;           // 最后一次检测到目标的时间
    time_t  last_fire_detect;      // 最后一次检测到火灾的时间
    pthread_mutex_t lock;
} g_voice = { .lock = PTHREAD_MUTEX_INITIALIZER };

static void *voice_thread(void *arg)
{
    char *file = (char *)arg;
    play_voice(file);
    free(file);
    g_voice.voice_playing = 0;
    g_voice.stage_start = time(NULL);  // 播放结束后才重新计时
    return NULL;
}

static void voice_play_async(const char *file)
{
    if (g_voice.voice_playing) return;
    g_voice.voice_playing = 1;
    pthread_t tid;
    char *name = strdup(file);
    if (pthread_create(&tid, NULL, voice_thread, name) != 0) {
        g_voice.voice_playing = 0;
        free(name);
    } else {
        pthread_detach(tid);
    }
}

/* 存储吸烟评分数值到历史 */
static void voice_push_score(float score)
{
    pthread_mutex_lock(&g_voice.lock);
    g_voice.history[g_voice.h_idx] = score;
    g_voice.h_idx = (g_voice.h_idx + 1) % VOICE_HISTORY_MAX;
    if (g_voice.h_count < VOICE_HISTORY_MAX) g_voice.h_count++;
    if (score >= VOICE_TH1) g_voice.last_detect = time(NULL);
    pthread_mutex_unlock(&g_voice.lock);
}

/* 存储火灾评分数值到历史 */
static void voice_push_fire_score(float score)
{
    pthread_mutex_lock(&g_voice.lock);
    g_voice.fire_history[g_voice.fire_h_idx] = score;
    g_voice.fire_h_idx = (g_voice.fire_h_idx + 1) % VOICE_HISTORY_MAX;
    if (g_voice.fire_h_count < VOICE_HISTORY_MAX) g_voice.fire_h_count++;
    if (score >= FIRE_TH1) g_voice.last_fire_detect = time(NULL);
    pthread_mutex_unlock(&g_voice.lock);
}

/* 计算最近N帧平均评分 */
static float voice_avg_score(int frames)
{
    if (frames <= 0 || frames > g_voice.h_count) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < frames; i++) {
        int idx = (g_voice.h_idx - 1 - i + VOICE_HISTORY_MAX) % VOICE_HISTORY_MAX;
        sum += g_voice.history[idx];
    }
    return sum / frames;
}

/* 计算最近N帧火灾平均评分 */
static float voice_avg_fire_score(int frames)
{
    if (frames <= 0 || frames > g_voice.fire_h_count) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < frames; i++) {
        int idx = (g_voice.fire_h_idx - 1 - i + VOICE_HISTORY_MAX) % VOICE_HISTORY_MAX;
        sum += g_voice.fire_history[idx];
    }
    return sum / frames;
}

/* 计算最近N帧中评分>=阈值占比 (0.0~1.0) */
static float voice_fire_ratio(int frames, float threshold)
{
    if (frames <= 0 || frames > g_voice.fire_h_count) return 0.0f;
    int count = 0;
    for (int i = 0; i < frames; i++) {
        int idx = (g_voice.fire_h_idx - 1 - i + VOICE_HISTORY_MAX) % VOICE_HISTORY_MAX;
        if (g_voice.fire_history[idx] >= threshold) count++;
    }
    return (float)count / frames;
}

/* 计算最近N帧吸烟评分占比 */
static float voice_smoke_ratio(int frames, float threshold)
{
    if (frames <= 0 || frames > g_voice.h_count) return 0.0f;
    int count = 0;
    for (int i = 0; i < frames; i++) {
        int idx = (g_voice.h_idx - 1 - i + VOICE_HISTORY_MAX) % VOICE_HISTORY_MAX;
        if (g_voice.history[idx] >= threshold) count++;
    }
    return (float)count / frames;
}

    /* 占比阈值 (80%) */
#define SMOKE_RATIO_TRIGGER    0.8f
#define FIRE_RATIO_TRIGGER     0.8f
#define FIRE_RATIO_RESET       0.8f

static void voice_check_trigger(void)
{
    time_t now = time(NULL);

    int win = 28;  /* 4秒窗口 */

    /* ---- 吸烟检测: 占比≥80% ---- */
    float smoke_ratio = voice_smoke_ratio(win, VOICE_TH1);
    int detection_ok = (smoke_ratio >= SMOKE_RATIO_TRIGGER && g_voice.h_count >= win);

    /* 重置: 占比不足持续超时 */
    if (!detection_ok && g_voice.stage > 0 &&
        now - g_voice.last_detect > VOICE_RESET_SEC) {
        printf("[VOICE] smoke_ratio=%.0f%% < %.0f%%, 重置 (stage %d→0)\n",
               smoke_ratio * 100, SMOKE_RATIO_TRIGGER * 100, g_voice.stage);
        g_voice.stage = 0;
        g_voice.stage_start = 0;
    }

    /* ---- 火灾检测: 占比≥80% (独立状态机) ---- */
    int fire_win = 28;  /* 4秒窗口 */
    float fire_ratio = voice_fire_ratio(fire_win, FIRE_TH1);  // 单帧≥5.5即计数
    int fire_detection_ok = (fire_ratio >= FIRE_RATIO_TRIGGER && g_voice.fire_h_count >= fire_win);
    static int fire_active = 0;
    static int fire_voice_was_playing = 0;
    static time_t fire_voice_end = 0;

    /* 温度直接判定: ≥80°C 立即触发火灾 */
    float _, __, temp; int ___;
    ai_fifo_get_sensor(&_, &__, &___, &___, &temp);
    if (temp >= TEMP_FIRE_THRESHOLD) {
        if (!fire_active) printf("[VOICE] FIRE by temperature! (%.1f°C ≥ %.0f°C)\n", temp, TEMP_FIRE_THRESHOLD);
        fire_active = 1;
    }

    /* 触发: 窗口内≥80%帧 fire_score >= 5.5 */
    else if (fire_detection_ok) {
        fire_active = 1;
    }
    /* 取消: 窗口内<80%帧 fire_score >= 5.5 持续5秒 */
    if (fire_active && !fire_detection_ok &&
        now - g_voice.last_fire_detect > VOICE_RESET_SEC) {
        fire_active = 0;
        printf("[VOICE] FIRE alarm reset (fire_ratio=%.0f%% < %.0f%%)\n",
               fire_ratio * 100, FIRE_RATIO_RESET * 100);
    }

    /* 检测火灾语音播放结束 (voice_playing 1→0 转换) */
    if (fire_voice_was_playing && !g_voice.voice_playing) {
        fire_voice_end = time(NULL);
        printf("[VOICE] FIRE voice4 finished, next in 2s\n");
    }
    fire_voice_was_playing = (g_voice.voice_playing && fire_active);

    /* 火灾语音循环 */
    if (fire_active && !g_voice.voice_playing &&
        now - fire_voice_end >= 2) {
        printf("[VOICE] FIRE alarm! (fire_ratio=%.0f%% TH1=%.1f) → voice4\n",
               fire_ratio * 100, FIRE_TH1);
        voice_play_async(VOICE_PATH "/voice4.wav");
        fire_voice_was_playing = 1;
    }

    if (g_voice.voice_playing) return;

    switch (g_voice.stage) {

    case 0: /* 占比≥80% → voice1 */
        if (smoke_ratio >= SMOKE_RATIO_TRIGGER) {
            printf("[VOICE] voice1 trigger (smoke_ratio=%.0f%% TH1=%.1f)\n",
                   smoke_ratio * 100, VOICE_TH1);
            voice_play_async(VOICE_PATH "/voice1.wav");
            g_voice.stage = 1;
            g_voice.stage_start = now;
        }
        break;

    case 1: /* 4s后 → voice2 */
        if (now - g_voice.stage_start >= VOICE_STAGE1_SEC) {
            printf("[VOICE] voice2 trigger (elapsed=%llds)\n",
                   (long long)(now - g_voice.stage_start));
            voice_play_async(VOICE_PATH "/voice2.wav");
            g_voice.stage = 2;
            g_voice.stage_start = now;
        }
        break;

    case 2: /* 8s后 → voice3 */
        if (now - g_voice.stage_start >= VOICE_STAGE2_SEC) {
            printf("[VOICE] voice3 trigger (elapsed=%llds)\n",
                   (long long)(now - g_voice.stage_start));
            voice_play_async(VOICE_PATH "/voice3.wav");
            g_voice.stage = 3;
            g_voice.stage_start = now;
        }
        break;

    case 3: /* 每2s重复voice3 */
        if (now - g_voice.stage_start >= VOICE_REPEAT_SEC) {
            printf("[VOICE] voice3 repeat\n");
            voice_play_async(VOICE_PATH "/voice3.wav");
            g_voice.stage_start = now;
        }
        break;
    }
}

/* ============================================================
 *  C/C++ 兼容的原子操作 (GCC builtins)
 * ============================================================ */
// __ATOMIC_RELAXED 足够, 因为统计不需要严格序,
// running/write_idx 有 mutex 配合保证一致性.
#define ATOMIC_LOAD(p)     __atomic_load_n((p), __ATOMIC_RELAXED)
#define ATOMIC_STORE(p,v)  __atomic_store_n((p), (v), __ATOMIC_RELAXED)
#define ATOMIC_ADD(p,v)    __atomic_fetch_add((p), (v), __ATOMIC_RELAXED)

/* ============================================================
 *  全局状态
 * ============================================================ */
static struct {
    /* 初始标志 */
    int   initialized;

    /* 配置 */
    cv_branch_config_t cfg;

    /* 处理线程 */
    pthread_t  thread;
    int        running;       // 原子访问 via ATOMIC_*

    /* 帧队列 — leaky ring buffer */
    cv_frame_t    *ring;
    int            ring_size;
    int            write_idx;  // 原子访问 via ATOMIC_* (生产者)
    int            read_idx;   // 仅在消费者线程访问
    pthread_mutex_t lock;
    pthread_cond_t  cond;

    /* 统计 */
    int64_t  total_in;        // 原子访问 via ATOMIC_*
    int64_t  total_out;
    int64_t  total_drop;

    /* 标注帧输出缓冲 (线程安全) */
    uint8_t        *out_jpeg;         // 最新标注 JPEG 数据
    size_t          out_jpeg_size;
    int64_t         out_jpeg_id;      // 对应的帧 ID, -1 = 无新帧
    pthread_mutex_t out_lock;         // 保护 out_jpeg / out_jpeg_size / out_jpeg_id

} g_cv = {0};

/* ============================================================
 *  内部: 帧拷贝 & 释放
 * ============================================================ */
static void free_frame_content(cv_frame_t *f)
{
    if (f->data) {
        free(f->data);
        f->data = NULL;
    }
    f->size = 0;
}

static int copy_frame(cv_frame_t *dst, const cv_frame_t *src)
{
    dst->width        = src->width;
    dst->height       = src->height;
    dst->stride       = src->stride;
    dst->format       = src->format;
    dst->timestamp_us = src->timestamp_us;
    dst->frame_id     = src->frame_id;
    dst->size         = src->size;

    dst->data = (uint8_t *)malloc(src->size);
    if (!dst->data) {
        fprintf(stderr, "[CV] malloc frame(%zu) failed\n", src->size);
        return -1;
    }
    memcpy(dst->data, src->data, src->size);
    return 0;
}

/* ============================================================
 *  内部: 默认结果回调 (空实现)
 * ============================================================ */
static void default_on_result(const cv_result_t *result, void *user_data)
{
    (void)result;
    (void)user_data;
    // 默认不打印, 用户可通过注册回调来处理
}

/* ============================================================
 *  辅助: 获取当前时间 (微秒)
 * ============================================================ */
#include <time.h>
static int64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

/* ============================================================
 *  OpenCV 处理模块 (条件编译)
 *  - 未启用时: 透传帧信息, 回调中 count=0
 *  - 启用时:   MJPEG→BGR 解码 → 降采样 → AI 推理 → 回调
 * ============================================================ */
#ifdef CV_BRANCH_HAS_OPENCV

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "rknn_infer.h"
#include "tcp_server.h"
#include "rtsp_stream.h"
/* MPP硬解码参考: vpu_decode.h (需与GStreamer MPP管线协调) */

/* ---- 帧格式 → cv::Mat (BGR) ---- */
static int frame_to_bgr(const cv_frame_t *frame, cv::Mat &mat)
{
    if (frame->format == CV_FMT_JPEG) {
        if (!frame->data || frame->size == 0) return -1;
        try {
            std::vector<uint8_t> jpeg_buf(frame->data, frame->data + frame->size);
            mat = cv::imdecode(jpeg_buf, cv::IMREAD_COLOR);  // 全分辨率 1920x1080
            return mat.empty() ? -1 : 0;
        } catch (const cv::Exception &e) {
            fprintf(stderr, "[CV] imdecode error: %s\n", e.what());
            return -1;
        }
    }
    if (frame->format == CV_FMT_BGR888) {
        mat = cv::Mat(frame->height, frame->width, CV_8UC3,
                      frame->data, frame->stride > 0 ? frame->stride : frame->width * 3);
        return 0;
    }
    if (frame->format == CV_FMT_GRAY8) {
        cv::Mat gray(frame->height, frame->width, CV_8UC1,
                     frame->data, frame->stride > 0 ? frame->stride : frame->width);
        cv::cvtColor(gray, mat, cv::COLOR_GRAY2BGR);
        return 0;
    }
    fprintf(stderr, "[CV] unsupported pixel format: %d\n", frame->format);
    return -1;
}

/* ---- 框平滑: EMA (旧框×0.7 + 新框×0.3), 消除检测框抖动 ---- */
#define SMOOTH_ALPHA 0.3f  /* 新框权重, 越小越平滑 */
#define SMOOTH_MIN_IOU 0.3f /* IoU低于此值视为不同目标, 不匹配 */

static float box_iou(const rknn_detection_t *a, const rknn_detection_t *b)
{
    int x1 = (a->x > b->x) ? a->x : b->x;
    int y1 = (a->y > b->y) ? a->y : b->y;
    int x2 = ((a->x + a->w) < (b->x + b->w)) ? (a->x + a->w) : (b->x + b->w);
    int y2 = ((a->y + a->h) < (b->y + b->h)) ? (a->y + a->h) : (b->y + b->h);
    if (x2 <= x1 || y2 <= y1) return 0.0f;
    float inter = (float)(x2 - x1) * (y2 - y1);
    float area_a = (float)a->w * a->h, area_b = (float)b->w * b->h;
    return inter / (area_a + area_b - inter + 1e-6f);
}

static void smooth_detections(rknn_result_t *res)
{
    static rknn_detection_t prev[20];
    static int prev_cnt = 0;

    for (int i = 0; i < res->count && i < 20; i++) {
        /* 找与当前框最匹配的上一帧框 (同类别 + 最大IoU) */
        int best_j = -1; float best_iou = SMOOTH_MIN_IOU;
        for (int j = 0; j < prev_cnt; j++) {
            if (res->detections[i].class_id != prev[j].class_id) continue;
            float iou_val = box_iou(&res->detections[i], &prev[j]);
            if (iou_val > best_iou) { best_iou = iou_val; best_j = j; }
        }
        if (best_j >= 0) {
            /* EMA 平滑: x = old*0.7 + new*0.3 */
            rknn_detection_t *d = &res->detections[i];
            rknn_detection_t *p = &prev[best_j];
            d->x = (int)(p->x * (1.0f - SMOOTH_ALPHA) + d->x * SMOOTH_ALPHA);
            d->y = (int)(p->y * (1.0f - SMOOTH_ALPHA) + d->y * SMOOTH_ALPHA);
            d->w = (int)(p->w * (1.0f - SMOOTH_ALPHA) + d->w * SMOOTH_ALPHA);
            d->h = (int)(p->h * (1.0f - SMOOTH_ALPHA) + d->h * SMOOTH_ALPHA);
            d->confidence = p->confidence * (1.0f - SMOOTH_ALPHA) + d->confidence * SMOOTH_ALPHA;
        }
    }
    /* 保存当前帧作为下一帧的参考 */
    prev_cnt = res->count;
    for (int i = 0; i < res->count && i < 20; i++)
        prev[i] = res->detections[i];
}

/* ---- 画检测框 (在 BGR 图像上) ---- */
static void draw_detections(cv::Mat &mat, rknn_result_t *rknn_res)
{
    const cv::Scalar colors[] = {
        cv::Scalar(0, 255, 0),    // 绿
        cv::Scalar(255, 0, 0),    // 蓝
        cv::Scalar(0, 0, 255),    // 红
        cv::Scalar(255, 255, 0),  // 青
        cv::Scalar(255, 0, 255),  // 紫
    };
    const int n_colors = sizeof(colors) / sizeof(colors[0]);

    for (int i = 0; i < rknn_res->count && i < 64; i++) {
        rknn_detection_t *d = &rknn_res->detections[i];
        cv::Scalar color = colors[d->class_id % n_colors];

        // 边界框
        cv::rectangle(mat,
                      cv::Point(d->x, d->y),
                      cv::Point(d->x + d->w, d->y + d->h),
                      color, 2);

        // 标签 + 置信度
        char label[128];
        snprintf(label, sizeof(label), "%s %.2f",
                 d->label, d->confidence);
        cv::putText(mat, label,
                    cv::Point(d->x, d->y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }
}

/* ---- 单帧处理 (OpenCV + RKNN NPU) ---- */
static int64_t micros(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

static void process_one_frame_cv(const cv_frame_t *frame)
{
    cv_result_t result;
    memset(&result, 0, sizeof(result));
    result.frame_id = frame->frame_id;
    int64_t t0 = micros(), t1;

    // 1. MJPEG → BGR 解码
    cv::Mat mat_full;
    if (frame_to_bgr(frame, mat_full) != 0) {
        if (g_cv.cfg.on_result)
            g_cv.cfg.on_result(&result, g_cv.cfg.user_data);
        return;
    }
    t1 = micros();

    // 2. RKNN NPU 推理 (preprocess 内部 resize+cvtColor, 坐标直接从模型空间→原图)
    int64_t t2 = micros();
    rknn_result_t rknn_res;
    int has_detection = 0;
    int64_t t3 = 0;
    if (rknn_infer_run(mat_full.data, mat_full.cols, mat_full.rows, &rknn_res) == 0) {
        has_detection = 1;
        t3 = micros();

        // 坐标已由 rescale_boxes 映射到原图, 无需二次缩放
        // EMA平滑消抖, 然后在原始分辨率图像上画框
        smooth_detections(&rknn_res);
        draw_detections(mat_full, &rknn_res);

        // 回调 + 坐标
        result.count       = rknn_res.count;
        result.elapsed_us  = rknn_res.elapsed_us;
        result.detections  = (cv_detection_t *)calloc(rknn_res.count,
                                                       sizeof(cv_detection_t));
        for (int i = 0; i < rknn_res.count; i++) {
            cv_detection_t *d = &result.detections[i];
            d->class_id   = rknn_res.detections[i].class_id;
            d->confidence = rknn_res.detections[i].confidence;
            d->x = rknn_res.detections[i].x;
            d->y = rknn_res.detections[i].y;
            d->w = rknn_res.detections[i].w;
            d->h = rknn_res.detections[i].h;
            snprintf(d->label, sizeof(d->label), "%s",
                     rknn_res.detections[i].label);
        }
        float fire_score_voice = 0.0f;  /* 火灾语音评分 (跨分支) */
        if (rknn_res.count > 0) {
            rknn_detection_t *best = &rknn_res.detections[0];
            /* 计算评分 */
            float sk_conf = 0, sc_conf = 0, fr_conf = 0;
            for (int i = 0; i < rknn_res.count; i++) {
                if (rknn_res.detections[i].class_id == 1)        // smoking
                    sk_conf = fmaxf(sk_conf, rknn_res.detections[i].confidence);
                else if (rknn_res.detections[i].class_id == 2)   // fire
                    fr_conf = fmaxf(fr_conf, rknn_res.detections[i].confidence);
                else if (rknn_res.detections[i].class_id == 3)   // smoke
                    sc_conf = fmaxf(sc_conf, rknn_res.detections[i].confidence);
            }
            float tvoc, ch2o; int co2, alarm;
            ai_fifo_get_sensor(&tvoc, &ch2o, &co2, &alarm, NULL);
            float sm = sk_conf * 8.0f + sc_conf * 0.5f + (ch2o/1.0f)*0.6f + (tvoc/2.0f)*0.5f + (co2/2000.0f)*0.4f;
            float fm = fr_conf * 7.0f + sc_conf * 0.8f + (tvoc/2.0f)*1.0f + (co2/2000.0f)*0.7f + (ch2o/1.0f)*0.5f;
            fire_score_voice = fm;
            /* 过滤no_smoking: 仅推送吸烟/火灾/烟雾 */
            if (best->class_id != 0)
                ai_fifo_send(best->x + best->w/2, best->y + best->h/2,
                              best->w, best->h,
                              mat_full.cols, mat_full.rows, best->confidence, 1,
                              best->class_id, sm, fm);
            else
                ai_fifo_send(0, 0, 0, 0, mat_full.cols, mat_full.rows, 0.0f, 0, -1, sm, fm);
        } else {
            /* 无检测目标 → 清零画框, 但仍发送传感器评分 */
            float tvoc, ch2o; int co2, alarm;
            ai_fifo_get_sensor(&tvoc, &ch2o, &co2, &alarm, NULL);
            float sm = (ch2o/1.0f)*0.6f + (tvoc/2.0f)*0.5f + (co2/2000.0f)*0.4f;
            float fm = (tvoc/2.0f)*1.0f + (co2/2000.0f)*0.7f + (ch2o/1.0f)*0.5f;
            fire_score_voice = fm;
            ai_fifo_send(0, 0, 0, 0, mat_full.cols, mat_full.rows, 0.0f, 0, -1, sm, fm);
        }
        /* 语音: 计算吸烟评分 */
        float smoke_score = 0.0f;
        for (int i = 0; i < rknn_res.count; i++) {
            if (rknn_res.detections[i].class_id == 1)   // smoking
                smoke_score += rknn_res.detections[i].confidence * 8.0f;
            else if (rknn_res.detections[i].class_id == 3)  // smoke
                smoke_score += rknn_res.detections[i].confidence * 0.5f;
        }
        float tvoc, ch2o; int co2, alarm;
        ai_fifo_get_sensor(&tvoc, &ch2o, &co2, &alarm, NULL);
        smoke_score += (ch2o / 1.0f) * 0.6f + (tvoc / 2.0f) * 0.5f + (co2 / 2000.0f) * 0.4f;
        free(rknn_res.detections);

        voice_push_score(smoke_score);
        voice_push_fire_score(fire_score_voice);
        voice_check_trigger();

        /* 日志: 含检测结果 */
        {
            float _t, _c; int _co, _al; float _temp;
            ai_fifo_get_sensor(&_t, &_c, &_co, &_al, &_temp);
            int cid = (rknn_res.count > 0) ? rknn_res.detections[0].class_id : -1;
            float conf = (rknn_res.count > 0) ? rknn_res.detections[0].confidence : 0.0f;
            write_detection_log((rknn_res.count > 0), cid, conf,
                _t, _c, _co, _al, _temp, smoke_score, fire_score_voice);
        }
    } else {
        /* 推理失败 → 清零画框, 仍发送传感器评分 */
        float tvoc, ch2o; int co2, alarm;
        float temp;
        ai_fifo_get_sensor(&tvoc, &ch2o, &co2, &alarm, &temp);
        float sm = (ch2o/1.0f)*0.6f + (tvoc/2.0f)*0.5f + (co2/2000.0f)*0.4f;
        float fm = (tvoc/2.0f)*1.0f + (co2/2000.0f)*0.7f + (ch2o/1.0f)*0.5f;
        ai_fifo_send(0, 0, 0, 0, frame->width, frame->height, 0.0f, 0, -1, sm, fm);
        voice_push_score(0.0f);
        voice_push_fire_score(fm);
        voice_check_trigger();
        write_detection_log(0, -1, 0.0f,
            tvoc, ch2o, co2, alarm, temp, sm, fm);
    }

    // 3. 始终编码 JPEG (有检测画框, 无检测原图)
    {
        std::vector<uchar> jpeg_buf;
        std::vector<int>  jpeg_params = { cv::IMWRITE_JPEG_QUALITY, 60 };
        cv::imencode(".jpg", mat_full, jpeg_buf, jpeg_params);
        pthread_mutex_lock(&g_cv.out_lock);
        free(g_cv.out_jpeg);
        g_cv.out_jpeg      = (uint8_t *)malloc(jpeg_buf.size());
        g_cv.out_jpeg_size = jpeg_buf.size();
        g_cv.out_jpeg_id   = frame->frame_id;
        if (g_cv.out_jpeg) memcpy(g_cv.out_jpeg, jpeg_buf.data(), jpeg_buf.size());
        pthread_mutex_unlock(&g_cv.out_lock);
    }
    int64_t t4 = micros();

    /* 每100帧打印一次阶段耗时分布 */
    static int perf_cnt = 0;
    if (++perf_cnt % 100 == 0 && t3 > 0) {
        printf("[PERF] decode=%lldms rknn=%lldms encode=%lldms total=%lldms\n",
               (long long)(t1-t0)/1000,
               (long long)(t3-t2)/1000, (long long)(t4-t3)/1000,
               (long long)(t4-t0)/1000);
    }

    if (g_cv.cfg.on_result)
        g_cv.cfg.on_result(&result, g_cv.cfg.user_data);

    free(result.detections);
}

#else // !CV_BRANCH_HAS_OPENCV

/* ---- 单帧处理 (无 OpenCV: 空实现) ---- */
static void process_one_frame_cv(const cv_frame_t *frame)
{
    cv_result_t result;
    memset(&result, 0, sizeof(result));
    result.frame_id = frame->frame_id;
    (void)frame;

    if (g_cv.cfg.on_result)
        g_cv.cfg.on_result(&result, g_cv.cfg.user_data);
}

#endif // CV_BRANCH_HAS_OPENCV

/* ============================================================
 *  内部: 单帧处理入口 (统一)
 * ============================================================ */
static void process_one_frame(const cv_frame_t *frame)
{
    process_one_frame_cv(frame);
}

/* ============================================================
 *  内部: 处理线程主循环
 * ============================================================ */
static void *process_thread(void *arg)
{
    (void)arg;
    printf("[CV] Process thread started\n");

    while (ATOMIC_LOAD(&g_cv.running)) {
        /* ---- 等待新帧 ---- */
        pthread_mutex_lock(&g_cv.lock);

        // 等待队列非空 (write_idx != read_idx)
        while (ATOMIC_LOAD(&g_cv.write_idx) == g_cv.read_idx &&
               ATOMIC_LOAD(&g_cv.running)) {
            pthread_cond_wait(&g_cv.cond, &g_cv.lock);
        }

        if (!ATOMIC_LOAD(&g_cv.running)) {
            pthread_mutex_unlock(&g_cv.lock);
            break;
        }

        // 取出待处理的帧索引, 并清空该槽位所有权 (消费方独占)
        int idx = g_cv.read_idx;
        pthread_mutex_unlock(&g_cv.lock);

        /* ---- 处理帧 ---- */
        process_one_frame(&g_cv.ring[idx]);

        ATOMIC_ADD(&g_cv.total_out, 1);

        /* ---- 释放该槽帧数据 ---- */
        pthread_mutex_lock(&g_cv.lock);
        free_frame_content(&g_cv.ring[idx]);
        g_cv.read_idx = (g_cv.read_idx + 1) % g_cv.ring_size;
        pthread_mutex_unlock(&g_cv.lock);
    }

    printf("[CV] Process thread stopped\n");
    return NULL;
}

/* ============================================================
 *  API 实现
 * ============================================================ */

int cv_branch_init(const cv_branch_config_t *cfg)
{
    if (g_cv.initialized) {
        fprintf(stderr, "[CV] Already initialized\n");
        return -1;
    }

    memset(&g_cv, 0, sizeof(g_cv));

    /* ---- 拷贝配置 ---- */
    if (cfg) {
        g_cv.cfg = *cfg;
    }
    if (g_cv.cfg.max_queue_size <= 0) {
        g_cv.cfg.max_queue_size = DEFAULT_QUEUE_SIZE;
    }
    if (!g_cv.cfg.on_result) {
        g_cv.cfg.on_result = default_on_result;
    }

    /* ---- 分配环形缓冲区 ---- */
    g_cv.ring_size = g_cv.cfg.max_queue_size + 1;  // +1 区分空/满
    g_cv.ring = (cv_frame_t *)calloc(g_cv.ring_size, sizeof(cv_frame_t));
    if (!g_cv.ring) {
        fprintf(stderr, "[CV] ring alloc failed\n");
        return -1;
    }
    g_cv.write_idx = 0;
    g_cv.read_idx  = 0;

    /* ---- 同步原语 ---- */
    if (pthread_mutex_init(&g_cv.lock, NULL) != 0) {
        perror("[CV] mutex_init");
        free(g_cv.ring);
        g_cv.ring = NULL;
        return -1;
    }
    if (pthread_cond_init(&g_cv.cond, NULL) != 0) {
        perror("[CV] cond_init");
        pthread_mutex_destroy(&g_cv.lock);
        free(g_cv.ring);
        g_cv.ring = NULL;
        return -1;
    }
    if (pthread_mutex_init(&g_cv.out_lock, NULL) != 0) {
        perror("[CV] out_lock_init");
        pthread_cond_destroy(&g_cv.cond);
        pthread_mutex_destroy(&g_cv.lock);
        free(g_cv.ring);
        g_cv.ring = NULL;
        return -1;
    }

    /* ---- 初始化 RKNN 推理 ---- */
#ifdef CV_BRANCH_HAS_OPENCV
    if (g_cv.cfg.model_path && g_cv.cfg.model_path[0]) {
        if (rknn_infer_init(g_cv.cfg.model_path) != 0) {
            fprintf(stderr, "[CV] WARN: RKNN init failed, AI disabled\n");
            // 不致命, 继续运行但不做推理
        }
    }
#endif

    /* ---- 启动处理线程 ---- */
    ATOMIC_STORE(&g_cv.running, 1);
    if (pthread_create(&g_cv.thread, NULL, process_thread, NULL) != 0) {
        perror("[CV] pthread_create");
        pthread_cond_destroy(&g_cv.cond);
        pthread_mutex_destroy(&g_cv.lock);
        free(g_cv.ring);
        g_cv.ring = NULL;
        return -1;
    }

    g_cv.initialized = 1;
    printf("[CV] Initialized OK (queue_size=%d, ring_size=%d)\n",
           g_cv.cfg.max_queue_size, g_cv.ring_size);
    return 0;
}

int cv_branch_push_frame(const cv_frame_t *frame)
{
    if (!g_cv.initialized || !ATOMIC_LOAD(&g_cv.running)) {
        return -2;
    }

    ATOMIC_ADD(&g_cv.total_in, 1);

    // 按像素格式估算最小大小
    size_t min_size = frame->size;
    if (min_size == 0) {
        // 回退估算
        int bpp = 3; // 默认 BGR
        switch (frame->format) {
            case CV_FMT_JPEG:  bpp = 0; break; // JPEG 不适用
            case CV_FMT_GRAY8: bpp = 1; break;
            case CV_FMT_NV12:
            case CV_FMT_NV21:  bpp = 1; min_size = frame->width * frame->height * 3 / 2; break;
            case CV_FMT_YUYV:  bpp = 2; break;
            default:           bpp = 3; break;
        }
        if (bpp > 0) min_size = frame->width * frame->height * bpp;
    }

    pthread_mutex_lock(&g_cv.lock);

    int w_idx = ATOMIC_LOAD(&g_cv.write_idx);
    int next_w = (w_idx + 1) % g_cv.ring_size;

    if (next_w == g_cv.read_idx) {
        // 队列满: drop 最旧帧, 移动 read_idx
        int drop_idx = g_cv.read_idx;
        free_frame_content(&g_cv.ring[drop_idx]);
        g_cv.read_idx = (g_cv.read_idx + 1) % g_cv.ring_size;
        ATOMIC_ADD(&g_cv.total_drop, 1);
    }

    // 拷贝帧到当前 write 槽
    if (copy_frame(&g_cv.ring[w_idx], frame) != 0) {
        pthread_mutex_unlock(&g_cv.lock);
        return -1;
    }

    ATOMIC_STORE(&g_cv.write_idx, next_w);
    pthread_cond_signal(&g_cv.cond);
    pthread_mutex_unlock(&g_cv.lock);

    return 0;
}

int cv_branch_is_running(void)
{
    return (g_cv.initialized && ATOMIC_LOAD(&g_cv.running)) ? 1 : 0;
}

void cv_branch_get_stats(int64_t *total_in, int64_t *total_out, int64_t *total_drop)
{
    if (total_in)  *total_in  = ATOMIC_LOAD(&g_cv.total_in);
    if (total_out) *total_out = ATOMIC_LOAD(&g_cv.total_out);
    if (total_drop) *total_drop = ATOMIC_LOAD(&g_cv.total_drop);
}

const uint8_t *cv_branch_get_annotated_frame(size_t *out_size)
{
    if (!g_cv.initialized) return NULL;

    pthread_mutex_lock(&g_cv.out_lock);
    if (g_cv.out_jpeg_id >= 0 && g_cv.out_jpeg) {
        *out_size = g_cv.out_jpeg_size;
        uint8_t *ptr = g_cv.out_jpeg;
        g_cv.out_jpeg_id = -1;  // 消费掉, 下次返回 NULL 推原始帧
        pthread_mutex_unlock(&g_cv.out_lock);
        return ptr;
    }
    pthread_mutex_unlock(&g_cv.out_lock);
    *out_size = 0;
    return NULL;
}

void cv_branch_deinit(void)
{
    if (!g_cv.initialized) return;

    printf("[CV] Deinitializing...\n");

    // 1. 通知处理线程退出
    ATOMIC_STORE(&g_cv.running, 0);
    pthread_cond_signal(&g_cv.cond);
    pthread_join(g_cv.thread, NULL);

    // 2. 释放队列中残留的帧
    pthread_mutex_lock(&g_cv.lock);
    while (g_cv.read_idx != ATOMIC_LOAD(&g_cv.write_idx)) {
        free_frame_content(&g_cv.ring[g_cv.read_idx]);
        g_cv.read_idx = (g_cv.read_idx + 1) % g_cv.ring_size;
    }
    pthread_mutex_unlock(&g_cv.lock);

    // 3. 释放输出缓冲
    pthread_mutex_lock(&g_cv.out_lock);
    free(g_cv.out_jpeg);
    g_cv.out_jpeg = NULL;
    pthread_mutex_unlock(&g_cv.out_lock);

    // 3.5 销毁同步对象
    pthread_cond_destroy(&g_cv.cond);
    pthread_mutex_destroy(&g_cv.lock);
    pthread_mutex_destroy(&g_cv.out_lock);

    // 3.5 释放 RKNN 推理
#ifdef CV_BRANCH_HAS_OPENCV
    rknn_infer_deinit();
#endif

    // 4. 释放内存
    free(g_cv.ring);
    g_cv.ring = NULL;
    g_cv.initialized = 0;

    // 5. 打印统计
    printf("[CV] Deinit done. "
           "in=%lld out=%lld drop=%lld\n",
           (long long)ATOMIC_LOAD(&g_cv.total_in),
           (long long)ATOMIC_LOAD(&g_cv.total_out),
           (long long)ATOMIC_LOAD(&g_cv.total_drop));
}
