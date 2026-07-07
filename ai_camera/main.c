/**
 * main.c — RV1126B 智能摄像头 (四线程架构)
 *
 *   capture_thread ──► frame_queue ──► push_thread ──► GStreamer → RTSP
 *                          │
 *                          └──────► ai_thread (cv_branch) → RKNN NPU → console
 *
 *   sensor_thread ──► TVOC/CH2O/CO2 → ai_fifo_set_sensor
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>
#include "rtsp_service.h"
#include "capture.h"
#include "rtsp_stream.h"
#include "ai_processor.h"
#include "tcp_server.h"
#include "sensor.h"
#include "oled_display.h"
#include "recorder.h"
#include "servo_thread.h"
#include "audio_receiver.h"
#include "voice_service.h"

/* ============================================================
 *  配置
 * ============================================================ */
#define CAP_WIDTH    1920
#define CAP_HEIGHT   1080
#define CAP_FPS      30
#define H264_BITRATE 4000000
#define QUEUE_SIZE   4

static volatile int running = 1;
static volatile int g_first_valid_frame = 0;
static pthread_mutex_t g_start_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_start_cond = PTHREAD_COND_INITIALIZER;

/* 标注帧缓存 — 720p带检测框, 推送用 */
static uint8_t        *g_anno_buf = NULL;
static size_t          g_anno_len = 0;
static pthread_mutex_t g_anno_lock = PTHREAD_MUTEX_INITIALIZER;

/* OLED 显示数据 (由 AI 回调和传感器线程更新) */
static struct {
    int   has_target;
    int   class_id;
    float confidence;
} g_oled_detect;

static struct {
    float tvoc;
    int   co2;
    int   alarm;
} g_oled_sensor;

static pthread_mutex_t g_oled_lock = PTHREAD_MUTEX_INITIALIZER;

/* ============================================================
 *  帧队列 (线程安全)
 * ============================================================ */
typedef struct {
    uint8_t *data;
    size_t   size;
    int64_t  frame_id;
} queued_frame_t;

typedef struct {
    queued_frame_t frames[QUEUE_SIZE];
    int             head, tail, count;
    pthread_mutex_t lock;
    pthread_cond_t  cond;
} frame_queue_t;

static frame_queue_t g_queue;

static void queue_init(frame_queue_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->cond, NULL);
}

static void queue_put(frame_queue_t *q, const uint8_t *data, size_t size, int64_t id)
{
    pthread_mutex_lock(&q->lock);
    while (q->count >= QUEUE_SIZE && running)
        pthread_cond_wait(&q->cond, &q->lock);
    if (!running) { pthread_mutex_unlock(&q->lock); return; }

    queued_frame_t *f = &q->frames[q->tail];
    f->data     = (uint8_t *)malloc(size);
    f->size     = size;
    f->frame_id = id;
    memcpy(f->data, data, size);
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->lock);
}

static int queue_get(frame_queue_t *q, queued_frame_t *out)
{
    pthread_mutex_lock(&q->lock);
    while (q->count == 0 && running)
        pthread_cond_wait(&q->cond, &q->lock);
    if (!running && q->count == 0) { pthread_mutex_unlock(&q->lock); return -1; }

    *out = q->frames[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count--;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->lock);
    return 0;
}

/* ============================================================
 *  线程 1: 采集
 * ============================================================ */
static void *capture_thread(void *arg)
{
    (void)arg;
    printf("[THREAD] capture started\n");

    while (running) {
        capture_frame_t cap;
        if (capture_get_frame(&cap) != 0) {
            fprintf(stderr, "[THREAD] capture error, retrying...\n");
            usleep(100000);  /* 等100ms重试 */
            continue;
        }
        queue_put(&g_queue, cap.data, cap.size, cap.frame_id);
        recorder_feed(cap.data, cap.size);  /* 存到SD卡 */
    }

    printf("[THREAD] capture stopped\n");
    return NULL;
}

/* ============================================================
 *  线程 2: 推流 (原始 MJPEG → RTSP, AI低分辨率解码)
 * ============================================================ */
static void *push_thread(void *arg)
{
    (void)arg;
    printf("[THREAD] push started\n");

    while (running) {
        queued_frame_t f;
        if (queue_get(&g_queue, &f) != 0) break;

        // 空帧计数 + 过滤
        {
            static int empty_streak = 0;
            if (f.size < 256) {
                empty_streak++;
                if (empty_streak % 100 == 1)
                    printf("[PUSH] skip empty frame streak=%d\n", empty_streak);
                free(f.data);
                continue;
            }
            // 空帧恢复 → 重启mediamtx
            if (empty_streak > 90) {  /* ~3秒 */
                printf("[PUSH] frame recovered after %d empty, restarting mediamtx\n", empty_streak);
                system("killall -9 mediamtx 2>/dev/null; sleep 1; "
                       "/userdata/mediamtx /userdata/mediamtx.yml &");
                printf("[PUSH] mediamtx restarted\n");
            }
            empty_streak = 0;
        }

        /* 更新标注帧缓存 (720p带框) */
        {
            size_t ns = 0;
            const uint8_t *nd = cv_branch_get_annotated_frame(&ns);
            if (nd && ns > 256) {
                pthread_mutex_lock(&g_anno_lock);
                free(g_anno_buf);
                g_anno_buf = (uint8_t *)malloc(ns);
                if (g_anno_buf) { memcpy(g_anno_buf, nd, ns); g_anno_len = ns; }
                else g_anno_len = 0;
                pthread_mutex_unlock(&g_anno_lock);
            }
        }
        /* 推送标注帧 (720p带框), 无缓存时回退原始帧 */
        {
            int pushed = 0;
            pthread_mutex_lock(&g_anno_lock);
            if (g_anno_buf && g_anno_len > 256) {
                gst_pipeline_push_frame(g_anno_buf, g_anno_len, f.frame_id);
                pushed = 1;
            }
            pthread_mutex_unlock(&g_anno_lock);
            if (!pushed)
                gst_pipeline_push_frame(f.data, f.size, f.frame_id);
        }

        // 通知main线程: 第一帧有效数据已进入管线
        if (!g_first_valid_frame) {
            pthread_mutex_lock(&g_start_lock);
            g_first_valid_frame = 1;
            pthread_cond_signal(&g_start_cond);
            pthread_mutex_unlock(&g_start_lock);
        }

        // 同时喂给 AI 分支
        cv_frame_t cvf;
        cvf.data         = f.data;
        cvf.size         = f.size;
        cvf.width        = CAP_WIDTH;
        cvf.height       = CAP_HEIGHT;
        cvf.stride       = 0;
        cvf.format       = CV_FMT_JPEG;
        cvf.timestamp_us = 0;
        cvf.frame_id     = f.frame_id;
        cv_branch_push_frame(&cvf);

        free(f.data);
    }

    printf("[THREAD] push stopped\n");
    return NULL;
}

/* ============================================================
 *  AI 结果回调
 * ============================================================ */
static void on_ai_result(const cv_result_t *result, void *user_data)
{
    (void)user_data;
    if (!result || result->count == 0) {
        pthread_mutex_lock(&g_oled_lock);
        g_oled_detect.has_target = 0;
        pthread_mutex_unlock(&g_oled_lock);
        return;
    }

    printf("[AI] frame %lld: %d detections, elapsed=%lld us\n",
           (long long)result->frame_id, result->count,
           (long long)result->elapsed_us);
    for (int i = 0; i < result->count && i < 8; i++) {
        const cv_detection_t *d = &result->detections[i];
        printf("  [%d] %s: %.2f @ (%d,%d %dx%d)\n",
               d->class_id, d->label, d->confidence,
               d->x, d->y, d->w, d->h);
    }
    /* 更新 OLED 显示数据 */
    pthread_mutex_lock(&g_oled_lock);
    g_oled_detect.has_target = 1;
    g_oled_detect.class_id   = result->detections[0].class_id;
    g_oled_detect.confidence = result->detections[0].confidence;
    pthread_mutex_unlock(&g_oled_lock);
}

/* ============================================================
 *  信号
 * ============================================================ */
static void sig_handler(int sig)
{
    (void)sig;
    running = 0;
}

/* ============================================================
 *  main
 * ============================================================ */
int main(void)
{
    printf("=== RV1126B Smart Camera (3-Thread) ===\n");

    signal(SIGTERM, sig_handler);
    signal(SIGINT,  sig_handler);

    /* ---- 1. V4L2 自动查找摄像头 ---- */
    char cap_device[128];
    if (capture_find_device(cap_device, sizeof(cap_device)) != 0) {
        fprintf(stderr, "FATAL: USB camera not found\n");
        return -1;
    }
    printf("[MAIN] using camera: %s\n", cap_device);
    if (capture_init(cap_device, CAP_WIDTH, CAP_HEIGHT, CAP_FPS) != 0) {
        return -1;
    }

    /* ---- 2. GStreamer (先启动管线，再启动mediamtx) ---- */
    if (gst_pipeline_init(CAP_WIDTH, CAP_HEIGHT, CAP_FPS, H264_BITRATE) != 0) {
        capture_deinit(); return -1;
    }

    /* ---- 3. AI 分支 (RKNN加载较慢) ---- */
    cv_branch_config_t cv_cfg = {
        .max_queue_size    = 2,
        .processing_width  = 0,
        .processing_height = 0,
        .on_result         = on_ai_result,
        .user_data         = NULL,
        .model_path        = "/userdata/best-four-new-i8.rknn",
    };
    cv_branch_init(&cv_cfg);

    /* ---- 4. mediamtx — 将在工作线程启动后调用 (见下方) ---- */

    /* ---- 5. AI 坐标 Socket ---- */
    ai_fifo_init();

    /* ---- 5b. TVOC 传感器 ---- */
    sensor_init();

    /* ---- 5c. OLED 显示屏 ---- */
    oled_display_init();

    /* ---- 5d. 本地录像 (SD卡) ---- */
    recorder_init();

    /* ---- 5e. 舵机追踪线程 ---- */
    servo_thread_init();

    /* ---- 5f. UDP 远程喊话 ---- */
    udp_audio_init();

    /* ---- 6. 启动工作线程 (先推流, 再启动mediamtx) ---- */
    queue_init(&g_queue);
    pthread_t cap_tid, push_tid;
    pthread_create(&cap_tid,  NULL, capture_thread, NULL);
    pthread_create(&push_tid, NULL, push_thread,    NULL);

    /* 等待几帧进入管线后再启动mediamtx, 避免RTP源超时 */
    usleep(500000);  /* 0.5秒, 足够前几帧到达 */

    /* 等待第一帧有效数据进入管线 (最多等15秒) */
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 15;
        pthread_mutex_lock(&g_start_lock);
        while (!g_first_valid_frame && running) {
            if (pthread_cond_timedwait(&g_start_cond, &g_start_lock, &ts) != 0)
                break;
        }
        pthread_mutex_unlock(&g_start_lock);
        printf("[MAIN] first valid frame %s, starting mediamtx\n",
               g_first_valid_frame ? "OK" : "TIMEOUT");
    }

    if (start_mediamtx() != 0) { fprintf(stderr, "FATAL: mediamtx\n"); return -1; }

    /* ---- 6. 统计 + OLED ---- */
    printf("\n=== Running ===\n");

    /* 系统就绪语音 */
    play_voice("/userdata/voices/0.wav");

    /* 等3秒后检测WiFi: 有IP→wifi.wav, 无IP→nowifi.wav */
    sleep(3);
    {
        int has_wifi = 0;
        for (int i = 0; i < 10; i++) {
            FILE *fp = popen("ip addr show wlan0 | grep -q 'inet ' && echo yes || echo no", "r");
            if (fp) { char buf[8]={0}; fgets(buf,sizeof(buf),fp); pclose(fp);
                      if (buf[0]=='y') { has_wifi = 1; break; } }
            usleep(500000);
        }
        play_voice(has_wifi ? "/userdata/voices/wifi.wav" : "/userdata/voices/nowifi.wav");
    }
    int64_t  tick = 0;
    while (running) {
        sleep(1);
        tick++;
        if (tick % 30 == 0) {
            int64_t in, out, drop;
            cv_branch_get_stats(&in, &out, &drop);
            printf("[STATS] cv(in=%lld out=%lld drop=%lld)\n",
                   (long long)in, (long long)out, (long long)drop);
        }
        /* OLED 每 2 秒刷新 */
        if (tick % 2 == 0) {
            pthread_mutex_lock(&g_oled_lock);
            int ht = g_oled_detect.has_target;
            int cid = g_oled_detect.class_id;
            float conf = g_oled_detect.confidence;
            pthread_mutex_unlock(&g_oled_lock);
            float tvoc, ch2o, temp; int co2, alarm;
            ai_fifo_get_sensor(&tvoc, &ch2o, &co2, &alarm, &temp);
            float ss, fs;
            ai_fifo_get_scores(&ss, &fs);
            oled_display_update(ht, cid, conf, tvoc, co2, alarm, ss, fs);
        }
    }

    /* ---- 7. 清理 ---- */
    printf("\n=== Shutting down ===\n");
    pthread_cond_broadcast(&g_queue.cond);
    pthread_join(cap_tid,  NULL);
    pthread_join(push_tid, NULL);
    cv_branch_deinit();
    sensor_deinit();
    oled_display_deinit();
    recorder_deinit();
    servo_thread_deinit();
    udp_audio_deinit();
    ai_fifo_deinit();
    free(g_anno_buf); g_anno_buf = NULL;
    gst_pipeline_deinit();
    capture_deinit();
    stop_mediamtx();
    printf("=== Done ===\n");
    return 0;
}
