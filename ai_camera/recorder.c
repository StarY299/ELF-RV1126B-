/**
 * recorder.c — 板端本地录像 (独立JPEG, 720p, SD卡)
 *
 * 每2帧存1帧 (15fps), 720p分辨率, 30分钟一个文件夹
 * 独立线程+队列: capture线程不阻塞, OpenCV处理在后台
 * 文件夹: YYYYMMDD_HHMM-HHMM / 文件名: YYYYMMDD_HHMMSS.jpg
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <errno.h>
#include <dirent.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "recorder.h"

#define RECORD_DIR       "/run/media/mmcblk1p1/recordings"
#define RECORD_KEEP_HOURS 24
#define CAPTURE_EVERY    2
#define SPLIT_MINUTES    30
#define OUT_WIDTH        1280
#define OUT_HEIGHT       720
#define JPEG_QUALITY     75
#define QUEUE_MAX        16

/* 队列项 */
struct FrameItem { uint8_t *data; size_t size; };
static FrameItem      g_queue[QUEUE_MAX];
static int            g_q_head=0, g_q_tail=0, g_q_count=0;
static pthread_mutex_t g_q_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_q_cond = PTHREAD_COND_INITIALIZER;

static pthread_t    g_tid;
static volatile int g_running = 0;
static int          g_frame_cnt = 0;
static char         g_cur_dir[256] = {0};
static time_t       g_dir_start = 0;

static void ensure_dir(const char *path) {
    struct stat st;
    if (stat(path, &st) != 0) mkdir(path, 0755);
}

static int open_folder(time_t now) {
    g_dir_start = now - (now % (SPLIT_MINUTES * 60));
    time_t dir_end = g_dir_start + SPLIT_MINUTES * 60;
    struct tm ts, te;
    localtime_r(&g_dir_start, &ts);
    localtime_r(&dir_end, &te);
    snprintf(g_cur_dir, sizeof(g_cur_dir),
             "%s/%04d%02d%02d_%02d%02d-%02d%02d", RECORD_DIR,
             ts.tm_year+1900, ts.tm_mon+1, ts.tm_mday,
             ts.tm_hour, ts.tm_min, te.tm_hour, te.tm_min);
    ensure_dir(RECORD_DIR);
    ensure_dir(g_cur_dir);
    return 0;
}

static void save_frame(const uint8_t *jpg_data, size_t size) {
    time_t now = time(NULL);
    if (g_cur_dir[0] == 0 || (now - g_dir_start) >= SPLIT_MINUTES * 60)
        open_folder(now);

    try {
        std::vector<uint8_t> in(jpg_data, jpg_data + size);
        cv::Mat src = cv::imdecode(in, cv::IMREAD_COLOR);
        if (src.empty()) return;
        cv::Mat dst;
        cv::resize(src, dst, cv::Size(OUT_WIDTH, OUT_HEIGHT), 0, 0, cv::INTER_LINEAR);

        struct tm t; localtime_r(&now, &t);
        char path[512];
        snprintf(path, sizeof(path), "%s/%04d%02d%02d_%02d%02d%02d.jpg",
                 g_cur_dir, t.tm_year+1900, t.tm_mon+1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);

        std::vector<int> prm = { cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY };
        std::vector<uint8_t> out;
        cv::imencode(".jpg", dst, out, prm);
        FILE *fp = fopen(path, "wb");
        if (fp) { fwrite(out.data(), 1, out.size(), fp); fclose(fp); }
    } catch (const cv::Exception &e) {
        fprintf(stderr, "[REC] cv err: %s\n", e.what());
    }
}

static void cleanup_old(void) {
    DIR *d = opendir(RECORD_DIR); if (!d) return;
    time_t cutoff = time(NULL) - RECORD_KEEP_HOURS * 3600;
    struct dirent *ent;
    while ((ent = readdir(d))) {
        if (ent->d_name[0] == '.') continue;
        char path[640]; snprintf(path, sizeof(path), "%s/%s", RECORD_DIR, ent->d_name);
        struct stat st;
        if (stat(path, &st) == 0 && S_ISDIR(st.st_mode) && st.st_mtime < cutoff) {
            char cmd[1024]; snprintf(cmd, sizeof(cmd), "rm -rf \"%s\"", path);
            system(cmd);
            printf("[REC] del: %s\n", ent->d_name);
        }
    }
    closedir(d);
}

/* 后台线程: 从队列取帧 → OpenCV处理 → 存盘 */
static void *recorder_thread(void *arg) {
    (void)arg;
    printf("[REC] started %dx%d %dfps keep=%dh split=%dmin\n",
           OUT_WIDTH, OUT_HEIGHT, 30/CAPTURE_EVERY, RECORD_KEEP_HOURS, SPLIT_MINUTES);
    time_t last_cl = time(NULL);
    while (g_running) {
        pthread_mutex_lock(&g_q_lock);
        while (g_q_count == 0 && g_running)
            pthread_cond_wait(&g_q_cond, &g_q_lock);
        if (!g_running) { pthread_mutex_unlock(&g_q_lock); break; }
        FrameItem it = g_queue[g_q_head];
        g_q_head = (g_q_head + 1) % QUEUE_MAX;
        g_q_count--;
        pthread_mutex_unlock(&g_q_lock);

        save_frame(it.data, it.size);
        free(it.data);

        time_t now = time(NULL);
        if (now - last_cl >= 60) { cleanup_old(); last_cl = now; }
    }
    /* 清空队列 */
    pthread_mutex_lock(&g_q_lock);
    while (g_q_count > 0) {
        free(g_queue[g_q_head].data);
        g_q_head = (g_q_head+1) % QUEUE_MAX; g_q_count--;
    }
    pthread_mutex_unlock(&g_q_lock);
    return NULL;
}

int recorder_init(void) {
    ensure_dir(RECORD_DIR);
    cleanup_old();
    g_running = 1;
    if (pthread_create(&g_tid, NULL, recorder_thread, NULL) != 0) {
        perror("[REC] pthread"); g_running = 0; return -1;
    }
    return 0;
}

/* capture线程调用: 只拷贝数据到队列, 不阻塞 */
void recorder_feed(const uint8_t *jpeg_data, size_t size) {
    if (!g_running || !jpeg_data || size < 100) return;
    g_frame_cnt++;
    if (g_frame_cnt % CAPTURE_EVERY != 0) return;

    uint8_t *cp = (uint8_t*)malloc(size);
    if (!cp) return;
    memcpy(cp, jpeg_data, size);

    pthread_mutex_lock(&g_q_lock);
    if (g_q_count >= QUEUE_MAX) {
        free(g_queue[g_q_head].data);
        g_q_head = (g_q_head+1) % QUEUE_MAX;
        g_q_count--;
    }
    g_queue[g_q_tail].data = cp;
    g_queue[g_q_tail].size = size;
    g_q_tail = (g_q_tail+1) % QUEUE_MAX;
    g_q_count++;
    pthread_cond_signal(&g_q_cond);
    pthread_mutex_unlock(&g_q_lock);
}

void recorder_deinit(void) {
    g_running = 0;
    pthread_cond_signal(&g_q_cond);
    pthread_join(g_tid, NULL);
    printf("[REC] stopped\n");
}
