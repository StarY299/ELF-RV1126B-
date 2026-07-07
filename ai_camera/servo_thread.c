/**
 * servo_thread.c — 舵机追踪线程 (自动追踪 + UDP手动 + 环境异常扫描)
 *
 * 自动模式: FIFO读取AI坐标 → tracker P-controller → PWM
 * 手动模式: UDP :9997 接收指令 → 直接PWM
 * 扫描模式: 传感器报警→自动启停, 水平往复扫描查找异常源
 *
 * UDP协议 (纯文本, 以\n结尾):
 *   AUTO     → 切换自动追踪
 *   MANUAL   → 切换手动控制
 *   LEFT     → 左转 (←)
 *   RIGHT    → 右转 (→)
 *   UP       → 上转 (↑)
 *   DOWN     → 下转 (↓)
 *   STOP     → 停止手动移动
 *   CENTER   → 归中
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include "fifo_rx.h"
#include "pca9685.h"
#include "tracker.h"
#include "servo_thread.h"
#include "tcp_server.h"   /* ai_fifo_get_sensor */

#define I2C_DEV      "/dev/i2c-4"
#define FIFO_PATH    "/tmp/ai_track.fifo"
#define UDP_PORT     9997
#define PAN          0
#define TILT         1
#define CENTER_US    1500
#define MANUAL_STEP_PAN   40
#define MANUAL_STEP_TILT  25
#define PAN_MIN_US   500
#define PAN_MAX_US   2500
#define TILT_MIN_US  830   /* 1500 - 60°(666us) */
#define TILT_MAX_US  2170  /* 1500 + 60°(666us) */

/* 扫描模式 (180°舵机全范围) */
#define SCAN_PAN_LEFT   600
#define SCAN_PAN_RIGHT  2400
#define SCAN_PAN_STEP   8     /* 每步μs */
#define SCAN_PAUSE_MS   800   /* 到边界停顿ms */

static pthread_t    g_tid;
static volatile int g_running = 0;
static volatile int g_mode = 0;        /* 0=自动 1=手动 2=扫描 */
static volatile int g_manual_cmd = 0;  /* 0=停 1=左 2=右 3=上 4=下 5=归中 */
static volatile int g_scan_cooldown = 0; /* 扫描冷却: 退出扫描后若干周期内禁止重入 */

static void clamp_pwm(int *pan, int *tilt) {
    if (*pan  < PAN_MIN_US)  *pan  = PAN_MIN_US;
    if (*pan  > PAN_MAX_US)  *pan  = PAN_MAX_US;
    if (*tilt < TILT_MIN_US) *tilt = TILT_MIN_US;
    if (*tilt > TILT_MAX_US) *tilt = TILT_MAX_US;
}

static int udp_init(int port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("[SERVO-UDP] socket"); return -1; }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port) };
    a.sin_addr.s_addr = INADDR_ANY;
    if (bind(fd, (struct sockaddr*)&a, sizeof(a)) < 0) {
        perror("[SERVO-UDP] bind"); close(fd); return -1;
    }
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    printf("[SERVO-UDP] listening on :%d (auto/manual)\n", port);
    return fd;
}

static void udp_poll(int fd) {
    char buf[32];
    struct sockaddr_in from; socklen_t slen = sizeof(from);
    int n = recvfrom(fd, buf, sizeof(buf)-1, 0, (struct sockaddr*)&from, &slen);
    if (n <= 0) return;
    buf[n] = 0;
    for (int i = 0; i < n; i++) if (buf[i]=='\n'||buf[i]=='\r') buf[i]=0;

    if      (strcmp(buf, "AUTO") == 0)   { g_mode = 0; g_manual_cmd = 0; printf("[SERVO] → AUTO mode\n"); }
    else if (strcmp(buf, "MANUAL") == 0) { g_mode = 1; g_manual_cmd = 0; printf("[SERVO] → MANUAL mode\n"); }
    else if (strcmp(buf, "LEFT") == 0)    g_manual_cmd = 1;
    else if (strcmp(buf, "RIGHT") == 0)   g_manual_cmd = 2;
    else if (strcmp(buf, "UP") == 0)      g_manual_cmd = 3;
    else if (strcmp(buf, "DOWN") == 0)    g_manual_cmd = 4;
    else if (strcmp(buf, "STOP") == 0)    g_manual_cmd = 0;
    else if (strcmp(buf, "CENTER") == 0)  g_manual_cmd = 5;
    else if (strcmp(buf, "REBOOT") == 0)   { printf("[SERVO] → REBOOT\n"); system("sudo reboot"); }
    else if (strcmp(buf, "SHUTDOWN") == 0) { printf("[SERVO] → SHUTDOWN\n"); system("sudo shutdown -h now"); }
}

static void *servo_loop(void *arg) {
    (void)arg;
    printf("=== Servo Tracker (PAN+TILT) ===\n");

    int pca_ok = (pca9685_init(I2C_DEV, 0x40) == 0);
    if (pca_ok) {
        pca9685_set_servo(PAN,  CENTER_US);
        pca9685_set_servo(TILT, CENTER_US);
        printf("[INIT] PCA ok, both axes active\n");
    } else {
        printf("[WARN] no PCA, dry run\n");
    }

    if (fifo_rx_init(FIFO_PATH) < 0) {
        if (pca_ok) pca9685_close();
        printf("[SERVO] FIFO init failed\n");
        return NULL;
    }

    int udp_fd = udp_init(UDP_PORT);
    tracker_init();
    printf("=== Ready (PAN+TILT, DB=±45px, KP=0.16) ===\n");

    ai_coord_msg_t m;
    int manual_pan = CENTER_US, manual_tilt = CENTER_US;
    int cycle = 0;
    int scan_dir = 1, scan_pan = CENTER_US, scan_pause = 0;
    while (g_running) {
        if (udp_fd >= 0) udp_poll(udp_fd);

        /* 读取传感器和FIFO */
        float _, __, temp; int co2, alarm;
        ai_fifo_get_sensor(&_, &__, &co2, &alarm, &temp);
        int has_fifo = fifo_rx_read(&m);

        /* 环境异常 → 自动启动扫描 (需冷却结束 + 无目标) */
        if (g_mode == 0 && alarm == SENSOR_BAD_AIR &&
            g_scan_cooldown == 0 && (!has_fifo || !m.has_target)) {
            g_mode = 2;
            scan_pan = CENTER_US; scan_dir = 1; scan_pause = 0;
            printf("[SERVO] → SCAN 环境异常\n");
        }

        if (g_mode == 0) {
            /* ---- 自动追踪模式 ---- */
            if (has_fifo == 1) {
                /* ai_coord_msg_t → ai_msg_t 字段对齐 (offset差4B) */
                ai_msg_t tm;
                tm.cx = m.cx; tm.cy = m.cy; tm.w = m.w; tm.h = m.h;
                tm.frame_w = m.frame_w; tm.frame_h = m.frame_h;
                tm.confidence = m.confidence;
                tm.has_target = m.has_target; tm.class_id = m.class_id;

                int pan_us  = tracker_update(&tm, PAN);
                int tilt_us = tracker_update(&tm, TILT);
                if (pca_ok) {
                    if (pan_us  < 0) pca9685_disable_servo(PAN);
                    else             pca9685_set_servo(PAN,  pan_us);
                    if (tilt_us < 0) pca9685_disable_servo(TILT);
                    else             pca9685_set_servo(TILT, tilt_us);
                }
                if (++cycle % 25 == 0) {
                    if (tm.has_target)
                        printf("[TRACK] cx=%d cy=%d err=%+d,%+dpx | pan=%dus tilt=%dus | conf=%.2f\n",
                               tm.cx, tm.cy, tm.cx-tm.frame_w/2, tm.cy-tm.frame_h/2, pan_us, tilt_us, tm.confidence);
                    else
                        printf("[TRACK] no target | pan=%dus tilt=%dus (holding)\n", pan_us, tilt_us);
                }
            }
        } else if (g_mode == 2) {
            /* ---- 扫描模式: 水平往复, 检测到目标→自动追踪 ---- */
            /* 环境恢复 → 切回自动, 清除冷却 */
            if (alarm == SENSOR_OK) {
                g_mode = 0;
                g_scan_cooldown = 0;
                printf("[SERVO] → AUTO 环境恢复\n");
                continue;
            }
            /* ai_coord_msg_t → ai_msg_t 字段对齐 */
            ai_msg_t tm;
            tm.cx = m.cx; tm.cy = m.cy; tm.w = m.w; tm.h = m.h;
            tm.frame_w = m.frame_w; tm.frame_h = m.frame_h;
            tm.has_target = m.has_target; tm.class_id = m.class_id;

            /* 检测到有效目标 → 退出扫描, 切入追踪, 设置冷却防重入 */
            if (has_fifo == 1 && tm.has_target &&
                tm.frame_w > 0 && tm.frame_w <= 4096 &&
                tm.cx >= 0 && tm.cx < tm.frame_w) {
                g_mode = 0;
                g_scan_cooldown = 250;  /* 5秒冷却, 避免FIFO空窗被误判重入扫描 */
                tracker_init();         /* 重置追踪器状态 */
                printf("[SERVO] → TRACK 发现目标 (cooldown=250)\n");
                continue;
            }
            if (scan_pause > 0) { scan_pause--; usleep(20000); continue; }
            scan_pan += scan_dir * SCAN_PAN_STEP;
            if (scan_pan >= SCAN_PAN_RIGHT) {
                scan_pan = SCAN_PAN_RIGHT; scan_dir = -1;
                scan_pause = SCAN_PAUSE_MS / 20;
            } else if (scan_pan <= SCAN_PAN_LEFT) {
                scan_pan = SCAN_PAN_LEFT; scan_dir = 1;
                scan_pause = SCAN_PAUSE_MS / 20;
            }
            clamp_pwm(&scan_pan, &manual_tilt);
            if (pca_ok) {
                pca9685_set_servo(PAN,  scan_pan);
                pca9685_set_servo(TILT, CENTER_US);
            }
            if (++cycle % 25 == 0)
                printf("[SCAN] pan=%dus dir=%d alarm=%d\n", scan_pan, scan_dir, alarm);
        } else {
            /* ---- 手动控制模式 ---- */
            switch (g_manual_cmd) {
                case 1: manual_pan  -= MANUAL_STEP_PAN;  break;
                case 2: manual_pan  += MANUAL_STEP_PAN;  break;
                case 3: manual_tilt += MANUAL_STEP_TILT; break;
                case 4: manual_tilt -= MANUAL_STEP_TILT; break;
                case 5: manual_pan = CENTER_US; manual_tilt = CENTER_US;
                        g_manual_cmd = 0; break;
            }
            clamp_pwm(&manual_pan, &manual_tilt);
            if (pca_ok) {
                pca9685_set_servo(PAN,  manual_pan);
                pca9685_set_servo(TILT, manual_tilt);
            }
            if (++cycle % 25 == 0)
                printf("[MANUAL] pan=%dus tilt=%dus cmd=%d\n", manual_pan, manual_tilt, g_manual_cmd);
        }
        /* 扫描冷却递减 */
        if (g_scan_cooldown > 0) g_scan_cooldown--;
        usleep(20000);
    }

    printf("[EXIT] centering...\n");
    if (pca_ok) {
        pca9685_set_servo(PAN,  CENTER_US);
        pca9685_set_servo(TILT, CENTER_US);
    }
    fifo_rx_close();
    if (udp_fd >= 0) close(udp_fd);
    if (pca_ok) pca9685_close();
    printf("=== Done ===\n");
    return NULL;
}

int servo_thread_init(void) {
    g_running = 1;
    if (pthread_create(&g_tid, NULL, servo_loop, NULL) != 0) {
        perror("[SERVO] pthread_create"); g_running = 0; return -1;
    }
    return 0;
}
void servo_thread_deinit(void) {
    g_running = 0; pthread_join(g_tid, NULL);
}
