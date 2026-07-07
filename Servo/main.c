/** Servo Auto Tracker — 双轴(PAN+TILT)稳定追踪版 */
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "fifo_rx.h"
#include "pca9685.h"
#include "tracker.h"

#define I2C_DEV   "/dev/i2c-4"
#define FIFO_PATH "/tmp/ai_track.fifo"
#define PAN   0
#define TILT  1
#define CENTER_US 1500

static volatile int run = 1;
static void sig(int s) { (void)s; run = 0; }

int main(void) {
    signal(SIGTERM, sig); signal(SIGINT, sig);
    printf("=== Servo Tracker (PAN+TILT) ===\n");

    /* 初始化 PCA9685 */
    int pca_ok = (pca9685_init(I2C_DEV, 0x40) == 0);
    if (pca_ok) {
        pca9685_set_servo(PAN,  CENTER_US);
        pca9685_set_servo(TILT, CENTER_US);
        printf("[INIT] PCA ok, both axes active\n");
    } else {
        printf("[WARN] no PCA, dry run\n");
    }

    /* 初始化 FIFO 和追踪器 */
    if (fifo_rx_init(FIFO_PATH) < 0) {
        if (pca_ok) pca9685_close();
        return -1;
    }
    tracker_init();
    printf("=== Ready (PAN+TILT, DB=±45px, KP=0.16) ===\n");

    ai_msg_t m;
    int cycle = 0;
    while (run) {
        if (fifo_rx_read(&m) == 1) {
            /* 有目标时校验数据合理性，无目标直接放行 */
            if (m.has_target) {
                if (m.frame_w <= 0 || m.frame_w > 4096 ||
                    m.frame_h <= 0 || m.frame_h > 4096 ||
                    m.cx < 0 || m.cx >= m.frame_w ||
                    m.cy < 0 || m.cy >= m.frame_h) {
                    printf("[WARN] bad data: cx=%d cy=%d fw=%d fh=%d — skipped\n",
                           m.cx, m.cy, m.frame_w, m.frame_h);
                    continue;
                }
            }

            /* 双轴追踪 */
            int pan_us  = tracker_update(&m, PAN);
            int tilt_us = tracker_update(&m, TILT);
            if (pca_ok) {
                if (pan_us < 0) pca9685_disable_servo(PAN);
                else            pca9685_set_servo(PAN, pan_us);
                if (tilt_us < 0) pca9685_disable_servo(TILT);
                else             pca9685_set_servo(TILT, tilt_us);
            }

            /* 前 5 次收到目标时打印原始数据，方便调试 */
            if (m.has_target && cycle < 5) {
                printf("[DBG] raw: cx=%d cy=%d fw=%d fh=%d conf=%.2f has=%d\n",
                       m.cx, m.cy, m.frame_w, m.frame_h,
                       m.confidence, m.has_target);
            }

            /* 每 25 周期 (~0.5s) 打印一次状态，避免刷屏 */
            if (++cycle % 25 == 0) {
                if (m.has_target) {
                    int cx = m.cx, cy = m.cy, fw = m.frame_w, fh = m.frame_h;
                    int err_x = cx - fw / 2;
                    int err_y = cy - fh / 2;
                    printf("[TRACK] cx=%d cy=%d err=%+d,%+dpx | pan=%dus tilt=%dus | conf=%.2f\n",
                           cx, cy, err_x, err_y, pan_us, tilt_us, m.confidence);
                } else {
                    printf("[TRACK] no target | pan=%dus tilt=%dus (holding)\n",
                           pan_us, tilt_us);
                }
            }
        }
        usleep(20000);  /* 50Hz */
    }

    /* 安全退出：舵机归中 */
    printf("[EXIT] centering...\n");
    if (pca_ok) {
        pca9685_set_servo(PAN,  CENTER_US);
        pca9685_set_servo(TILT, CENTER_US);
    }
    fifo_rx_close();
    if (pca_ok) pca9685_close();
    printf("=== Done ===\n");
    return 0;
}
