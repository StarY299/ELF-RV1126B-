/** PCA9685 舵机驱动 (I2C) — 带重试机制 */
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "pca9685.h"

#define MODE1     0x00
#define PRESCALE  0xFE
#define LED0      0x06
#define FREQ      50
#define PERIOD_US (1000000/FREQ)

static int fd = -1;

/* 带重试的 I2C 写入 (PCA9685 上电需稳定时间) */
static int pca_write(const uint8_t *buf, int len) {
    for (int r = 0; r < 10; r++) {
        if (write(fd, buf, len) == len) {
            if (r > 0) printf("[PCA] retry %d OK\n", r);
            return 0;
        }
        usleep(30000);
    }
    perror("i2c write");
    return -1;
}

int pca9685_init(const char *dev, uint8_t addr) {
    fd = open(dev, O_RDWR);
    if (fd < 0) { perror("i2c open"); return -1; }

    /* ioctl 也可能需要重试 */
    for (int r = 0; r < 10; r++) {
        if (ioctl(fd, I2C_SLAVE, addr) >= 0) break;
        if (r == 9) { perror("i2c ioctl"); close(fd); fd = -1; return -1; }
        usleep(30000);
    }

    uint8_t buf[2];

    /* 复位 (首次写入需要重试) */
    buf[0] = MODE1; buf[1] = 0x00;
    if (pca_write(buf, 2) < 0) { close(fd); fd = -1; return -1; }
    usleep(10000);

    /* 50Hz */
    float ps = 25e6f / 4096 / FREQ - 1;
    buf[0] = MODE1;    buf[1] = 0x10;                  pca_write(buf, 2);
    buf[0] = PRESCALE; buf[1] = (uint8_t)(ps + 0.5f);  pca_write(buf, 2);
    buf[0] = MODE1;    buf[1] = 0x00;                  pca_write(buf, 2);
    usleep(5000);
    buf[0] = MODE1;    buf[1] = 0xA1;                  pca_write(buf, 2);

    printf("[PCA] init OK %s@0x%02x\n", dev, addr);
    return 0;
}

void pca9685_close(void) { if (fd >= 0) close(fd); }

void pca9685_set_servo(uint8_t ch, int us) {
    if (fd < 0 || ch > 15) return;
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    uint16_t t = (uint32_t)us * 4096 / PERIOD_US;
    uint8_t r = LED0 + 4 * ch, b[5] = { r, 0, 0, t & 0xFF, t >> 8 };
    pca_write(b, 5);
}

void pca9685_disable_servo(uint8_t ch) {
    if (fd < 0 || ch > 15) return;
    uint8_t r = LED0 + 4 * ch, b[5] = { r, 0, 0, 0, 0 };
    pca_write(b, 5);
}
